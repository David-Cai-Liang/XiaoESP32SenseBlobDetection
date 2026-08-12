# ESP-FLY Blimp

A vision-tracking, IMU-stabilized blimp controlled wirelessly from a keyboard, built on two ESP32 boards linked over **ESP-NOW** and bridged to a PC over **USB serial**.

```
 ┌─────────────┐   USB Serial    ┌────────────────┐   ESP-NOW (2.4GHz)   ┌──────────────┐
 │ base_station │ <────wired────>│  Base Station   │ <──────wireless─────>│    Blimp     │
 │    .py       │  (keyboard in, │    (ESP32)      │   telemetry / ctrl    │   (ESP32)    │
 │  (your PC)   │  telemetry out)│  base_station.ino│                      │  blimp.ino   │
 └─────────────┘                 └────────────────┘                      └──────┬───────┘
                                                                                  │
                                                                     ┌────────────┼────────────┐
                                                                     │            │             │
                                                                Vision (cam)  IMU (MPU6050)  4x Motors
                                                              Vision.cpp/h   IMU.cpp/h      (analogWrite)
```

## How it works

1. **Blimp** runs an onboard camera + IMU loop, packages the readings into a `TelemetryPacket`, and blasts it to the base station over ESP-NOW every loop iteration. It also listens for `ControlPacket` motor commands from the base station and drives the 4 motors via `analogWrite`.
2. **Base station (ESP32)** is a dumb relay: it forwards every `TelemetryPacket` it receives from the blimp out over USB serial (framed with a header/footer), and forwards any `ControlPacket` it reads from serial out to the blimp over ESP-NOW.
3. **base_station.py** runs on your PC, reads WASD-style key state, computes motor values at ~20 Hz, and writes them to the base station over serial. It also parses incoming telemetry frames, tracks round-trip frame timing, and prints a live status/latency readout to the terminal.

## Repository contents

| File | Runs on | Purpose |
|---|---|---|
| `blimp.ino` | Blimp ESP32 | Vision + IMU sensor loop, motor output, ESP-NOW telemetry/control |
| `Vision.h` / `Vision.cpp` | Blimp ESP32 | OV-series camera capture, Lab-color threshold blob tracking, ROI locking |
| `IMU.h` / `IMU.cpp` | Blimp ESP32 | MPU6050 accelerometer/gyro readout |
| `base_station.ino` | Base station ESP32 | Serial ⇄ ESP-NOW protocol bridge (no processing) |
| `base_station.py` | PC | Keyboard control, serial framing, live telemetry/latency dashboard |

## Hardware

**Blimp ESP32 (ESP-FLY board)**

| Motor | Function | GPIO |
|---|---|---|
| M1 | Front Right | 7 |
| M2 | Rear Right | 4 |
| M3 | Rear Left | 3 |
| M4 | Front Left | 1 |

- Camera: OV-series module wired per `Vision.h` pin map (XCLK=10, PCLK=13, VSYNC=38, HREF=47, SIOD=40, SIOC=39, D0–D7 as defined), status LED on GPIO 21.
- IMU: MPU6050 on I²C, SDA=5, SCL=6, address `0x68`.

**Base station ESP32**

- Any ESP32 board with USB serial to the PC. No sensors — it's purely a bridge.

## Wireless protocol (ESP-NOW)

Two fixed-size, `packed` structs are exchanged directly as ESP-NOW payloads:

```cpp
// Blimp -> Base station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4x uint16_t) — locked ROI / blob box
  IMUData imu;       // ax, ay, az, tz (4x float)  — accel XYZ + gyro Z
} TelemetryPacket;

// Base station -> Blimp
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // M1, M2, M3, M4
} ControlPacket;
```

Each side's `esp_now_add_peer` must point at the other's MAC address — set these in both `.ino` files before flashing:

- `base_station.ino` → `blimpAddress[]`
- `blimp.ino` → `baseStationAddress[]`

## Serial protocol (base station ⇄ PC)

The base station re-frames each `TelemetryPacket` for transport over serial, and unwraps `ControlPacket`s the same way:

```
Telemetry (Base Station -> PC), 30 bytes total:
  [ 4B header: 00 AA 55 FF ] [ 24B payload: 4x uint16 + 4x float ] [ 2B footer: EE FF ]

Control (PC -> Base Station), 12 bytes total:
  [ 4B header: 00 BB 66 FF ] [ 8B payload: 4x int16 motor values ]
```

`base_station.py` re-syncs to the header on every parse pass, so it tolerates dropped/partial bytes on the serial line.

## Setup

### 1. Flash the firmware (Arduino IDE / arduino-cli)

Required libraries: `esp_now`, `WiFi`, `esp_camera` (ESP32 board package), `Adafruit_MPU6050`, `Adafruit_Sensor`, `Wire`.

1. Find each board's MAC address (e.g. `WiFi.macAddress()` in a throwaway sketch).
2. Set `blimpAddress[]` in `base_station.ino` and `baseStationAddress[]` in `blimp.ino`.
3. Flash `blimp.ino` (with `Vision.h/.cpp`, `IMU.h/.cpp` alongside it) to the blimp board.
4. Flash `base_station.ino` to the base station board and plug it into your PC via USB.

### 2. Run the ground control script

```bash
pip install pyserial pynput
python base_station.py
```

Edit `SERIAL_PORT` at the top of `base_station.py` first (e.g. `COM9` on Windows, `/dev/ttyUSB0` / `/dev/ttyACM0` on Linux/macOS) and confirm `BAUD_RATE` matches both `.ino` files (`115200`).

### Controls

| Key | Effect |
|---|---|
| `W` | +50 to M2 and M3 |
| `A` | +50 to M3 |
| `D` | +50 to M2 |
| `Q` | +50 to M4 |
| `E` | +50 to M1 |
| `Ctrl+C` | Stop — sends an all-zero motor command and exits |

M1 carries a constant idle offset of `10` even with no keys held (see `compute_motors()`); every other motor idles at `0`.

The terminal shows live motor state, the blimp's tracked vision blob (center/box), IMU readings, and round-trip telemetry latency/FPS. On exit it prints a benchmark summary (frame count, average delta, jitter, throughput).

## Vision tracking

`Vision.cpp` converts each captured frame to CIELAB color space via a precomputed 64K-entry lookup table, thresholds against a configured `LabThreshold`, and flood-fills to find the largest connected blob above an area threshold. Once a target is found it locks onto a padded ROI around it (`AREA_THRESHOLD_LOCKED`) for cheaper tracking on subsequent frames, and falls back to a full-frame search (`AREA_THRESHOLD_SEARCH`) if the target is lost. Tune the color threshold in `Vision.h`'s `THRESHOLD_BLIMP` constant for your target's color under your lighting.

## Safety: control-link failsafe

`blimp.ino` tracks `lastRecvTime` (updated in `OnDataRecv`) and treats the last command as **stale** if more than `CONTROL_TIMEOUT_MS` (currently 1000 ms) has passed since a `ControlPacket` was received:

```cpp
bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

if (newControlAvailable || stale) {
  newControlAvailable = false;
  int16_t m1 = stale ? 0 : incomingControl.motors[0];
  int16_t m2 = stale ? 0 : incomingControl.motors[1];
  int16_t m3 = stale ? 0 : incomingControl.motors[2];
  int16_t m4 = stale ? 0 : incomingControl.motors[3];
  ...
}
```

If the base station crashes, loses power, goes out of range, or a packet is simply dropped over ESP-NOW, the blimp zeroes its own motors once the timeout elapses — it doesn't depend on a shutdown command actually arriving. `base_station.py`'s `Ctrl+C` handler (which flushes and resends an all-zero `ControlPacket` a few times) is a fast-path on top of this, not the only thing standing between "quit" and "still flying."

Note the 1000 ms timeout is fairly loose relative to the ~20 Hz (50 ms) control rate — worth tightening (e.g. 200–300 ms) if you want the blimp to fail-safe faster after a lost link.

## Known limitations / suggested hardening

- `esp_now_send` calls in both sketches don't check their return status, so a failed send is currently silent.
- `CONTROL_TIMEOUT_MS` (1000 ms) is generous compared to the control loop's ~50 ms cadence; a tighter timeout would reduce how long the blimp can drift on a stale command before self-stopping.
