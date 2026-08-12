# ESP-FLY Blimp

A vision-tracking, IMU-stabilized blimp controlled wirelessly from a keyboard, built on two ESP32 boards linked over **ESP-NOW** and bridged to a PC over **USB serial**.

```
 ┌──────────────┐   USB Serial    ┌────────────────┐   ESP-NOW (2.4GHz)   ┌──────────────┐
 │ base_station │ <────wired────> │  Base Station  │ <──────wireless─────>│    Blimp     │
 │     .py      │  (keyboard in,  │    (ESP32)     │   telemetry / ctrl   │   (ESP32)    │
 │   (your PC)  │  telemetry out) │base_station.ino│                      │  blimp.ino   │
 └──────────────┘                 └────────────────┘                      └──────┬───────┘
                                                                                 │
                                                                     ┌───────────┼────────────┐
                                                                     │           │            │
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
  VisionData vision; // cx, cy, w, h (4x uint16_t) — blob centroid + ROI box size
  IMUData imu;       // ax, ay, az, tz (4x float)  — accel XYZ + gyro Z
} TelemetryPacket;

// Base station -> Blimp
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // M1, M2, M3, M4
} ControlPacket;
```

`vision.cx`/`vision.cy` are the tracked blob's true centroid in frame coordinates (0–320, 0–240 on the QVGA camera); `vision.w`/`vision.h` are the padded tracking ROI's dimensions, useful for display but not for locating the target itself.

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

> **Before compiling either sketch:** copy the `bidirectional` folder into your Arduino `libraries` folder (e.g. `~/Documents/Arduino/libraries/` on most OSes, `Documents\Arduino\libraries\` on Windows). Both `blimp.ino` and `base_station.ino` depend on the shared code in it, and the Arduino IDE/CLI won't find it otherwise.

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

## Control modes (compile-time select)

`blimp.ino` supports two motor-control modes, chosen at **compile time** via a `#define` near the top of the file — flip it and reflash to switch:

```cpp
#define MODE_MANUAL       0
#define MODE_PROPORTIONAL 1
#define CONTROL_MODE MODE_PROPORTIONAL   // <-- change this + reflash to switch modes
```

Because this is resolved by the preprocessor (`#if CONTROL_MODE == ...`), only the selected mode's code is actually compiled in.

### `MODE_MANUAL`

Motors are driven directly from the base station's `ControlPacket` — i.e. whatever `base_station.py`'s keyboard input computed (see [Controls](#controls) below).

### `MODE_PROPORTIONAL`

Incoming manual stick input is ignored. Instead, a proportional (P) controller steers **yaw only** (left/right turning) off the vision blob's centroid:

- **Deadzone:** a 40×40 px box centered on the 320×240 frame (only the x-extent, ±20 px around `cx = 160`, is used since this controller only corrects yaw).
- **Gain:** for every pixel the centroid's x-coordinate sits outside the deadzone, the corresponding motor's power increases by `YAW_GAIN` (1).
- **Direction:** target right of center → boost `M2` (Rear Right, mirrors the `D` key); target left of center → boost `M3` (Rear Left, mirrors the `A` key). This mirrors the manual key bindings but hasn't been flight-verified — if the blimp turns away from the target instead of toward it, swap the `m2`/`m3` assignment in `blimp.ino`.
- All motor outputs are clamped to `[0, 255]` (`MOTOR_MAX`) before `analogWrite`.

If no target is currently locked (`vData.w == 0 && vData.h == 0`), the controller applies no correction and the blimp sits at zero thrust rather than steering blind.

**This mode still respects the control-link watchdog** (see below) — losing contact with the base station zeroes the motors regardless of what the camera sees, so there's always a way to kill the blimp by cutting the base station's link, even in autonomous mode. This does mean `base_station.py` (or something) must still be actively sending `ControlPacket`s at ~20 Hz for the blimp to leave the "stale" state, even though those packets' motor values are discarded in this mode — if nothing is sending, the blimp stays at zero thrust indefinitely.

> **`base_station.ino` must be flashed and powered on `MODE_PROPORTIONAL` too.** It's not just a manual-mode relay — it's the source of the heartbeat packets the blimp's watchdog needs to leave the "stale" state (see below). Without it running, the blimp will sit at zero thrust even with a target locked in view.

## Vision tracking

`Vision.cpp` converts each captured frame to CIELAB color space via a precomputed 64K-entry lookup table, thresholds against a configured `LabThreshold`, and flood-fills to find the largest connected blob above an area threshold. Once a target is found it locks onto a padded ROI around it (`AREA_THRESHOLD_LOCKED`) for cheaper tracking on subsequent frames, and falls back to a full-frame search (`AREA_THRESHOLD_SEARCH`) if the target is lost. Tune the color threshold in `Vision.h`'s `THRESHOLD_BLIMP` constant for your target's color under your lighting.

The blob's true centroid (`largest.cx`/`largest.cy` from `findLargestBlob`) is what gets transmitted as `VisionData.cx`/`cy` and is what the proportional yaw controller reacts to — it is not the same as the padded tracking ROI's corner or size (`w`/`h`), which exist for telemetry/display only.

## Safety: control-link failsafe

`blimp.ino` tracks `lastRecvTime` (updated in `OnDataRecv`, which fires on any received `ControlPacket`) and treats the link as **stale** if more than `CONTROL_TIMEOUT_MS` (currently 1000 ms) has passed since the last packet arrived:

```cpp
bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);
```

`stale` acts as a heartbeat check, independent of control mode:

- **`MODE_MANUAL`:** if stale, motor values are forced to zero instead of using the (possibly ancient) `incomingControl` values.
- **`MODE_PROPORTIONAL`:** if stale, the yaw controller doesn't run at all — motors stay at zero — even if the camera can still see a target. This guarantees a human always has a way to stop the blimp (by killing the base station link) even while it's flying itself.

If the base station crashes, loses power, goes out of range, or a packet is simply dropped over ESP-NOW, the blimp zeroes its own motors once the timeout elapses — it doesn't depend on a shutdown command actually arriving. `base_station.py`'s `Ctrl+C` handler (which flushes and resends an all-zero `ControlPacket` a few times) is a fast-path on top of this, not the only thing standing between "quit" and "still flying."

Note the 1000 ms timeout is fairly loose relative to the ~20 Hz (50 ms) control rate — worth tightening (e.g. 200–300 ms) if you want the blimp to fail-safe faster after a lost link.

## Known limitations / suggested hardening

- `esp_now_send` calls in both sketches don't check their return status, so a failed send is currently silent.
- `CONTROL_TIMEOUT_MS` (1000 ms) is generous compared to the control loop's ~50 ms cadence; a tighter timeout would reduce how long the blimp can drift on a stale command before self-stopping.
- The proportional yaw controller's turn direction (`M2` vs `M3` for a given error sign) is inferred from the manual key bindings, not confirmed in flight — verify and flip if needed.
- The 40×40 px deadzone and `YAW_GAIN` of 1 are untuned starting values; expect to adjust both once you see how the blimp actually responds.
