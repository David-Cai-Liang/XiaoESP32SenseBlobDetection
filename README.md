## Blob Detection — Xiao ESP32-S3 Sense

Color-blob (LAB thresholded) target detector and tracker running on-device, streaming
telemetry (and optionally annotated JPEG video) over USB.

---

### Hardware

- Board: [Xiao ESP32-S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html)
- Camera sensor: [OV3660](https://datasheet.iiic.cc/datasheets-0/omnivision_technologies/OV03660-A51A.pdf) (default sensor shipped with the board, 65 degree FOV)
- PSRAM is **required** — the camera frame buffer lives in PSRAM; everything else
  (mask buffer, flood-fill stack, threshold LUT) lives in internal SRAM. See
  [Memory layout](#memory-layout) below.
- IMU (`camera+IMU/` builds only): MPU-6050 6-axis IMU (accelerometer + gyro), the same
  IMU used on the [ESP-FLY](https://www.seeedstudio.com/ESP-FLY-DIY-Drone-Kit-p-6408.html)
  IMU / Motor Driver module. `camera+IMU/` reads the MPU-6050 over I2C directly — it does
  not use the ESP-FLY board's motor drivers, and there's no dependency on the rest of
  the ESP-FLY firmware/hardware stack. See [IMU implementations](#imu-implementations-future)
  below.
  - Note: You cannot use the SD card and IMU at the same time because GPIO 8 is used by both.
- Current CAD Model: https://cad.onshape.com/documents/6abb47ea8df48ed88ff41eff/w/fc873cead17ca65b0af647cb/e/1d23eafa7f8f630fb687ec6b
---

### Repository layout

| Path | What it is |
|---|---|
| `camera/BlobDetectionESP-IDF/` | Primary implementation, built with ESP-IDF |
| `camera/BlobDetectionArduinoIDE/` | Arduino IDE port of `camera/BlobDetectionESP-IDF/` — see [ESP-IDF vs Arduino IDE](#esp-idf-vs-arduino-ide-important-differences) before assuming these behave identically |
| `camera/BlobDetectionCalibrate.ino` / `camera/BlobDetectionCalibrate.py` | Interactive LAB threshold calibration tool |
| `camera/ViewDetections.py` | Live viewer for telemetry + (optionally) annotated video stream |
| `camera+IMU/BlobDetectionIMU/` | Single-core build: same blob detector, adds MPU-6050 IMU sampling on the same core/loop |
| `camera+IMU/BlobDetectionIMUArduinoIDE/` | Arduino IDE port of `camera+IMU/BlobDetectionIMU/` — see [ESP-IDF vs Arduino IDE](#esp-idf-vs-arduino-ide-important-differences) before assuming these behave identically |
| `camera+IMU/BlobDetectionIMUMultiCore/` | Dual-core build: Core 0 handles IMU sampling + telemetry, Core 1 handles vision, non-blocking |
| `camera+IMU/BlobDetectionIMUMultiCore/` | Version of `camera+IMU/BlobDetectionIMUMultiCore/` that uses IMU interrupts instead of pulling every 10 milliseconds |
| `camera+IMU/ViewDetectionsWithIMU.py` | Live viewer for the `camera+IMU/` builds' telemetry format (see [Telemetry format](#telemetry-format)) — `camera+IMU/` has no calibration tool of its own; use `camera/BlobDetectionCalibrate.ino` / `.py` to derive `THRESHOLD_BLIMP` values, see below |

`camera+IMU/` is where active development on the IMU-augmented detector happens; `camera/`
is the stable, camera-only baseline described in most of this document.

---

### Calibrating the color mask

1. Flash `camera/BlobDetectionCalibrate.ino` onto the Xiao ESP32S3.
2. Keep the board connected to the computer over USB.
3. Run `camera/BlobDetectionCalibrate.py` on the computer. A window opens showing the
   default mask applied live.
4. Adjust the mask with the following keys:
   | Parameter | Decrease | Increase |
   |---|---|---|
   | `L_min` | `1` | `Q` |
   | `L_max` | `2` | `W` |
   | `A_min` | `3` | `E` |
   | `A_max` | `4` | `R` |
   | `B_min` | `5` | `T` |
   | `B_max` | `6` | `Y` |
5. Every adjustment prints the current mask values to the terminal.
6. The default starting mask lives in `camera/BlobDetectionCalibrate.py` (lines 14–16) —
   edit it there if you want a different starting point for future calibration runs.

`camera+IMU/` does not have its own calibration tool — it relies entirely on `camera/`'s
calibration workflow. Calibrate against `camera/BlobDetectionCalibrate.ino`/`.py`
first, then copy the resulting `THRESHOLD_BLIMP` values into whichever `camera+IMU/` build
you're using (see below).

#### Applying a calibrated mask to the detector

Copy the printed `L_min/L_max/A_min/A_max/B_min/B_max` values into:

```cpp
static const LabThreshold THRESHOLD_BLIMP = { l_min, l_max, a_min, a_max, b_min, b_max };
```

This line exists independently in `camera/BlobDetectionESP-IDF/main.cpp`,
`camera/BlobDetectionArduinoIDE/BlobDetectionArduinoIDE.ino`,
`camera+IMU/BlobDetectionIMU/main.cpp`, `camera+IMU/BlobDetectionIMUMultiCore/main.cpp`,
and `camera+IMU/BlobDetectionIMUMultiCoreInterrupt/main.cpp` —
none of these share a config file, so a recalibration only takes effect on whichever
file you update.

---

### Running the detector

1. Flash one of the blob detector builds (`camera/` ESP-IDF, `camera/` Arduino, or a
   `camera+IMU/` IMU build — see build steps below) onto the Xiao ESP32S3.
2. Keep the board connected over USB.
3. Run the matching `ViewDetections.py` (`camera/ViewDetections.py` for `camera/`
   builds, `camera+IMU/ViewDetections.py` for `camera+IMU/` builds — the telemetry layouts
   differ, see [Telemetry format](#telemetry-format)) on the computer to view live
   telemetry (and video, if `DEBUG_STREAM` is enabled — see [Runtime flags](#runtime-flags)).

---

### Building — ESP-IDF version

```bash
cd camera/BlobDetectionESP-IDF
idf.py fullclean
# Equivalent to manually deleting: build/, managed_components/, dependencies.lock, sdkconfig
idf.py set-target esp32s3
idf.py -p <PORT> build flash
```

**Before flashing:** press the reset button on the board. The flashing tool and
the running firmware both use the USB port, and a stale connection from a
previous run can prevent the flash from starting.

### Building — Arduino IDE version

Open `camera/BlobDetectionArduinoIDE.ino` in Arduino IDE and flash normally.
**Important:** none of the tuning in `sdkconfig.defaults` (see below) applies to this
build — the Arduino IDE uses a separate, prebuilt core and its own build settings. See
[ESP-IDF vs Arduino IDE](#esp-idf-vs-arduino-ide-important-differences) for what
you need to configure manually to get comparable behavior.

### Building — IMU versions (`camera+IMU/`)

`camera+IMU/BlobDetectionIMU`, `camera+IMU/BlobDetectionIMUMultiCore`, and `camera+IMU/BlobDetectionIMUMultiCoreInterrupt` are ESP-IDF
projects, built the same way as `camera/BlobDetectionESP-IDF`:

```bash
cd camera+IMU/BlobDetectionIMU        # or camera+IMU/BlobDetectionIMUMultiCore
idf.py fullclean
idf.py set-target esp32s3
idf.py -p <PORT> build flash
```

Both pull in the `mpu6050` and `i2cdev` components in addition to everything
`camera/BlobDetectionESP-IDF` uses. Same reset-before-flash note as above applies.

---

### Runtime flags

All four builds (`camera/` and `camera+IMU/`) share the same two `#define`s at the top of
the source file:

```cpp
#define DEBUG_STREAM 1        // 1 = annotated JPEG video + telemetry, 0 = telemetry only
#define MASKED_DEBUG_STREAM 1 // 1 = video shows the binary threshold mask, 0 = normal color video with overlays
```

`MASKED_DEBUG_STREAM` only has an effect when `DEBUG_STREAM` is `1`. Note the `camera+IMU/`
builds default to `DEBUG_STREAM 0` (telemetry only), unlike `camera/`'s default of `1`
— check the top of the file for whichever build you're flashing.

| `DEBUG_STREAM` | `MASKED_DEBUG_STREAM` | Output |
|---|---|---|
| 0 | — | Telemetry header only, no image. Fastest, lowest USB bandwidth. |
| 1 | 0 | Full-color annotated video (ROI + blob overlay) + telemetry |
| 1 | 1 | Binary threshold-mask video (what the detector actually "sees") + telemetry, useful for tuning `THRESHOLD_BLIMP` |

For competition/flight use, prefer `DEBUG_STREAM 0` — it removes JPEG encoding
and the associated per-frame CPU/USB cost entirely.

---

### Telemetry format

Every frame begins with a fixed `FrameHeader`, little-endian:

```cpp
struct FrameHeader {
  uint8_t  magic[4];          // {0xFF, 0xAA, 0x55, 0xFF} — frame sync marker
  uint32_t payload_len;       // 0 if telemetry-only; JPEG byte length otherwise
  TelemetryData telem;        // layout differs between camera/ and camera+IMU/, see below
};
```

If `payload_len > 0`, exactly that many bytes of JPEG data immediately follow the
header. `ViewDetections.py` re-syncs on the magic bytes if the stream is ever
corrupted — see [Troubleshooting](#troubleshooting) if you see repeated resync
messages.

**`camera/` — 30-byte `FrameHeader` (22-byte `TelemetryData`):**

```cpp
struct TelemetryData {        // 22 bytes, all uint16_t
  uint16_t hb;                 // heartbeat / last-detection timestamp (ms)
  uint16_t roi_x, roi_y, roi_w, roi_h;   // current tracking ROI
  uint16_t cx, cy;              // detected blob centroid
  uint16_t w, h;                 // detected blob bounding box size
  uint16_t max_w, max_h;         // frame dimensions (320x240), included for host-side sanity checks
};
```

**`camera+IMU/` — 34-byte `FrameHeader` (26-byte `TelemetryData`):**

```cpp
struct __attribute__((packed)) TelemetryData {  // 26 bytes
  uint16_t hb;                 // heartbeat / last-detection timestamp (ms)
  uint16_t cx, cy, w, h;       // detected blob centroid + bounding box size
  float ax, ay, az;            // MPU-6050 acceleration, all 3 axes
  float tz;                    // MPU-6050 yaw rate (gyro Z)
};
```

`camera+IMU/`'s `TelemetryData` is `packed` and swaps the tracking-ROI/frame-size fields
for the four IMU floats — it drops `roi_x/roi_y/roi_w/roi_h` and `max_w/max_h` from
`camera/`'s layout. The two formats are not interchangeable; use `camera/ViewDetections.py`
for `camera/` builds and `camera+IMU/ViewDetections.py` for `camera+IMU/` builds.

---

### IMU implementations (`camera+IMU/`)

`camera+IMU/BlobDetectionIMU` and `camera+IMU/BlobDetectionIMUMultiCore` add an MPU-6050 IMU,
polled over I2C, alongside the same camera-based blob detection pipeline used in
`camera/`. The IMU is **read-only telemetry** in both builds — there is no sensor
fusion with the vision tracking, and neither build drives motors; they only report
`ax, ay, az` (acceleration, all three axes) and `tz` (yaw rate) alongside the existing
blob/ROI telemetry.

**Wiring (both builds):**

| MPU-6050 pin | Xiao ESP32-S3 pin |
|---|---|
| SDA | GPIO5 |
| SCL | GPIO6 |
| I2C address | `0x68` |

I2C is run at 200 kHz. Both builds probe for the MPU-6050 at boot and retry once a
second, logging over serial, until the device responds — the board otherwise waits
indefinitely rather than falling through to `errorLoop()`.

**`BlobDetectionIMU` (single-core):** IMU sampling and blob detection run in the same
loop/core, the same structural pattern as `camera/`'s single `visionTask`-style loop.

**`BlobDetectionIMUMultiCore`:** splits the work across the ESP32-S3's two cores,
non-blocking:
- **Core 0** — `imuTelemetryTask`: samples the MPU-6050 at a fixed 100 Hz
  (`vTaskDelayUntil`-paced) and, when `DEBUG_STREAM` is `0`, emits telemetry frames
  directly at that same 100 Hz rate.
- **Core 1** — `visionTask`: runs the camera capture + blob detection loop as fast as
  frames arrive, and (when `DEBUG_STREAM` is `1`) emits the annotated/masked video
  stream itself.

The two tasks share state (latest blob/ROI, latest IMU sample) through two mutex-guarded
structs (`g_vision_state`, `g_imu_data|VisionState`, `ImuData`) rather than direct
cross-task calls, with short (2 ms) mutex timeouts so neither task blocks waiting on
the other — a missed update just means that frame's telemetry carries the previous
sample instead of a fresh one.

---

### Memory layout

| Buffer | Location | Approx. size (320×240) |
|---|---|---|
| Camera frame buffer | PSRAM | 153.6 KB (RGB565) |
| `threshold_lut` | Internal SRAM | 8 KB (bit-packed, 1 bit/RGB565 value) |
| `mask_buf` | Internal SRAM | ~9.6 KB (bit-packed, 1 bit/pixel) |
| `stack_buf` | Internal SRAM | `MAX_STACK_SIZE * 4` bytes (32 KB at the default 8192 entries) |

`threshold_lut` and `mask_buf` both use **bit-level**, not byte-level, encoding —
one bit per value/pixel rather than one byte. This is a deliberate space
optimization; see the "Fast skip/fill" comments in `findLargestBlob` for how the
flood fill exploits this packing to check/clear 8 pixels per operation instead of 1.

If you run out of internal SRAM, lower `MAX_STACK_SIZE` (8192 → 4096 halves the
stack buffer to 16 KB). See [Flood-fill stack sizing](#flood-fill-stack-sizing)
before doing this blind.

---

### Flood-fill stack sizing

`MAX_STACK_SIZE` bounds how many pending flood-fill spans can be queued at once.
Overflow is **silently truncated**, not fatal — but a truncated fill can split one
real blob into multiple smaller, mis-centered detections, which is a much harder
bug to notice than a crash.

The worst case scales with mask *noise/fragmentation*, not blob size — a solid,
clean blob is cheap; a speckled/glare-affected mask can need far more concurrent
spans. If you need to tune this down for SRAM reasons, instrument it first:
count how often the `stack_size < MAX_STACK_SIZE` check fails in your real
operating conditions (not just a clean bench test) before assuming a smaller
value is safe.

---

### ESP-IDF vs Arduino IDE: important differences

`camera/BlobDetectionESP-IDF` and `camera/BlobDetectionArduinoIDE` run the **same
detection algorithm** (bit-packed mask, scanline flood fill, `IRAM_ATTR`-pinned hot
functions) — but the surrounding infrastructure differs, and the differences matter
for correctness and performance parity. (The `camera+IMU/` builds are ESP-IDF only, so
this section doesn't apply to them.)

- **`sdkconfig.defaults` does not apply to the Arduino build.** Compiler
  optimization level, FreeRTOS tick rate, and console/log configuration are all
  ESP-IDF build-system settings. If you want comparable performance from the
  Arduino build, you need to find and set the equivalent options under Arduino
  IDE's own Tools menu (where available — not every setting has an Arduino-side
  equivalent).
- **Log output is not silenced by default in the Arduino build.** The ESP-IDF
  build explicitly disables logging (`CONFIG_LOG_DEFAULT_LEVEL_NONE`,
  `esp_log_level_set("*", ESP_LOG_NONE)`) so log lines can't interleave with the
  binary telemetry stream. Set Arduino IDE's **Tools → Core Debug Level** to
  "None," or you risk stream corruption from stray log output.
- **Confirm what `Serial` actually maps to** on your board's **Tools → USB Mode**
  setting before relying on the Arduino build's throughput — depending on
  configuration this could be the same USB-Serial-JTAG peripheral the ESP-IDF
  build uses, a separate native USB CDC path, or (if misconfigured) a real UART
  at the literal 115200 baud declared in `Serial.begin()`, which would be far
  too slow for `DEBUG_STREAM 1` video.
- **The Arduino `loop()` never explicitly yields** the way the ESP-IDF
  `processingTask` does (`taskYIELD()` after every frame). This was added on the
  ESP-IDF side specifically to prevent USB stream corruption under load — watch
  for the same symptom on the Arduino build before assuming it's unaffected.

---

### Troubleshooting

**`ViewDetections.py` reports "Misaligned stream: discarded N byte(s)"**
This means the magic-byte sync was lost — usually because bytes were dropped or
corrupted in transit. Common causes, roughly in order of likelihood:
1. Log output (from `ESP_LOG*` or Arduino Core Debug Level) interleaving with the
   binary stream — see the logging notes above.
2. The processing task never yielding the CPU, starving the USB driver's
   internal drain task.
3. Sending faster than the transport can drain (large JPEG payloads in
   `DEBUG_STREAM 1` mode are the most likely trigger) — check that writes aren't
   silently truncated; a partial write followed by the next frame's header will
   desync the stream exactly like this.
4. Using the wrong `ViewDetections.py` for the build flashed — `camera/` and
   `camera+IMU/` telemetry structs are different sizes and layouts (see
   [Telemetry format](#telemetry-format)); pointing the wrong viewer at a build
   will misparse every frame.

**Camera init fails / LED blinks in a fast on/off pattern**
`errorLoop()` triggers on camera init failure or SRAM allocation failure. Check
that PSRAM is enabled in your build config, and that the wiring matches the pin
definitions at the top of the source file.

---

### Design notes

- **Image path:** RGB565 → JPEG. The sensor delivers big-endian RGB565, so the
  threshold LUT and overlay colors are pre-swapped to native/little-endian at
  init time — the mask generation and drawing code then treat everything as
  little-endian internally, and only the final JPEG output re-swaps to the
  format the encoder expects. It's easy to get one of these swaps backwards; if
  colors look wrong after a change, check endianness first.
- **Why LAB instead of HSV:** LAB was chosen to minimize sensitivity to lighting
  changes. HSV is a candidate for a future speed optimization if LAB conversion
  ever becomes a bottleneck (it currently isn't — LAB conversion happens once,
  at LUT-build time, not per frame).
- **Camera frame buffers cannot be packed** (bit-packed) — only `mask_buf` and
  `threshold_lut` are.
- **Loop unrolling:** `generateSramMaskFast` and `applyColorMaskFast` both
  process 8 pixels per iteration to take advantage of the 32-bit data bus.
- **`IRAM_ATTR`** is an ESP-IDF macro that places a function's code in internal
  Instruction RAM instead of leaving it flash-cached — used on the hot,
  per-frame functions (`findLargestBlob`, `generateSramMaskFast`) to avoid
  flash-cache-miss stalls on the tightest loops in the pipeline.
- **`camera+IMU/` IMU builds are camera + IMU only** — they read the MPU-6050 for
  telemetry but do not perform sensor fusion with the vision tracking and do not
  control any motors, unlike the full ESP-FLY flight-controller firmware that
  the IMU/Motor Driver module normally pairs with.

---

### Project history

Rough chronological order, oldest first:

1. **CameraWebServer** — initial reference implementation (Espressif example).
2. **BlobDetection** — first working version of the detector; no LUT precaching.
3. **BlobDetectionSoftAP** — adds a WiFi access point + web server for visual
   debugging, plus LUT precaching.
4. **BlobDetectionFast** — same inputs/outputs as `BlobDetection`, with WiFi and
   the web server removed for speed. Used for initial benchmarking, confirming
   the Xiao ESP32S3 Sense could reach speeds comparable to the Nicla Vision.
5. **AutoCalibrateWithStdOverROI** — first attempt at automatic calibration,
   using mean ± 1 standard deviation over a region of interest to build the
   mask. Uses `viewfinder.py` to align the ROI with the target, and raw RGB565
   rather than JPEG (to avoid the hardware JPEG encoder's denoising).
6. **AutoCalibrateWithStdOverROIWithMasking** — same mask-creation algorithm as
   above, with the mask visualized live via `viewfinder.py` (masking logic
   lives in `AutoCalibrateWithStdOverROIWithMasking.ino`).
7. **BlobDetectionSpeedTest** — a modified `BlobDetectionESP-IDF` with image
   streaming removed and timing instrumentation added. Reached a 130ms loop
   time; paired with `speedTest.py`.
8. **BlobDetectionIMU** — camera-only detector reworked to also poll an
   MPU-6050 IMU (acceleration + yaw rate) on the same core, wired through the
   ESP-FLY IMU/Motor Driver module's MPU-6050. First step toward flight-relevant
   telemetry; no sensor fusion or motor control.
9. **BlobDetectionIMUMultiCore** — same IMU + blob detection pipeline as
   `BlobDetectionIMU`, restructured across both ESP32-S3 cores (IMU/telemetry
   on Core 0, vision on Core 1) so neither task blocks the other.
