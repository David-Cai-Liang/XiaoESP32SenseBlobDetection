// Balloon Race Mode - ESP32 port of largestredblob.py (OpenMV/Nicla)
// Camera bring-up follows the CameraWebServer example (board_config.h /
// camera_pins.h). Blob search, ROI locking, and telemetry framing mirror
// the original Python script's structure and logic. VL53L1X / LSM6DSOX
// sensor code has been removed per request - this now replicates only
// the camera/blob-tracking behavior.
//
// ---------------------------------------------------------------------
// KNOWN DISCREPANCY FROM THE ORIGINAL SCRIPT (please read):
// largestredblob.py's msg_array had 12 elements (including a `distance`
// field from the removed VL53L1X) but was packed with struct.pack('<11H',
// ...), which only accepts 11 shorts - that would raise struct.error on
// every loop of the original. Now that `distance` is gone, msg_array is
// naturally back down to 11 fields, so sendTelemetry() below packs
// 11 x uint16 (22 bytes, little-endian, no header/checksum), matching
// the original '<11H' format exactly.
// ---------------------------------------------------------------------
//
// ASSUMPTIONS TO VERIFY FOR YOUR HARDWARE:
//   - board_config.h / camera_pins.h are set up as in earlier turns
//     (CAMERA_MODEL_XIAO_ESP32S3).
//   - Telemetry goes out a second UART on TELEM_TX_PIN/TELEM_RX_PIN below;
//     adjust those pin numbers to whatever's actually free on your wiring.

#include "Arduino.h"
#include "esp_camera.h"
#include "board_config.h"
#include <vector>
#include <math.h>

// ---------------------------------------------------------
// Telemetry UART (equivalent of the Python's `uart` object)
// ---------------------------------------------------------
#define TELEM_TX_PIN 43   // <-- verify these are free on your wiring
#define TELEM_RX_PIN 44
HardwareSerial TelemUART(1);

// ---------------------------------------------------------
// Frame / tracking parameters (mirrors max_w, max_h, thresholdBlimp,
// tracking_roi, target_locked from the Python script)
// ---------------------------------------------------------
static const int MAX_W = 320;
static const int MAX_H = 240;

struct LabThreshold {
  int l_min, l_max, a_min, a_max, b_min, b_max;
};
// Ported directly from: thresholdBlimp = (15, 80, 15, 70, 0, 40)
static const LabThreshold THRESHOLD_BLIMP = {15, 80, 15, 70, 0, 40};

static const uint32_t AREA_THRESHOLD_LOCKED = 400;  // matches locked-ROI search
static const uint32_t AREA_THRESHOLD_SEARCH = 800;  // matches full-frame search
static const int ROI_PADDING = 30;
static const int SCAN_STEP = 2;  // pixel stride for speed; 1 = full accuracy, slower

struct Rect {
  int x, y, w, h;
};
struct Blob {
  int x, y, w, h;    // bounding box (top-left + size)
  int cx, cy;        // centroid
  uint32_t pixels;   // matched pixel count (sampled, see SCAN_STEP)
};

Rect tracking_roi = {0, 0, MAX_W, MAX_H};
bool target_locked = false;
uint16_t heartbeat = 1;

// ---------------------------------------------------------
// RGB565 -> CIE L*a*b* (D65), matching the (L, A, B) space
// OpenMV's find_blobs() thresholds against.
// ---------------------------------------------------------
static inline float srgbToLinear(float c) {
  return (c <= 0.04045f) ? (c / 12.92f) : powf((c + 0.055f) / 1.055f, 2.4f);
}

static inline float labF(float t) {
  return (t > 0.008856f) ? cbrtf(t) : (7.787f * t + 16.0f / 116.0f);
}

void rgb565ToLab(uint16_t px, int &L, int &A, int &B) {
  uint8_t r5 = (px >> 11) & 0x1F;
  uint8_t g6 = (px >> 5) & 0x3F;
  uint8_t b5 = px & 0x1F;

  float r = srgbToLinear(r5 / 31.0f);
  float g = srgbToLinear(g6 / 63.0f);
  float b = srgbToLinear(b5 / 31.0f);

  float X = (r * 0.4124f + g * 0.3576f + b * 0.1805f) / 0.95047f;
  float Y = (r * 0.2126f + g * 0.7152f + b * 0.0722f);
  float Z = (r * 0.0193f + g * 0.1192f + b * 0.9505f) / 1.08883f;

  float fx = labF(X), fy = labF(Y), fz = labF(Z);

  L = (int)(116.0f * fy - 16.0f);
  A = (int)(500.0f * (fx - fy));
  B = (int)(200.0f * (fy - fz));
}

static inline bool inThreshold(int L, int A, int B, const LabThreshold &t) {
  return L >= t.l_min && L <= t.l_max &&
         A >= t.a_min && A <= t.a_max &&
         B >= t.b_min && B <= t.b_max;
}

// ---------------------------------------------------------
// Connected-component search within `roi`, equivalent to
// img.find_blobs([thresholdBlimp], roi=..., area_threshold=...,
// merge=True) followed by the Python's own "pick largest" loop.
// Returns the largest qualifying blob (pixels == 0 if none found).
// ---------------------------------------------------------
Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;

  int rw = roi.w, rh = roi.h;
  std::vector<uint8_t> visited(rw * rh, 0);

  Blob best = {0, 0, 0, 0, 0, 0, 0};
  std::vector<int> stackX, stackY;

  // Sampled area threshold scales with SCAN_STEP since we only test
  // every SCAN_STEP-th pixel (approximates the original's full-density count)
  uint32_t sampled_threshold = area_threshold / (SCAN_STEP * SCAN_STEP);
  if (sampled_threshold < 1) sampled_threshold = 1;

  for (int ry = 0; ry < rh; ry += SCAN_STEP) {
    for (int rx = 0; rx < rw; rx += SCAN_STEP) {
      int vidx = ry * rw + rx;
      if (visited[vidx]) continue;
      visited[vidx] = 1;

      int gx = roi.x + rx, gy = roi.y + ry;
      int L, A, B;
      rgb565ToLab(buf[gy * stride + gx], L, A, B);
      if (!inThreshold(L, A, B, THRESHOLD_BLIMP)) continue;

      // Flood-fill this connected component (merge=True behavior)
      stackX.clear();
      stackY.clear();
      stackX.push_back(rx);
      stackY.push_back(ry);

      long sumX = 0, sumY = 0;
      uint32_t count = 0;
      int minX = rx, maxX = rx, minY = ry, maxY = ry;

      while (!stackX.empty()) {
        int cx0 = stackX.back(); stackX.pop_back();
        int cy0 = stackY.back(); stackY.pop_back();

        count++;
        sumX += cx0;
        sumY += cy0;
        if (cx0 < minX) minX = cx0;
        if (cx0 > maxX) maxX = cx0;
        if (cy0 < minY) minY = cy0;
        if (cy0 > maxY) maxY = cy0;

        const int dx[4] = {SCAN_STEP, -SCAN_STEP, 0, 0};
        const int dy[4] = {0, 0, SCAN_STEP, -SCAN_STEP};
        for (int d = 0; d < 4; d++) {
          int nx = cx0 + dx[d], ny = cy0 + dy[d];
          if (nx < 0 || ny < 0 || nx >= rw || ny >= rh) continue;
          int nidx = ny * rw + nx;
          if (visited[nidx]) continue;
          visited[nidx] = 1;

          int nL, nA, nB;
          rgb565ToLab(buf[(roi.y + ny) * stride + (roi.x + nx)], nL, nA, nB);
          if (inThreshold(nL, nA, nB, THRESHOLD_BLIMP)) {
            stackX.push_back(nx);
            stackY.push_back(ny);
          }
        }
      }

      if (count >= sampled_threshold && count > best.pixels) {
        best.pixels = count;
        best.x = roi.x + minX;
        best.y = roi.y + minY;
        best.w = maxX - minX + SCAN_STEP;
        best.h = maxY - minY + SCAN_STEP;
        best.cx = roi.x + (int)(sumX / count);
        best.cy = roi.y + (int)(sumY / count);
      }
    }
  }
  return best;
}

// ---------------------------------------------------------
// Equivalent of Python's IBus_message(): raw little-endian
// uint16 packing, no header/checksum. Now 11 fields (distance
// removed), matching the original's '<11H' format exactly.
// ---------------------------------------------------------
void sendTelemetry(uint16_t hb, int roi_x, int roi_y, int roi_w, int roi_h,
                    int x_val, int y_val, int w_val, int h_val,
                    int max_w, int max_h) {
  uint16_t msg[11] = {
    hb,
    (uint16_t)roi_x, (uint16_t)roi_y, (uint16_t)roi_w, (uint16_t)roi_h,
    (uint16_t)x_val, (uint16_t)y_val, (uint16_t)w_val, (uint16_t)h_val,
    (uint16_t)max_w, (uint16_t)max_h
  };

  uint8_t out[22];
  for (int i = 0; i < 11; i++) {
    out[i * 2]     = msg[i] & 0xFF;
    out[i * 2 + 1] = (msg[i] >> 8) & 0xFF;
  }
  TelemUART.write(out, sizeof(out));

  Serial.print("[");
  for (int i = 0; i < 11; i++) {
    Serial.print(msg[i]);
    if (i < 10) Serial.print(", ");
  }
  Serial.println("]");
}

// ---------------------------------------------------------
// Camera bring-up, lifted from the CameraWebServer example
// (board_config.h / camera_pins.h supply the pin macros).
// RGB565 is required here (not JPEG) since blob search needs
// raw pixel access.
// ---------------------------------------------------------
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;      // 320x240, matches max_w/max_h
  config.pixel_format = PIXFORMAT_RGB565;  // raw pixels needed for blob search
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) delay(1000);
  }

  // sensor.set_vflip(True) / set_hmirror(True) equivalent
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  setupCamera();

  TelemUART.begin(115200, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);

  // sensor.skip_frames(time=2000) equivalent - let auto-exposure settle
  delay(2000);
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(10);
    return;
  }

  Blob largest;
  if (target_locked) {
    largest = findLargestBlob(fb, tracking_roi, AREA_THRESHOLD_LOCKED);
  } else {
    Rect full = {0, 0, MAX_W, MAX_H};
    largest = findLargestBlob(fb, full, AREA_THRESHOLD_SEARCH);
  }

  if (largest.pixels > 0) {
    heartbeat = 3 - heartbeat;

    int roi_x = max(0, largest.x - ROI_PADDING);
    int roi_y = max(0, largest.y - ROI_PADDING);
    int roi_w = min(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
    int roi_h = min(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

    tracking_roi = {roi_x, roi_y, roi_w, roi_h};
    target_locked = true;

    sendTelemetry(heartbeat, roi_x, roi_y, roi_w, roi_h,
                  largest.cx, largest.cy, largest.w, largest.h,
                  MAX_W, MAX_H);
  } else {
    // Target lost: reset to full-frame search next loop
    target_locked = false;
    tracking_roi = {0, 0, MAX_W, MAX_H};
    sendTelemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  esp_camera_fb_return(fb);
}
