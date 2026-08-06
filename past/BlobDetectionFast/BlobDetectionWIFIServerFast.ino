// Balloon Race Mode - ESP32 port of largestredblob.py (OpenMV/Nicla)
// Camera bring-up follows the CameraWebServer example. Blob search, ROI
// locking, and telemetry framing mirror the original Python script's
// structure and logic. VL53L1X / LSM6DSOX sensor code has been removed
// per request - this replicates only the camera/blob-tracking behavior.

#include "Arduino.h"
#include "esp_camera.h"
#include <vector>
#include <math.h>

// ---------------------------------------------------------
// Camera pins - hard-coded for Xiao ESP32S3 Sense
// ---------------------------------------------------------
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

// ---------------------------------------------------------
// Telemetry UART
// ---------------------------------------------------------
#define TELEM_TX_PIN 43
#define TELEM_RX_PIN 44
HardwareSerial TelemUART(1);

// ---------------------------------------------------------
// Frame / tracking parameters
// ---------------------------------------------------------
static const int MAX_W = 320;
static const int MAX_H = 240;

struct LabThreshold {
  int l_min, l_max, a_min, a_max, b_min, b_max;
};

static const LabThreshold THRESHOLD_BLIMP = {15, 80, 15, 70, 0, 40};


static const uint32_t AREA_THRESHOLD_LOCKED = 400;
static const uint32_t AREA_THRESHOLD_SEARCH = 800;
static const int ROI_PADDING = 30;
static const int SCAN_STEP = 2;

struct Rect {
  int x, y, w, h;
};

struct Blob {
  int x, y, w, h;
  int cx, cy;
  uint32_t pixels;
};

Rect tracking_roi = {0, 0, MAX_W, MAX_H};
bool target_locked = false;
uint16_t heartbeat = 1;

// ---------------------------------------------------------
// Pre-allocated Memory Buffers & LUT
// ---------------------------------------------------------
static uint8_t threshold_lut[65536];
static uint8_t *visited_buf = NULL;
static uint32_t *stack_buf = NULL;

// ---------------------------------------------------------
// Math Helpers for Startup LUT Calculation
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

// Builds LUT at setup time to allow O(1) threshold checking per pixel
void initThresholdLUT() {
  for (uint32_t px = 0; px < 65536; px++) {
    int L, A, B;
    rgb565ToLab((uint16_t)px, L, A, B);
    threshold_lut[px] = inThreshold(L, A, B, THRESHOLD_BLIMP) ? 1 : 0;
  }
}

// ---------------------------------------------------------
// Fast Connected-Component Search (Optimized Flood Fill)
// ---------------------------------------------------------
Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;
  int rw = roi.w, rh = roi.h;

  // Clear visited state for current ROI dimensions
  memset(visited_buf, 0, rw * rh);

  Blob best = {0, 0, 0, 0, 0, 0, 0};

  uint32_t sampled_threshold = area_threshold / (SCAN_STEP * SCAN_STEP);
  if (sampled_threshold < 1) sampled_threshold = 1;

  for (int ry = 0; ry < rh; ry += SCAN_STEP) {
    int v_row = ry * rw;
    int img_y = (roi.y + ry) * stride;

    for (int rx = 0; rx < rw; rx += SCAN_STEP) {
      int vidx = v_row + rx;
      if (visited_buf[vidx]) continue;
      visited_buf[vidx] = 1;

      uint16_t px = buf[img_y + roi.x + rx];
      if (!threshold_lut[px]) continue;

      // Iterative DFS flood-fill using preallocated flat array
      int stack_size = 0;
      stack_buf[stack_size++] = ((uint32_t)ry << 16) | (uint16_t)rx;

      long sumX = 0, sumY = 0;
      uint32_t count = 0;
      int minX = rx, maxX = rx, minY = ry, maxY = ry;

      while (stack_size > 0) {
        uint32_t pos = stack_buf[--stack_size];
        int cx0 = pos & 0xFFFF;
        int cy0 = pos >> 16;

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
          int nx = cx0 + dx[d];
          int ny = cy0 + dy[d];

          if (nx < 0 || ny < 0 || nx >= rw || ny >= rh) continue;

          int nidx = ny * rw + nx;
          if (visited_buf[nidx]) continue;
          visited_buf[nidx] = 1;

          uint16_t n_px = buf[(roi.y + ny) * stride + (roi.x + nx)];
          if (threshold_lut[n_px]) {
            stack_buf[stack_size++] = ((uint32_t)ny << 16) | (uint16_t)nx;
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
// Telemetry Output
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

  Serial.print("[");
  for (int i = 0; i < 11; i++) {
    Serial.print(msg[i]);
    if (i < 10) Serial.print(", ");
  }
  Serial.println("]");
}

// ---------------------------------------------------------
// Setup & Initialization
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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    s->set_whitebal(s, 0);       // Disable Auto White Balance
    s->set_exposure_ctrl(s, 0);  // Disable Auto Exposure
    s->set_gain_ctrl(s, 0);      // Disable Auto Gain Control
    s->set_aec_value(s, 120);    // Manual exposure value (0-1200)
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  setupCamera();

  // Allocate tracking working buffers once (uses PSRAM if available)
  size_t max_pixels = MAX_W * MAX_H;
  if (psramFound()) {
    visited_buf = (uint8_t *)ps_malloc(max_pixels);
    stack_buf = (uint32_t *)ps_malloc(max_pixels * sizeof(uint32_t));
  } else {
    visited_buf = (uint8_t *)malloc(max_pixels);
    stack_buf = (uint32_t *)malloc(max_pixels * sizeof(uint32_t));
  }

  initThresholdLUT();

  TelemUART.begin(115200, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);

  delay(2000);
}

// ---------------------------------------------------------
// Main Execution Loop
// ---------------------------------------------------------
static float avg_loop_ms = 0.0f;

void loop() {
  uint32_t loop_start_us = micros();

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
    target_locked = false;
    tracking_roi = {0, 0, MAX_W, MAX_H};
    sendTelemetry(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  esp_camera_fb_return(fb);

  uint32_t loop_us = micros() - loop_start_us;
  float loop_ms = loop_us / 1000.0f;
  avg_loop_ms = (avg_loop_ms == 0.0f) ? loop_ms : (avg_loop_ms * 0.9f + loop_ms * 0.1f);
  float fps = (loop_ms > 0) ? (1000.0f / loop_ms) : 0.0f;
  float avg_fps = (avg_loop_ms > 0) ? (1000.0f / avg_loop_ms) : 0.0f;

  Serial.printf("Loop: %.1fms (%.1ffps)  AVG: %.1fms (%.1ffps)\n",
                loop_ms, fps, avg_loop_ms, avg_fps);
}