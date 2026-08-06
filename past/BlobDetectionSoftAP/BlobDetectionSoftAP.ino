// Balloon Race Mode - ESP32-S3 Visualized Tracker
// Combines RGB565 blob tracking, on-frame overlay drawing,
// Wi-Fi SoftAP hosting, and live MJPEG HTTP video streaming.

#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <vector>
#include <math.h>

// ---------------------------------------------------------
// Wi-Fi Access Point Configuration
// ---------------------------------------------------------
const char* AP_SSID = "ESP32-Tracker";
const char* AP_PASSWORD = "Password123"; // Must be >= 8 characters (or NULL for open network)

httpd_handle_t stream_httpd = NULL;
volatile bool stream_active = false;

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
// RGB565 Byte-Swapped Color Definitions for ESP32 Camera Buffer
// ---------------------------------------------------------
#define COLOR_RED    0x00F8
#define COLOR_GREEN  0xE007
#define COLOR_BLUE   0x1F00
#define COLOR_YELLOW 0xE0FF
#define COLOR_CYAN   0xFF07

// ---------------------------------------------------------
// Frame / Tracking Parameters
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
// Memory Buffers & LUT
// ---------------------------------------------------------
static uint8_t threshold_lut[65536];
static uint8_t *visited_buf = NULL;
static uint32_t *stack_buf = NULL;

// ---------------------------------------------------------
// Math Helpers & Threshold LUT
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

void initThresholdLUT() {
  for (uint32_t px = 0; px < 65536; px++) {
    int L, A, B;
    rgb565ToLab((uint16_t)px, L, A, B);
    threshold_lut[px] = inThreshold(L, A, B, THRESHOLD_BLIMP) ? 1 : 0;
  }
}

// ---------------------------------------------------------
// Fast Blob Detection
// ---------------------------------------------------------
Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;
  int rw = roi.w, rh = roi.h;

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
// Frame Overlay Drawing Helpers
// ---------------------------------------------------------
inline void drawPixel(uint16_t *buf, int img_w, int img_h, int x, int y, uint16_t color) {
  if (x >= 0 && x < img_w && y >= 0 && y < img_h) {
    buf[y * img_w + x] = color;
  }
}

void drawRect(uint16_t *buf, int img_w, int img_h, int x, int y, int w, int h, uint16_t color) {
  for (int i = x; i < x + w; i++) {
    drawPixel(buf, img_w, img_h, i, y, color);
    drawPixel(buf, img_w, img_h, i, y + h - 1, color);
  }
  for (int j = y; j < y + h; j++) {
    drawPixel(buf, img_w, img_h, x, j, color);
    drawPixel(buf, img_w, img_h, x + w - 1, j, color);
  }
}

void drawCrosshair(uint16_t *buf, int img_w, int img_h, int cx, int cy, int size, uint16_t color) {
  for (int i = -size; i <= size; i++) {
    drawPixel(buf, img_w, img_h, cx + i, cy, color);
    drawPixel(buf, img_w, img_h, cx, cy + i, color);
  }
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
// Core Processing & Visualization Routine
// ---------------------------------------------------------
void processAndAnnotateFrame(camera_fb_t *fb) {
  Blob largest;
  if (target_locked) {
    largest = findLargestBlob(fb, tracking_roi, AREA_THRESHOLD_LOCKED);
  } else {
    Rect full = {0, 0, MAX_W, MAX_H};
    largest = findLargestBlob(fb, full, AREA_THRESHOLD_SEARCH);
  }

  uint16_t *frame_buf = (uint16_t *)fb->buf;

  if (largest.pixels > 0) {
    heartbeat = 3 - heartbeat;

    int roi_x = max(0, largest.x - ROI_PADDING);
    int roi_y = max(0, largest.y - ROI_PADDING);
    int roi_w = min(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
    int roi_h = min(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

    // Draw active ROI search box (Cyan)
    drawRect(frame_buf, MAX_W, MAX_H, tracking_roi.x, tracking_roi.y, tracking_roi.w, tracking_roi.h, COLOR_CYAN);

    // Draw detected object bounding box (Green)
    drawRect(frame_buf, MAX_W, MAX_H, largest.x, largest.y, largest.w, largest.h, COLOR_GREEN);

    // Draw target centroid crosshair (Red)
    drawCrosshair(frame_buf, MAX_W, MAX_H, largest.cx, largest.cy, 5, COLOR_RED);

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
}

// ---------------------------------------------------------
// HTTP MJPEG Stream Handler
// ---------------------------------------------------------
esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
  if (res != ESP_OK) return res;

  stream_active = true;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    // Process tracking logic and draw overlays directly into frame
    processAndAnnotateFrame(fb);

    // Convert drawn RGB565 buffer to JPEG
    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &_jpg_buf, &_jpg_buf_len);
    esp_camera_fb_return(fb);

    if (!jpeg_converted) {
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(part_buf, 64, "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 10);

    free(_jpg_buf);
    _jpg_buf = NULL;

    if (res != ESP_OK) break;
  }

  stream_active = false;
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t stream_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

// ---------------------------------------------------------
// Network & Camera Setup
// ---------------------------------------------------------
void setupWifiAP() {
  WiFi.mode(WIFI_AP);
  bool result = WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  if (result) {
    Serial.println("Wi-Fi Access Point Started!");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("Stream URL: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start Wi-Fi Access Point.");
  }
}

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
  }
}

// ---------------------------------------------------------
// Setup & Main Loop
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();

  setupCamera();

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

  setupWifiAP();
  startCameraServer();

  delay(2000);
}

static float avg_loop_ms = 0.0f;

void loop() {
  // If no active web browser is connected, run tracking via standard loop
  if (!stream_active) {
    uint32_t loop_start_us = micros();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      return;
    }

    processAndAnnotateFrame(fb);
    esp_camera_fb_return(fb);

    uint32_t loop_us = micros() - loop_start_us;
    float loop_ms = loop_us / 1000.0f;
    avg_loop_ms = (avg_loop_ms == 0.0f) ? loop_ms : (avg_loop_ms * 0.9f + loop_ms * 0.1f);
    float fps = (loop_ms > 0) ? (1000.0f / loop_ms) : 0.0f;
    float avg_fps = (avg_loop_ms > 0) ? (1000.0f / avg_loop_ms) : 0.0f;

    Serial.printf("Loop: %.1fms (%.1ffps)  AVG: %.1fms (%.1ffps)\n",
                  loop_ms, fps, avg_loop_ms, avg_fps);
  } else {
    // When browser stream is active, stream_handler owns frame capture
    delay(50);
  }
}