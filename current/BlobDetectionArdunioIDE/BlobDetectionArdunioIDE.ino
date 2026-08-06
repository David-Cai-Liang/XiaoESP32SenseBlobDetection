#include "Arduino.h"
#include "esp_camera.h"
#include "img_converters.h"
#include <vector>
#include <math.h>

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

static const int MAX_W = 320;
static const int MAX_H = 240;

struct LabThreshold {
  int l_min, l_max, a_min, a_max, b_min, b_max;
};

// Tune these exact thresholds using your hardware JPEG tuner script
static const LabThreshold THRESHOLD_BLIMP = {20, 60, 10, 50, 1, 30};

static const uint32_t AREA_THRESHOLD_LOCKED = 25;
static const uint32_t AREA_THRESHOLD_SEARCH = 50;
static const int ROI_PADDING = 30;
static const int SCAN_STEP = 1;

struct Rect { int x, y, w, h; };
struct Blob { int x, y, w, h, cx, cy; uint32_t pixels; };

struct __attribute__((packed)) TelemetryData {
  uint16_t hb;
  uint16_t roi_x, roi_y, roi_w, roi_h;
  uint16_t cx, cy, w, h;
  uint16_t max_w, max_h;
};

struct __attribute__((packed)) FrameHeader {
  uint8_t magic[4];     // {0xFF, 0xAA, 0x55, 0xFF}
  uint32_t payload_len;
  TelemetryData telem;
};

Rect tracking_roi = {0, 0, MAX_W, MAX_H};
bool target_locked = false;
uint16_t heartbeat = 1;

static uint8_t threshold_lut[65536];
static uint8_t *visited_buf = NULL;
static uint32_t *stack_buf = NULL;
static uint8_t *rgb_work_buf = NULL; // PSRAM buffer for decoded hardware JPEG

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

void applyColorMask(camera_fb_t *fb) {
  uint16_t *buf = (uint16_t *)fb->buf;
  size_t total_pixels = fb->width * fb->height;
  for (size_t i = 0; i < total_pixels; i++) {
    buf[i] = threshold_lut[buf[i]] ? 0xFFFF : 0x0000;
  }
}

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

static inline bool inBounds(int x, int y) {
  return x >= 0 && x < MAX_W && y >= 0 && y < MAX_H;
}

void drawOverlays(camera_fb_t *fb, Rect roi, Blob blob, bool locked) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;

  uint16_t COLOR_ROI = locked ? 0x07E0 : 0xFFE0;
  uint16_t COLOR_BLOB = 0xF800;
  // ROI Bounding Box
  for (int t = 0; t < 2; t++) {
    for (int x = roi.x - t; x < roi.x + roi.w + t; x++) {
      if (inBounds(x, roi.y - 1 - t)) buf[(roi.y - 1 - t) * stride + x] = COLOR_ROI;
      if (inBounds(x, roi.y + roi.h + t)) buf[(roi.y + roi.h + t) * stride + x] = COLOR_ROI;
    }
    for (int y = roi.y - t; y < roi.y + roi.h + t; y++) {
      if (inBounds(roi.x - 1 - t, y)) buf[y * stride + (roi.x - 1 - t)] = COLOR_ROI;
      if (inBounds(roi.x + roi.w + t, y)) buf[y * stride + (roi.x + roi.w + t)] = COLOR_ROI;
    }
  }

  if (blob.pixels > 0) {
      // Blob Bounding Box
    for (int x = blob.x; x < blob.x + blob.w; x++) {
      if (inBounds(x, blob.y)) buf[blob.y * stride + x] = COLOR_BLOB;
      if (inBounds(x, blob.y + blob.h)) buf[(blob.y + blob.h) * stride + x] = COLOR_BLOB;
    }
    for (int y = blob.y; y < blob.y + blob.h; y++) {
      if (inBounds(blob.x, y)) buf[y * stride + blob.x] = COLOR_BLOB;
      if (inBounds(blob.x + blob.w, y)) buf[y * stride + (blob.x + blob.w)] = COLOR_BLOB;
    }
    // BLob Center X
    for (int d = -4; d <= 4; d++) {
      if (inBounds(blob.cx + d, blob.cy)) buf[blob.cy * stride + (blob.cx + d)] = COLOR_BLOB;
      if (inBounds(blob.cx, blob.cy + d)) buf[(blob.cx + d) * stride + blob.cx] = COLOR_BLOB;
    }
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
  config.pixel_format = PIXFORMAT_JPEG; // Enable Hardware JPEG mode
  config.jpeg_quality = 20;             // Internal hardware compression quality
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_whitebal(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 1);
    s->set_aec_value(s, 40);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  setupCamera();

  size_t max_pixels = MAX_W * MAX_H;
  if (psramFound()) {
    visited_buf  = (uint8_t *)ps_malloc(max_pixels);
    stack_buf    = (uint32_t *)ps_malloc(max_pixels * sizeof(uint32_t));
    rgb_work_buf = (uint8_t *)ps_malloc(max_pixels * 2); // 153.6 KB RGB565 working buffer
  } else {
    visited_buf  = (uint8_t *)malloc(max_pixels);
    stack_buf    = (uint32_t *)malloc(max_pixels * sizeof(uint32_t));
    rgb_work_buf = (uint8_t *)malloc(max_pixels * 2);
  }

  initThresholdLUT();
}
void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  // Decode hardware JPEG directly into RGB565 working buffer
  if (jpg2rgb565(fb->buf, fb->len, rgb_work_buf, JPG_SCALE_NONE)) {
    camera_fb_t work_fb;
    work_fb.buf = rgb_work_buf;
    work_fb.len = MAX_W * MAX_H * 2;
    work_fb.width = MAX_W;
    work_fb.height = MAX_H;
    work_fb.format = PIXFORMAT_RGB565;

    Blob largest;
    if (target_locked) {
      largest = findLargestBlob(&work_fb, tracking_roi, AREA_THRESHOLD_LOCKED);
    } else {
      Rect full = {0, 0, MAX_W, MAX_H};
      largest = findLargestBlob(&work_fb, full, AREA_THRESHOLD_SEARCH);
    }

    TelemetryData telem;

    if (largest.pixels > 0) {
      heartbeat = 3 - heartbeat;

      int roi_x = max(0, largest.x - ROI_PADDING);
      int roi_y = max(0, largest.y - ROI_PADDING);
      int roi_w = min(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
      int roi_h = min(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

      tracking_roi = {roi_x, roi_y, roi_w, roi_h};
      target_locked = true;

      telem = {
        heartbeat,
        (uint16_t)roi_x, (uint16_t)roi_y, (uint16_t)roi_w, (uint16_t)roi_h,
        (uint16_t)largest.cx, (uint16_t)largest.cy, (uint16_t)largest.w, (uint16_t)largest.h,
        (uint16_t)MAX_W, (uint16_t)MAX_H
      };
    } else {
      target_locked = false;
      tracking_roi = {0, 0, MAX_W, MAX_H};
      telem = {0, 0, 0, 0, 0, 0, 0, 0, 0, (uint16_t)MAX_W, (uint16_t)MAX_H};
    }

    // Uncomment if testing binary threshold vision mask
    // applyColorMask(&work_fb);

    // Draw overlays on native Little-Endian buffer
    drawOverlays(&work_fb, tracking_roi, largest, target_locked);

    // --- ENDIANNESS FIX FOR fmt2jpg ---
    // jpg2rgb565 outputs Little-Endian RGB565, but fmt2jpg expects Big-Endian.
    // Byte-swap pixel endianness right before re-encoding to JPEG.
    uint16_t *buf16 = (uint16_t *)rgb_work_buf;
    size_t total_pixels = MAX_W * MAX_H;
    for (size_t i = 0; i < total_pixels; i++) {
      buf16[i] = __builtin_bswap16(buf16[i]);
    }
    // ---------------------------------

    // Encode modified frame buffer back to JPEG for streaming
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool converted = fmt2jpg(work_fb.buf, work_fb.len, MAX_W, MAX_H, PIXFORMAT_RGB565, 20, &jpg_buf, &jpg_len);

    if (converted) {
      FrameHeader header;
      header.magic[0] = 0xFF; header.magic[1] = 0xAA;
      header.magic[2] = 0x55; header.magic[3] = 0xFF;
      header.payload_len = jpg_len;
      header.telem = telem;

      Serial.write((uint8_t *)&header, sizeof(header));
      Serial.write(jpg_buf, jpg_len);
      free(jpg_buf);
    }
  }

  esp_camera_fb_return(fb);
}