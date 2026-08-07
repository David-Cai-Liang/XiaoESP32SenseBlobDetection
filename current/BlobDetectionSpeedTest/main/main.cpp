#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "esp_camera.h"
#include "img_converters.h"

#include "esp_vfs_usb_serial_jtag.h"

// --- Hardware Pins ---
#define LED_GPIO_NUM   (gpio_num_t)21 // XIAO ESP32S3 User LED (Active LOW)

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

#define MAX_W 320
#define MAX_H 240
#define MAX_STACK_SIZE 4096 // 4096 entries (16 KB in SRAM) is ample for Scanline Flood Fill

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct LabThreshold {
  int l_min, l_max, a_min, a_max, b_min, b_max;
};

static const LabThreshold THRESHOLD_BLIMP = {20, 60, 10, 50, 1, 30};

static const uint32_t AREA_THRESHOLD_LOCKED = 20;
static const uint32_t AREA_THRESHOLD_SEARCH = 40;
static const int ROI_PADDING = 30;
static const int SCAN_STEP = 1;

struct Rect { int x, y, w, h; };
struct Blob { int x, y, w, h, cx, cy; uint32_t pixels; };

typedef struct __attribute__((packed)) {
  uint16_t hb;
  uint16_t roi_x, roi_y, roi_w, roi_h;
  uint16_t cx, cy, w, h;
  uint16_t max_w, max_h;
} TelemetryData;

typedef struct __attribute__((packed)) {
  uint8_t magic[4];     // {0xFF, 0xAA, 0x55, 0xFF}
  uint32_t timestamp;
  TelemetryData telem;
} FrameHeader;

static Rect tracking_roi = {0, 0, MAX_W, MAX_H};
static bool target_locked = false;
static uint16_t heartbeat = 1;

static uint8_t threshold_lut[65536];
static uint8_t *mask_buf = NULL;     // Allocated in Internal SRAM (76.8 KB)
static uint32_t *stack_buf = NULL;   // Allocated in Internal SRAM (16 KB)
static uint8_t *rgb_work_buf = NULL; // Allocated in PSRAM (153.6 KB)

static inline float srgbToLinear(float c) {
  return (c <= 0.04045f) ? (c / 12.92f) : powf((c + 0.055f) / 1.055f, 2.4f);
}

static inline float labF(float t) {
  return (t > 0.008856f) ? cbrtf(t) : (7.787f * t + 16.0f / 116.0f);
}

static inline bool inThreshold(int L, int A, int B, const LabThreshold &t) {
  return L >= t.l_min && L <= t.l_max &&
         A >= t.a_min && A <= t.a_max &&
         B >= t.b_min && B <= t.b_max;
}

void initThresholdLUT() {
  float r_lin[32], g_lin[64], b_lin[32];
  for (int r = 0; r < 32; r++) r_lin[r] = srgbToLinear(r / 31.0f);
  for (int g = 0; g < 64; g++) g_lin[g] = srgbToLinear(g / 63.0f);
  for (int b = 0; b < 32; b++) b_lin[b] = srgbToLinear(b / 31.0f);

  for (uint32_t px = 0; px < 65536; px++) {
    uint8_t r5 = (px >> 11) & 0x1F;
    uint8_t g6 = (px >> 5) & 0x3F;
    uint8_t b5 = px & 0x1F;

    float r = r_lin[r5];
    float g = g_lin[g6];
    float b = b_lin[b5];

    float X = (r * 0.4124f + g * 0.3576f + b * 0.1805f) / 0.95047f;
    float Y = (r * 0.2126f + g * 0.7152f + b * 0.0722f);
    float Z = (r * 0.0193f + g * 0.1192f + b * 0.9505f) / 1.08883f;

    float fx = labF(X), fy = labF(Y), fz = labF(Z);

    int L = (int)(116.0f * fy - 16.0f);
    int A = (int)(500.0f * (fx - fy));
    int B = (int)(200.0f * (fy - fz));

    threshold_lut[px] = inThreshold(L, A, B, THRESHOLD_BLIMP) ? 1 : 0;
  }
}

// Pre-mask pass streaming pixels sequentially into fast internal SRAM
void generateSramMask(const camera_fb_t *fb, Rect roi) {
  const uint16_t *buf = (const uint16_t *)fb->buf;
  int stride = fb->width;

  for (int ry = 0; ry < roi.h; ry++) {
    int img_y = roi.y + ry;
    int row_idx = img_y * stride;
    for (int rx = 0; rx < roi.w; rx++) {
      int img_x = roi.x + rx;
      int idx = row_idx + img_x;
      mask_buf[idx] = threshold_lut[buf[idx]] ? 1 : 0;
    }
  }
}

Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  int stride = fb->width;
  int rw = roi.w, rh = roi.h;

  generateSramMask(fb, roi);

  Blob best = {0, 0, 0, 0, 0, 0, 0};

  uint32_t sampled_threshold = area_threshold / (SCAN_STEP * SCAN_STEP);
  if (sampled_threshold < 1) sampled_threshold = 1;

  for (int ry = 0; ry < rh; ry += SCAN_STEP) {
    int img_y = roi.y + ry;
    int row_idx = img_y * stride;

    for (int rx = 0; rx < rw; rx += SCAN_STEP) {
      int img_x = roi.x + rx;
      int idx = row_idx + img_x;

      if (mask_buf[idx] != 1) continue;

      int stack_size = 0;
      stack_buf[stack_size++] = ((uint32_t)ry << 16) | (uint16_t)rx;

      long sumX = 0, sumY = 0;
      uint32_t count = 0;
      int minX = rx, maxX = rx, minY = ry, maxY = ry;

      // --- Scanline Flood Fill Loop ---
      while (stack_size > 0) {
        uint32_t pos = stack_buf[--stack_size];
        int seed_x = pos & 0xFFFF;
        int seed_y = pos >> 16;

        int row_offset = (roi.y + seed_y) * stride + roi.x;
        if (mask_buf[row_offset + seed_x] != 1) continue;

        // Scan left boundary
        int lx = seed_x;
        while (lx > 0 && mask_buf[row_offset + lx - 1] == 1) {
          lx--;
        }

        // Scan right boundary
        int rx_span = seed_x;
        while (rx_span < rw - 1 && mask_buf[row_offset + rx_span + 1] == 1) {
          rx_span++;
        }

        // Process horizontal span & clear mask
        for (int x = lx; x <= rx_span; x++) {
          mask_buf[row_offset + x] = 0;
          count++;
          sumX += x;
          sumY += seed_y;
        }

        if (lx < minX) minX = lx;
        if (rx_span > maxX) maxX = rx_span;
        if (seed_y < minY) minY = seed_y;
        if (seed_y > maxY) maxY = seed_y;

        // Push seed points for adjacent lines (above and below)
        const int dys[2] = {-1, 1};
        for (int i = 0; i < 2; i++) {
          int ny = seed_y + dys[i];
          if (ny < 0 || ny >= rh) continue;

          int n_row_offset = (roi.y + ny) * stride + roi.x;
          bool in_span = false;

          for (int x = lx; x <= rx_span; x++) {
            if (mask_buf[n_row_offset + x] == 1) {
              if (!in_span) {
                if (stack_size < MAX_STACK_SIZE) {
                  stack_buf[stack_size++] = ((uint32_t)ny << 16) | (uint16_t)x;
                }
                in_span = true;
              }
            } else {
              in_span = false;
            }
          }
        }
      }

      if (count >= sampled_threshold && count > best.pixels) {
        best.pixels = count;
        best.x = roi.x + minX;
        best.y = roi.y + minY;
        best.w = maxX - minX + 1;
        best.h = maxY - minY + 1;
        best.cx = roi.x + (int)(sumX / count);
        best.cy = roi.y + (int)(sumY / count);
      }
    }
  }
  return best;
}

void errorLoop() {
  while (true) {
    gpio_set_level(LED_GPIO_NUM, 0); // LED ON
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LED_GPIO_NUM, 1); // LED OFF
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setupCamera() {
  camera_config_t config = {};
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 20;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    errorLoop(); // Fast blink on camera init fail
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

void processingTask(void *pvParameters) {
  while (1) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      taskYIELD();
      continue;
    }

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

        int roi_x = MAX(0, largest.x - ROI_PADDING);
        int roi_y = MAX(0, largest.y - ROI_PADDING);
        int roi_w = MIN(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
        int roi_h = MIN(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

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

      FrameHeader header;
      header.magic[0] = 0xFF; header.magic[1] = 0xAA;
      header.magic[2] = 0x55; header.magic[3] = 0xFF;
      header.timestamp = esp_log_timestamp();
      header.telem = telem;

      // Send telemetry header stream directly to raw unbuffered stdout
      fwrite((const void *)&header, 1, sizeof(header), stdout);
      fflush(stdout);
    }

    esp_camera_fb_return(fb);
    taskYIELD();
  }
}

extern "C" void app_main(void) {
  // 1. Disable CRLF translation on stdout so raw binary bytes are not altered
  esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
  esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_LF);

  // 2. Setup User LED
  gpio_reset_pin(LED_GPIO_NUM);
  gpio_set_direction(LED_GPIO_NUM, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_GPIO_NUM, 0); // Solid ON during boot

  // 3. Set stdout to unbuffered binary mode for raw streaming
  setvbuf(stdout, NULL, _IONBF, 0);

  vTaskDelay(pdMS_TO_TICKS(500));

  // 4. Initialize Camera
  setupCamera();

  size_t total_pixels = MAX_W * MAX_H;

  // 5. Allocation Strategy: Move heavy random-access buffers to Internal SRAM for max speed
  mask_buf     = (uint8_t *)heap_caps_malloc(total_pixels, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  stack_buf    = (uint32_t *)heap_caps_malloc(MAX_STACK_SIZE * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  rgb_work_buf = (uint8_t *)heap_caps_malloc(total_pixels * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!mask_buf || !stack_buf || !rgb_work_buf) {
    errorLoop();
  }

  initThresholdLUT();

  esp_log_level_set("*", ESP_LOG_NONE);

  xTaskCreatePinnedToCore(processingTask, "proc_task", 16384, NULL, 5, NULL, 1);
}
