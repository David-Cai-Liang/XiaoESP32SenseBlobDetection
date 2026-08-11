#define DEBUG_STREAM 1
#define MASKED_DEBUG_STREAM 1

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "esp_camera.h"
#include "img_converters.h"

#include "esp_vfs_usb_serial_jtag.h"

#include <mpu6050.h>
#include <i2cdev.h>

static const char *TAG = "MAIN";

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
#define MAX_STACK_SIZE 8192 // 4096 entries (32 KB in SRAM) for Scanline Flood Fill

// IMU PINS
#define PIN_SDA GPIO_NUM_5
#define PIN_SCL GPIO_NUM_6
#define MPU_ADDR 0x68

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct LabThreshold {
  int l_min, l_max, a_min, a_max, b_min, b_max;
};

static const LabThreshold THRESHOLD_BLIMP = {20, 60, 10, 50, 1, 30};

static const uint32_t AREA_THRESHOLD_LOCKED = 30;
static const uint32_t AREA_THRESHOLD_SEARCH = 60;
static const int ROI_PADDING = 30;
static const int SCAN_STEP = 1;

mpu6050_dev_t dev;

struct Rect { int x, y, w, h; };
struct Blob { int x, y, w, h, cx, cy; uint32_t pixels; };

typedef struct __attribute__((packed)) {
  uint16_t hb;
  uint16_t cx, cy, w, h;
  float ax, ay, az, tz;
} TelemetryData;

typedef struct __attribute__((packed)) {
  uint8_t magic[4];     // {0xFF, 0xAA, 0x55, 0xFF}
  uint32_t payload_len;
  TelemetryData telem;
} FrameHeader;

// --- Thread-Safe Shared Data Structures ---
typedef struct {
  Blob blob;
  Rect roi;
  bool locked;
  uint16_t heartbeat;
} VisionState;

typedef struct {
  float ax, ay, az, tz;
  bool valid;
} ImuData;

static VisionState g_vision_state = {};
static ImuData g_imu_data = {};

static SemaphoreHandle_t xVisionMutex = NULL;
static SemaphoreHandle_t xImuMutex = NULL;

static Rect tracking_roi = {0, 0, MAX_W, MAX_H};
static bool target_locked = false;
static uint16_t heartbeat = 1;

static uint8_t threshold_lut[8192];
static uint8_t *mask_buf = NULL;     // Internal SRAM (76.8 KB)
static uint32_t *stack_buf = NULL;   // Internal SRAM (16 KB)

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
  memset(threshold_lut, 0, sizeof(threshold_lut));

  float r_lin[32], g_lin[64], b_lin[32];
  for (int r = 0; r < 32; r++) r_lin[r] = srgbToLinear(r / 31.0f);
  for (int g = 0; g < 64; g++) g_lin[g] = srgbToLinear(g / 63.0f);
  for (int b = 0; b < 32; b++) b_lin[b] = srgbToLinear(b / 31.0f);

  for (uint32_t px = 0; px < 65536; px++) {
    uint16_t native_px = __builtin_bswap16((uint16_t)px);

    uint8_t r5 = (native_px >> 11) & 0x1F;
    uint8_t g6 = (native_px >> 5) & 0x3F;
    uint8_t b5 = native_px & 0x1F;

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

    if (inThreshold(L, A, B, THRESHOLD_BLIMP)) {
      threshold_lut[px >> 3] |= (1 << (px & 7));
    }
  }
}

static inline uint8_t getThresholdLUT(uint16_t px) {
  return (threshold_lut[px >> 3] >> (px & 7)) & 0x01;
}

static inline bool getMaskBit(int idx) {
  return (mask_buf[idx >> 3] >> (idx & 7)) & 1;
}

static inline void clearMaskBit(int idx) {
  mask_buf[idx >> 3] &= ~(1 << (idx & 7));
}

static inline void setMaskBit(int idx) {
  mask_buf[idx >> 3] |= (1 << (idx & 7));
}

IRAM_ATTR void generateSramMaskFast(const camera_fb_t *fb, Rect roi) {
  const uint16_t *buf = (const uint16_t *)fb->buf;
  int stride = fb->width;

  for (int ry = 0; ry < roi.h; ry++) {
    int row_pixel_offset = (roi.y + ry) * stride + roi.x;
    const uint16_t *src = &buf[row_pixel_offset];
    int rx = 0;

    while (rx < roi.w && ((row_pixel_offset + rx) & 7) != 0) {
      int bit_offset = row_pixel_offset + rx;
      if (getThresholdLUT(src[rx])) setMaskBit(bit_offset);
      else clearMaskBit(bit_offset);
      rx++;
    }

    for (; rx <= roi.w - 8; rx += 8) {
      uint8_t b = 0;
      b |= (getThresholdLUT(src[rx + 0]) << 0);
      b |= (getThresholdLUT(src[rx + 1]) << 1);
      b |= (getThresholdLUT(src[rx + 2]) << 2);
      b |= (getThresholdLUT(src[rx + 3]) << 3);
      b |= (getThresholdLUT(src[rx + 4]) << 4);
      b |= (getThresholdLUT(src[rx + 5]) << 5);
      b |= (getThresholdLUT(src[rx + 6]) << 6);
      b |= (getThresholdLUT(src[rx + 7]) << 7);

      int bit_offset = row_pixel_offset + rx;
      mask_buf[bit_offset >> 3] = b;
    }

    for (; rx < roi.w; rx++) {
      int bit_offset = row_pixel_offset + rx;
      if (getThresholdLUT(src[rx])) setMaskBit(bit_offset);
      else clearMaskBit(bit_offset);
    }
  }
}

#if MASKED_DEBUG_STREAM
void applyColorMaskFast(camera_fb_t *fb) {
  const uint16_t *src16 = (const uint16_t *)fb->buf;
  uint32_t *dst32 = (uint32_t *)fb->buf;
  size_t total_words = (fb->width * fb->height) / 2;

  for (size_t i = 0; i < total_words; i++) {
    uint16_t px0 = src16[i * 2];
    uint16_t px1 = src16[i * 2 + 1];

    uint32_t mask0 = getThresholdLUT(px0) ? 0x0000FFFF : 0x00000000;
    uint32_t mask1 = getThresholdLUT(px1) ? 0xFFFF0000 : 0x00000000;

    dst32[i] = mask0 | mask1;
  }
}
#endif

IRAM_ATTR Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  int stride = fb->width;
  int rw = roi.w, rh = roi.h;

  generateSramMaskFast(fb, roi);

  Blob best = {0, 0, 0, 0, 0, 0, 0};

  for (int ry = 0; ry < rh; ry += SCAN_STEP) {
    int img_y = roi.y + ry;
    int row_idx = img_y * stride;

    for (int rx = 0; rx < rw; rx += SCAN_STEP) {
      int img_x = roi.x + rx;
      int idx = row_idx + img_x;

      if ((idx & 7) == 0 && rx <= rw - 8 && mask_buf[idx >> 3] == 0x00) {
        rx += 7;
        continue;
      }

      if (!getMaskBit(idx)) continue;

      int stack_size = 0;
      stack_buf[stack_size++] = ((uint32_t)ry << 16) | (uint16_t)rx;

      long sumX = 0, sumY = 0;
      uint32_t count = 0;
      int minX = rx, maxX = rx, minY = ry, maxY = ry;

      while (stack_size > 0) {
        uint32_t pos = stack_buf[--stack_size];
        int seed_x = pos & 0xFFFF;
        int seed_y = pos >> 16;

        int row_offset = (roi.y + seed_y) * stride + roi.x;
        int seed_idx = row_offset + seed_x;
        if (!getMaskBit(seed_idx)) continue;

        int lx = seed_x;
        while (lx > 0) {
          int test_idx = row_offset + lx - 1;
          if ((test_idx & 7) == 7 && lx >= 8 && mask_buf[(test_idx - 7) >> 3] == 0xFF) {
            lx -= 8;
          } else if (getMaskBit(test_idx)) {
            lx--;
          } else {
            break;
          }
        }

        int rx_span = seed_x;
        while (rx_span < rw - 1) {
          int test_idx = row_offset + rx_span + 1;
          if ((test_idx & 7) == 0 && rx_span <= rw - 9 && mask_buf[test_idx >> 3] == 0xFF) {
            rx_span += 8;
          } else if (getMaskBit(test_idx)) {
            rx_span++;
          } else {
            break;
          }
        }

        for (int x = lx; x <= rx_span; x++) {
          int bit_idx = row_offset + x;

          if ((bit_idx & 7) == 0 && x + 7 <= rx_span && mask_buf[bit_idx >> 3] == 0xFF) {
            mask_buf[bit_idx >> 3] = 0x00;
            count += 8;
            sumX += (x * 8 + 28);
            sumY += seed_y * 8;
            x += 7;
          } else {
            if (getMaskBit(bit_idx)) {
              clearMaskBit(bit_idx);
              count++;
              sumX += x;
              sumY += seed_y;
            }
          }
        }

        if (lx < minX) minX = lx;
        if (rx_span > maxX) maxX = rx_span;
        if (seed_y < minY) minY = seed_y;
        if (seed_y > maxY) maxY = seed_y;

        const int dys[2] = {-1, 1};
        for (int i = 0; i < 2; i++) {
          int ny = seed_y + dys[i];
          if (ny < 0 || ny >= rh) continue;

          int n_row_offset = (roi.y + ny) * stride + roi.x;
          bool in_span = false;

          for (int x = lx; x <= rx_span; x++) {
            int n_idx = n_row_offset + x;

            if ((n_idx & 7) == 0 && x + 7 <= rx_span && mask_buf[n_idx >> 3] == 0x00) {
              in_span = false;
              x += 7;
              continue;
            }

            if (getMaskBit(n_idx)) {
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

      if (count >= area_threshold && count > best.pixels) {
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

static inline bool inBounds(int x, int y) {
  return x >= 0 && x < MAX_W && y >= 0 && y < MAX_H;
}

void drawOverlays(camera_fb_t *fb, Rect roi, Blob blob, bool locked) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;

  uint16_t COLOR_ROI  = locked ? 0xE007 : 0xE0FF;
  uint16_t COLOR_BLOB = 0x00F8;

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
    for (int x = blob.x; x < blob.x + blob.w; x++) {
      if (inBounds(x, blob.y)) buf[blob.y * stride + x] = COLOR_BLOB;
      if (inBounds(x, blob.y + blob.h - 1)) buf[(blob.y + blob.h - 1) * stride + x] = COLOR_BLOB;
    }
    for (int y = blob.y; y < blob.y + blob.h; y++) {
      if (inBounds(blob.x, y)) buf[y * stride + blob.x] = COLOR_BLOB;
      if (inBounds(blob.x + blob.w - 1, y)) buf[y * stride + (blob.x + blob.w - 1)] = COLOR_BLOB;
    }

    for (int d = -4; d <= 4; d++) {
      if (inBounds(blob.cx + d, blob.cy)) buf[blob.cy * stride + (blob.cx + d)] = COLOR_BLOB;
      if (inBounds(blob.cx, blob.cy + d)) buf[(blob.cy + d) * stride + blob.cx] = COLOR_BLOB;
    }
  }
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
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed with error 0x%x", err);
    errorLoop();
  }
}

#if DEBUG_STREAM
void streamDebug(camera_fb_t *work_fb, const Blob &blob, const Rect &roi, bool locked, const TelemetryData &telem) {
  #if MASKED_DEBUG_STREAM
    applyColorMaskFast(work_fb);
  #endif
  drawOverlays(work_fb, roi, blob, locked);

  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool converted = fmt2jpg(work_fb->buf, work_fb->len, MAX_W, MAX_H, PIXFORMAT_RGB565, 20, &jpg_buf, &jpg_len);

  if (converted) {
    FrameHeader header;
    header.magic[0] = 0xFF; header.magic[1] = 0xAA;
    header.magic[2] = 0x55; header.magic[3] = 0xFF;
    header.payload_len = jpg_len;
    header.telem = telem;

    fwrite((const void *)&header, 1, sizeof(header), stdout);
    fwrite((const void *)jpg_buf, 1, jpg_len, stdout);
    fflush(stdout);

    free(jpg_buf);
  }
}
#else
static void sendTelemetry(const TelemetryData &telem){
  FrameHeader header;

  header.magic[0] = 0xFF;
  header.magic[1] = 0xAA;
  header.magic[2] = 0x55;
  header.magic[3] = 0xFF;

  header.payload_len = 0;
  header.telem = telem;

  fwrite(&header, 1, sizeof(header), stdout);
  fflush(stdout);
}
#endif

// ======================= CORE 1: VISION TASK =======================
void visionTask(void *pvParameters) {
  while (1) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      taskYIELD();
      continue;
    }

    Blob blob;
    if (target_locked) {
      blob = findLargestBlob(fb, tracking_roi, AREA_THRESHOLD_LOCKED);
    } else {
      Rect full = {0, 0, MAX_W, MAX_H};
      blob = findLargestBlob(fb, full, AREA_THRESHOLD_SEARCH);
    }

    if (blob.pixels > 0) {
      heartbeat = esp_log_timestamp();

      int roi_x = MAX(0, blob.x - ROI_PADDING);
      int roi_y = MAX(0, blob.y - ROI_PADDING);
      int roi_w = MIN(MAX_W - roi_x, blob.w + 2 * ROI_PADDING);
      int roi_h = MIN(MAX_H - roi_y, blob.h + 2 * ROI_PADDING);

      tracking_roi = {roi_x, roi_y, roi_w, roi_h};
      target_locked = true;
    } else {
      target_locked = false;
      tracking_roi = {0, 0, MAX_W, MAX_H};
    }

    // Publish vision tracking state to shared memory (Non-blocking update)
    if (xSemaphoreTake(xVisionMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      g_vision_state.blob = blob;
      g_vision_state.roi = tracking_roi;
      g_vision_state.locked = target_locked;
      g_vision_state.heartbeat = heartbeat;
      xSemaphoreGive(xVisionMutex);
    }

    #if DEBUG_STREAM
      // Fetch latest IMU snapshot for debug streaming frame
      ImuData imu_snap = {};
      if (xSemaphoreTake(xImuMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        imu_snap = g_imu_data;
        xSemaphoreGive(xImuMutex);
      }

      TelemetryData telem = {};
      telem.ax = imu_snap.ax;
      telem.ay = imu_snap.ay;
      telem.az = imu_snap.az;
      telem.tz = imu_snap.tz;

      if (target_locked) {
        telem.hb = heartbeat;
        telem.cx = (uint16_t)tracking_roi.x;
        telem.cy = (uint16_t)tracking_roi.y;
        telem.w  = (uint16_t)tracking_roi.w;
        telem.h  = (uint16_t)tracking_roi.h;
      }

      streamDebug(fb, blob, tracking_roi, target_locked, telem);
    #endif

    // Instantly return camera buffer to keep driver buffer queue full
    esp_camera_fb_return(fb);
  }
}

// ==================== CORE 0: IMU & TELEMETRY TASK ====================
void imuTelemetryTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz sampling rate

  while (1) {
    // 1. Read IMU data independently at 100 Hz
    mpu6050_acceleration_t accel = {};
    mpu6050_rotation_t rotation = {};
    float temp = 0.0f;

    mpu6050_get_temperature(&dev, &temp);
    if (mpu6050_get_motion(&dev, &accel, &rotation) == ESP_OK) {
      if (xSemaphoreTake(xImuMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        g_imu_data.ax = accel.x;
        g_imu_data.ay = accel.y;
        g_imu_data.az = accel.z;
        g_imu_data.tz = rotation.x;
        g_imu_data.valid = true;
        xSemaphoreGive(xImuMutex);
      }
    }

    #if !DEBUG_STREAM
      // 2. In non-debug mode, emit binary telemetry at fixed 100 Hz rate
      VisionState vision_snap = {};
      if (xSemaphoreTake(xVisionMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        vision_snap = g_vision_state;
        xSemaphoreGive(xVisionMutex);
      }

      TelemetryData telem = {};
      telem.ax = accel.x;
      telem.ay = accel.y;
      telem.az = accel.z;
      telem.tz = rotation.x;

      if (vision_snap.locked) {
        telem.hb = vision_snap.heartbeat;
        telem.cx = (uint16_t)vision_snap.roi.x;
        telem.cy = (uint16_t)vision_snap.roi.y;
        telem.w  = (uint16_t)vision_snap.roi.w;
        telem.h  = (uint16_t)vision_snap.roi.h;
      }

      sendTelemetry(telem);
    #endif

    // Sleep precisely until next 10ms boundary
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

extern "C" void app_main(void) {
  esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
  esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
  setvbuf(stdout, NULL, _IONBF, 0);

  gpio_reset_pin(LED_GPIO_NUM);
  gpio_set_direction(LED_GPIO_NUM, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_GPIO_NUM, 0);

  // Initialize IMU
  ESP_ERROR_CHECK(i2cdev_init());
  dev = {};
  ESP_ERROR_CHECK(mpu6050_init_desc(&dev, MPU_ADDR, 0, PIN_SDA, PIN_SCL));
  dev.i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  dev.i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  ESP_LOGI(TAG, "Probing MPU6050 on SDA=GPIO%d, SCL=GPIO%d, Address=0x%02x...", PIN_SDA, PIN_SCL, MPU_ADDR);

  while (1) {
      esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
      if (res == ESP_OK) {
          ESP_LOGI(TAG, "SUCCESS: Found MPU60x0 device!");
          break;
      }
      ESP_LOGE(TAG, "MPU60x0 not found (error 0x%x)", res);
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_ERROR_CHECK(mpu6050_init(&dev));

  vTaskDelay(pdMS_TO_TICKS(500));

  setupCamera();

  size_t total_pixels = MAX_W * MAX_H;

  mask_buf = (uint8_t *)heap_caps_malloc((total_pixels + 7) / 8, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  stack_buf = (uint32_t *)heap_caps_malloc(MAX_STACK_SIZE * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!mask_buf || !stack_buf) {
    ESP_LOGE(TAG, "Heap allocation failed! mask_buf=%p, stack_buf=%p", mask_buf, stack_buf);
    errorLoop();
  }

  initThresholdLUT();

  esp_log_level_set("*", ESP_LOG_NONE);

  // Create Mutexes for double-buffered cross-core state sharing
  xVisionMutex = xSemaphoreCreateMutex();
  xImuMutex    = xSemaphoreCreateMutex();

  // Core 0: Decoupled 100Hz IMU sampling & telemetry generator
  xTaskCreatePinnedToCore(imuTelemetryTask, "imu_telem_task", 8192, NULL, 5, NULL, 0);

  // Core 1: High-speed Vision processing loop
  xTaskCreatePinnedToCore(visionTask,       "vision_task",    16384, NULL, 5, NULL, 1);
}
