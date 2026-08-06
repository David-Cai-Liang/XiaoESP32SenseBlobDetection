#include "Arduino.h"
#include "esp_camera.h"
#include "img_converters.h"
#include <math.h>

// Camera pins - hard-coded for Xiao ESP32S3 Sense
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

static const int FRAME_W = 320;
static const int FRAME_H = 240;

static const int ROI_X = 120; // Center 80x80 box
static const int ROI_Y = 80;
static const int ROI_W = 80;
static const int ROI_H = 80;

struct LabThreshold {
  int16_t l_min, l_max, a_min, a_max, b_min, b_max;
};

// 20-byte binary header packet sent before each JPEG payload
struct __attribute__((packed)) FrameHeader {
  uint8_t magic[4];     // {0xFF, 0xAA, 0x55, 0xFF}
  uint32_t payload_len; // Size of upcoming JPEG frame
  LabThreshold threshold;
};

static LabThreshold current_threshold = {15, 80, 15, 70, 0, 40};

// ---------------------------------------------------------
// LAB Color Conversion
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

static inline bool inBounds(int x, int y) {
  return x >= 0 && x < FRAME_W && y >= 0 && y < FRAME_H;
}

void processFrameAndDrawROI(camera_fb_t *fb) {
  uint16_t *buf = (uint16_t *)fb->buf;
  int stride = fb->width;
  uint32_t now = millis();

  // Rate-limit LAB stats calculation to every 150 ms
  static uint32_t last_calc_time = 0;
  if (now - last_calc_time >= 150) {
    last_calc_time = now;

    double sum_L = 0, sum_A = 0, sum_B = 0;
    uint32_t count = 0;

    const int STEP = 1; // Subsample 1x1 for accuracy
    for (int y = ROI_Y; y < ROI_Y + ROI_H; y += STEP) {
      int row_offset = y * stride;
      for (int x = ROI_X; x < ROI_X + ROI_W; x += STEP) {
        uint16_t px = buf[row_offset + x];
        int L, A, B;
        rgb565ToLab(px, L, A, B);
        if (L > 75) continue; // Skip specular glare
        if (L < 25) continue; // Skip deep shadow
        sum_L += L; sum_A += A; sum_B += B;
        count++;
      }
    }

    if (count > 0) {
      float mean_L = sum_L / count;
      float mean_A = sum_A / count;
      float mean_B = sum_B / count;

      double sq_L = 0, sq_A = 0, sq_B = 0;
      for (int y = ROI_Y; y < ROI_Y + ROI_H; y += STEP) {
        int row_offset = y * stride;
        for (int x = ROI_X; x < ROI_X + ROI_W; x += STEP) {
          uint16_t px = buf[row_offset + x];
          int L, A, B;
          rgb565ToLab(px, L, A, B);
          sq_L += (L - mean_L) * (L - mean_L);
          sq_A += (A - mean_A) * (A - mean_A);
          sq_B += (B - mean_B) * (B - mean_B);
        }
      }

      float stdev_L = sqrt(sq_L / count);
      float stdev_A = sqrt(sq_A / count);
      float stdev_B = sqrt(sq_B / count);

      current_threshold.l_min = constrain((int16_t)round(mean_L - 1.0f * stdev_L), 0, 100);
      current_threshold.l_max = constrain((int16_t)round(mean_L + 1.0f * stdev_L), 0, 100);
      current_threshold.a_min = constrain((int16_t)round(mean_A - 1.0f * stdev_A), -128, 127);
      current_threshold.a_max = constrain((int16_t)round(mean_A + 1.0f * stdev_A), -128, 127);
      current_threshold.b_min = constrain((int16_t)round(mean_B - 1.0f * stdev_B), -128, 127);
      current_threshold.b_max = constrain((int16_t)round(mean_B + 1.0f * stdev_B), -128, 127);
    }
  }

  // Draw Red ROI Box (2px thick)
  uint16_t RED = 0xF800;
  for (int t = 0; t < 2; t++) {
    for (int x = ROI_X - t; x < ROI_X + ROI_W + t; x++) {
      if (inBounds(x, ROI_Y - 1 - t)) buf[(ROI_Y - 1 - t) * stride + x] = RED;
      if (inBounds(x, ROI_Y + ROI_H + t)) buf[(ROI_Y + ROI_H + t) * stride + x] = RED;
    }
    for (int y = ROI_Y - t; y < ROI_Y + ROI_H + t; y++) {
      if (inBounds(ROI_X - 1 - t, y)) buf[y * stride + (ROI_X - 1 - t)] = RED;
      if (inBounds(ROI_X + ROI_W + t, y)) buf[y * stride + (ROI_X + ROI_W + t)] = RED;
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
  config.pixel_format = PIXFORMAT_RGB565;
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
    s->set_whitebal(s, 0);       // Disable Auto White Balance
    s->set_exposure_ctrl(s, 0);  // Disable Auto Exposure
    s->set_gain_ctrl(s, 0);      // Disable Auto Gain Control
    s->set_aec_value(s, 40);    // Manual exposure value (0-1200)
  }
}

void setup() {
  Serial.begin(115200);
  setupCamera();
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  processFrameAndDrawROI(fb);

  uint8_t *jpg_buf = NULL;
  size_t jpg_len = 0;
  bool converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 20, &jpg_buf, &jpg_len);
  esp_camera_fb_return(fb);

  if (converted) {
    FrameHeader header;
    header.magic[0] = 0xFF; header.magic[1] = 0xAA;
    header.magic[2] = 0x55; header.magic[3] = 0xFF;
    header.payload_len = jpg_len;
    header.threshold = current_threshold;

    // Send binary frame header and JPEG payload over native USB CDC
    Serial.write((uint8_t *)&header, sizeof(header));
    Serial.write(jpg_buf, jpg_len);
    free(jpg_buf);
  }
}