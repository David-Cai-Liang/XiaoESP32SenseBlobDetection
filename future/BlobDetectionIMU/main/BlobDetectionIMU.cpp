#define DEBUG_STREAM 1
#define MASKED_DEBUG_STREAM 1

// --- Standard Includes ---
#include <stdio.h>
#include <string.h>
#include <math.h>

// --- FreeRTOS & ESP System Includes ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "esp_vfs_usb_serial_jtag.h"

// --- Camera & Image Includes ---
#include "esp_camera.h"
#include "img_converters.h"

// --- Sensor Drivers ---
#include <mpu6050.h>
#include <i2cdev.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// =============================================================================

static const char *TAG = "MAIN";

// --- XIAO ESP32S3 Hardware Pins ---
#define LED_GPIO_NUM    (gpio_num_t)21 // User LED (Active LOW)

// Camera Control & Data Pins
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// MPU6050 IMU Pins & Address
#define PIN_SDA         GPIO_NUM_5
#define PIN_SCL         GPIO_NUM_6
#define MPU_ADDR        0x68

// Image & Frame Buffer Parameters
#define MAX_W           320
#define MAX_H           240
#define MAX_STACK_SIZE  8192 // Stack entries (32 KB in internal SRAM) for Scanline Flood Fill

// Macro Utilities
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// =============================================================================
// DATA STRUCTURES
// =============================================================================

struct LabThreshold {
    int l_min, l_max;
    int a_min, a_max;
    int b_min, b_max;
};

struct Rect {
    int x, y, w, h;
};

struct Blob {
    int x, y, w, h;
    int cx, cy;
    uint32_t pixels;
};

// Binary Telemetry Structure (Packed for serial transmission)
typedef struct __attribute__((packed)) {
    uint16_t hb;            // Heartbeat / Timestamp
    uint16_t cx, cy, w, h;  // Bounding box / ROI tracking parameters
    float ax, ay, az, tz;   // MPU6050 Accelerometer (X,Y,Z) & Gyro Z
} TelemetryData;

// Compressed Serial Output Frame Header
typedef struct __attribute__((packed)) {
    uint8_t magic[4];       // Sync Marker: {0xFF, 0xAA, 0x55, 0xFF}
    uint32_t payload_len;   // JPEG Payload Length in bytes
    TelemetryData telem;    // Sensor & Tracking Metadata
} FrameHeader;

// Internal Frame Processing Container
typedef struct {
    camera_fb_t *camera_fb; // Raw camera buffer pointer
    camera_fb_t work_fb;   // Working copy of frame metadata/buffer
    Blob blob;             // Detected target blob
    bool valid;            // Frame grab success flag
} FrameResult;

// =============================================================================
// CONFIGURATION & GLOBAL STATE
// =============================================================================

static const LabThreshold THRESHOLD_BLIMP = {20, 60, 10, 50, 1, 30};

static const uint32_t AREA_THRESHOLD_LOCKED = 30;  // Pixel threshold when target is locked
static const uint32_t AREA_THRESHOLD_SEARCH = 60;  // Pixel threshold during full-frame search
static const int ROI_PADDING = 30;                  // Extra padding around tracked target ROI
static const int SCAN_STEP = 1;                     // Pixel step size during blob scan

static mpu6050_dev_t dev;                           // MPU6050 driver instance

static Rect tracking_roi = {0, 0, MAX_W, MAX_H};
static bool target_locked = false;
static uint16_t heartbeat = 1;

// Internal Memory Allocations (DRAM)
static uint8_t threshold_lut[8192];  // 64Kb RGB565 binary lookup table (8192 bytes)
static uint8_t *mask_buf = NULL;      // Bitmask for active region of interest (76.8 KB SRAM)
static uint32_t *stack_buf = NULL;    // Stack buffer for scanline flood fill (16 KB SRAM)

// =============================================================================
// COLOR CONVERSION & LOOKUP TABLE (LUT) GENERATION
// =============================================================================

// Convert sRGB channel to Linear RGB float
static inline float srgbToLinear(float c) {
    return (c <= 0.04045f) ? (c / 12.92f) : powf((c + 0.055f) / 1.055f, 2.4f);
}

// Helper transfer function for CIELAB conversion
static inline float labF(float t) {
    return (t > 0.008856f) ? cbrtf(t) : (7.787f * t + (16.0f / 116.0f));
}

// Check if L*a*b* values fall within the target threshold
static inline bool inThreshold(int L, int A, int B, const LabThreshold &t) {
    return (L >= t.l_min && L <= t.l_max) &&
           (A >= t.a_min && A <= t.a_max) &&
           (B >= t.b_min && B <= t.b_max);
}

// Precomputes RGB565 to 1-bit binary threshold output in lookup table
void initThresholdLUT() {
    memset(threshold_lut, 0, sizeof(threshold_lut));

    // Precalculate linear sRGB lookup values
    float r_lin[32], g_lin[64], b_lin[32];
    for (int r = 0; r < 32; r++) r_lin[r] = srgbToLinear(r / 31.0f);
    for (int g = 0; g < 64; g++) g_lin[g] = srgbToLinear(g / 63.0f);
    for (int b = 0; b < 32; b++) b_lin[b] = srgbToLinear(b / 31.0f);

    // Populate binary bit-array for all 65,536 RGB565 colors
    for (uint32_t px = 0; px < 65536; px++) {
        uint16_t native_px = __builtin_bswap16((uint16_t)px);

        uint8_t r5 = (native_px >> 11) & 0x1F;
        uint8_t g6 = (native_px >> 5) & 0x3F;
        uint8_t b5 = native_px & 0x1F;

        float r = r_lin[r5];
        float g = g_lin[g6];
        float b = b_lin[b5];

        // Convert RGB to XYZ Color Space
        float X = (r * 0.4124f + g * 0.3576f + b * 0.1805f) / 0.95047f;
        float Y = (r * 0.2126f + g * 0.7152f + b * 0.0722f);
        float Z = (r * 0.0193f + g * 0.1192f + b * 0.9505f) / 1.08883f;

        // Convert XYZ to CIELAB Color Space
        float fx = labF(X), fy = labF(Y), fz = labF(Z);

        int L = (int)(116.0f * fy - 16.0f);
        int A = (int)(500.0f * (fx - fy));
        int B = (int)(200.0f * (fy - fz));

        // Store result as a single bit in threshold_lut
        if (inThreshold(L, A, B, THRESHOLD_BLIMP)) {
            threshold_lut[px >> 3] |= (1 << (px & 7));
        }
    }
}

// Read binary threshold state for an RGB565 pixel from LUT
static inline uint8_t getThresholdLUT(uint16_t px) {
    return (threshold_lut[px >> 3] >> (px & 7)) & 0x01;
}

// =============================================================================
// BITMASK HELPER FUNCTIONS (SRAM MASK)
// =============================================================================

static inline bool getMaskBit(int idx) {
    return (mask_buf[idx >> 3] >> (idx & 7)) & 1;
}

static inline void clearMaskBit(int idx) {
    mask_buf[idx >> 3] &= ~(1 << (idx & 7));
}

static inline void setMaskBit(int idx) {
    mask_buf[idx >> 3] |= (1 << (idx & 7));
}

// =============================================================================
// IMAGE PROCESSING & COMPUTER VISION ALGORITHMS
// =============================================================================

/**
 * @brief Generates a fast 1-bit memory mask within the designated ROI.
 *        Packs 8 pixel binary results into individual SRAM bytes for vectorized operations.
 */
IRAM_ATTR void generateSramMaskFast(const camera_fb_t *fb, Rect roi) {
    const uint16_t *buf = (const uint16_t *)fb->buf;
    int stride = fb->width;

    for (int ry = 0; ry < roi.h; ry++) {
        int row_pixel_offset = (roi.y + ry) * stride + roi.x;
        const uint16_t *src = &buf[row_pixel_offset];
        int rx = 0;

        // 1. Head pixels (Un-aligned bits up to byte boundary)
        while (rx < roi.w && ((row_pixel_offset + rx) & 7) != 0) {
            int bit_offset = row_pixel_offset + rx;
            if (getThresholdLUT(src[rx])) {
                setMaskBit(bit_offset);
            } else {
                clearMaskBit(bit_offset);
            }
            rx++;
        }

        // 2. Vectorized 8-pixel block writes (Pack 8 bits into a byte)
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

        // 3. Tail pixels (Remaining unaligned trailing bits)
        for (; rx < roi.w; rx++) {
            int bit_offset = row_pixel_offset + rx;
            if (getThresholdLUT(src[rx])) {
                setMaskBit(bit_offset);
            } else {
                clearMaskBit(bit_offset);
            }
        }
    }
}

#if MASKED_DEBUG_STREAM
/**
 * @brief Replaces frame buffer pixels with binary visual mask for debug streaming.
 */
void applyColorMaskFast(camera_fb_t *fb) {
    const uint16_t *src16 = (const uint16_t *)fb->buf;
    uint32_t *dst32 = (uint32_t *)fb->buf;
    size_t total_words = (fb->width * fb->height) / 2; // Process 2 pixels (32 bits) per iteration

    for (size_t i = 0; i < total_words; i++) {
        uint16_t px0 = src16[i * 2];
        uint16_t px1 = src16[i * 2 + 1];

        uint32_t mask0 = getThresholdLUT(px0) ? 0x0000FFFF : 0x00000000;
        uint32_t mask1 = getThresholdLUT(px1) ? 0xFFFF0000 : 0x00000000;

        dst32[i] = mask0 | mask1;
    }
}
#endif

/**
 * @brief Scanline Flood Fill implementation to extract the largest connected component blob.
 */
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

            // Fast skip 8 consecutive empty mask pixels
            if ((idx & 7) == 0 && rx <= rw - 8 && mask_buf[idx >> 3] == 0x00) {
                rx += 7; // Advance loop
                continue;
            }

            if (!getMaskBit(idx)) continue;

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
                int seed_idx = row_offset + seed_x;
                if (!getMaskBit(seed_idx)) continue;

                // Scan left boundary of current horizontal span
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

                // Scan right boundary of current horizontal span
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

                // Process horizontal span & clear mask
                for (int x = lx; x <= rx_span; x++) {
                    int bit_idx = row_offset + x;

                    // Fast clear 8 solid pixels at once
                    if ((bit_idx & 7) == 0 && x + 7 <= rx_span && mask_buf[bit_idx >> 3] == 0xFF) {
                        mask_buf[bit_idx >> 3] = 0x00;
                        count += 8;
                        sumX += (x * 8 + 28); // Direct sum calculation for span (x + ... + x+7)
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

                // Update component bounds
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
                        int n_idx = n_row_offset + x;

                        // Fast skip empty 8-pixel block on adjacent line
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

            // Save tracking information if largest component so far
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

// Check if image coordinates are within display boundary
static inline bool inBounds(int x, int y) {
    return x >= 0 && x < MAX_W && y >= 0 && y < MAX_H;
}

// Draw target overlays (ROI box, Blob box, and target crosshair) directly onto frame
void drawOverlays(camera_fb_t *fb, Rect roi, Blob blob, bool locked) {
    uint16_t *buf = (uint16_t *)fb->buf;
    int stride = fb->width;

    // Byte-swapped RGB565 Colors (Green: 0x07E0 -> 0xE007, Yellow: 0xFFE0 -> 0xE0FF, Red: 0xF800 -> 0x00F8)
    uint16_t COLOR_ROI  = locked ? 0xE007 : 0xE0FF;
    uint16_t COLOR_BLOB = 0x00F8;

    // Draw ROI outline
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

    // Draw Target Blob outline & crosshair
    if (blob.pixels > 0) {
        for (int x = blob.x; x < blob.x + blob.w; x++) {
            if (inBounds(x, blob.y)) buf[blob.y * stride + x] = COLOR_BLOB;
            if (inBounds(x, blob.y + blob.h - 1)) buf[(blob.y + blob.h - 1) * stride + x] = COLOR_BLOB;
        }
        for (int y = blob.y; y < blob.y + blob.h; y++) {
            if (inBounds(blob.x, y)) buf[y * stride + blob.x] = COLOR_BLOB;
            if (inBounds(blob.x + blob.w - 1, y)) buf[y * stride + (blob.x + blob.w - 1)] = COLOR_BLOB;
        }

        // Crosshair centered at blob center
        for (int d = -4; d <= 4; d++) {
            if (inBounds(blob.cx + d, blob.cy)) buf[blob.cy * stride + (blob.cx + d)] = COLOR_BLOB;
            if (inBounds(blob.cx, blob.cy + d)) buf[(blob.cy + d) * stride + blob.cx] = COLOR_BLOB;
        }
    }
}

// =============================================================================
// HARDWARE INITIALIZATION & TELEMETRY
// =============================================================================

// Infinite LED blink loop invoked upon fatal hardware failure
void errorLoop() {
    while (true) {
        gpio_set_level(LED_GPIO_NUM, 0); // LED ON
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO_NUM, 1); // LED OFF
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Initialize Camera Module Driver
void setupCamera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size   = FRAMESIZE_QVGA;
    config.pixel_format = PIXFORMAT_RGB565; // Direct raw RGB565 from sensor
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.fb_count     = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed with error 0x%x", err);
        errorLoop();
    }
}

// Fetch frame and analyze image for target blob
FrameResult processFrame() {
    FrameResult result = {};
    result.valid = false;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return result;
    }

    result.camera_fb = fb;
    result.work_fb = *fb;

    if (target_locked) {
        result.blob = findLargestBlob(&result.work_fb, tracking_roi, AREA_THRESHOLD_LOCKED);
    } else {
        Rect full = {0, 0, MAX_W, MAX_H};
        result.blob = findLargestBlob(&result.work_fb, full, AREA_THRESHOLD_SEARCH);
    }

    result.valid = true;
    return result;
}

// Read IMU sensor data & compute tracking telemetry
TelemetryData buildTelemetry(const Blob &largest) {
    TelemetryData telem = {};

    mpu6050_acceleration_t accel = {};
    mpu6050_rotation_t rotation = {};
    float temp = 0.0f;

    // Fetch motion readings regardless of image target state
    ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

    if (largest.pixels > 0) {
        heartbeat = esp_log_timestamp();

        int roi_x = MAX(0, largest.x - ROI_PADDING);
        int roi_y = MAX(0, largest.y - ROI_PADDING);
        int roi_w = MIN(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
        int roi_h = MIN(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

        tracking_roi = {roi_x, roi_y, roi_w, roi_h};
        target_locked = true;

        telem.hb = heartbeat;
        telem.cx = (uint16_t)roi_x;
        telem.cy = (uint16_t)roi_y;
        telem.w  = (uint16_t)roi_w;
        telem.h  = (uint16_t)roi_h;
    } else {
        target_locked = false;
        tracking_roi = {0, 0, MAX_W, MAX_H};

        telem.hb = 0;
        telem.cx = 0;
        telem.cy = 0;
        telem.w  = 0;
        telem.h  = 0;
    }

    // Populate IMU motion attributes
    telem.ax = accel.x;
    telem.ay = accel.y;
    telem.az = accel.z;
    telem.tz = rotation.x;

    return telem;
}

#if DEBUG_STREAM
// Stream JPEG compressed debug frame with custom binary header via stdout
void streamDebug(camera_fb_t *work_fb, const Blob &blob, const Rect &roi, bool locked, const TelemetryData &telem) {
    #if MASKED_DEBUG_STREAM
    applyColorMaskFast(work_fb); // Render visual mask overlay
    #endif

    drawOverlays(work_fb, roi, blob, locked);

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool converted = fmt2jpg(work_fb->buf, work_fb->len, MAX_W, MAX_H, PIXFORMAT_RGB565, 20, &jpg_buf, &jpg_len);

    if (converted) {
        FrameHeader header;
        header.magic[0] = 0xFF;
        header.magic[1] = 0xAA;
        header.magic[2] = 0x55;
        header.magic[3] = 0xFF;
        header.payload_len = jpg_len;
        header.telem = telem;

        fwrite((const void *)&header, 1, sizeof(header), stdout);
        fwrite((const void *)jpg_buf, 1, jpg_len, stdout);
        fflush(stdout);

        free(jpg_buf);
    }
}
#else
// Stream standard telemetry-only binary header via stdout
static void sendTelemetry(const TelemetryData &telem) {
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

// =============================================================================
// FREERTOS TASKS & APPLICATION MAIN ENTRY
// =============================================================================

// Main Frame Processing Task
void processingTask(void *pvParameters) {
    while (1) {
        FrameResult result = processFrame();
        if (!result.valid) {
            taskYIELD();
            continue;
        }

        TelemetryData telem = buildTelemetry(result.blob);

        #if DEBUG_STREAM
        streamDebug(&result.work_fb, result.blob, tracking_roi, target_locked, telem);
        #else
        sendTelemetry(telem);
        #endif

        esp_camera_fb_return(result.camera_fb);
        taskYIELD();
    }
}

extern "C" void app_main(void) {
    // Standard IO & Line Endings Configuration
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    setvbuf(stdout, NULL, _IONBF, 0);

    // Status LED Config
    gpio_reset_pin(LED_GPIO_NUM);
    gpio_set_direction(LED_GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO_NUM, 0); // Solid ON during initialization

    // Initialize MPU6050 via I2C Dev
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

    // Setup Camera
    setupCamera();

    // Allocate SRAM Buffers for high-speed bitwise operations
    size_t total_pixels = MAX_W * MAX_H;

    mask_buf  = (uint8_t *)heap_caps_malloc((total_pixels + 7) / 8, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    stack_buf = (uint32_t *)heap_caps_malloc(MAX_STACK_SIZE * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!mask_buf || !stack_buf) {
        ESP_LOGE(TAG, "Heap allocation failed! mask_buf=%p, stack_buf=%p", mask_buf, stack_buf);
        errorLoop();
    }

    // Precalculate CIELAB threshold lookup tables
    initThresholdLUT();

    // Disable serial logging output to keep stdout clear for streaming
    esp_log_level_set("*", ESP_LOG_NONE);

    // Launch core frame processing task on Core 1
    xTaskCreatePinnedToCore(processingTask, "proc_task", 16384, NULL, 5, NULL, 1);
}
