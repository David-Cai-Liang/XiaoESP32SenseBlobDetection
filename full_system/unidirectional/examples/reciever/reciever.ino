#include <Vision.h>
#define sensor_t adafruit_sensor_t
#include <IMU.h>
#undef sensor_t
#include <esp_now.h>
#include <WiFi.h>

// Combined telemetry packet structure
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
} PacketData;

PacketData incomingPacket;
volatile bool newDataAvailable = false;

// Callback function executed when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(PacketData)) {
    memcpy(&incomingPacket, incomingData, sizeof(PacketData));
    newDataAvailable = true;
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (newDataAvailable) {
    newDataAvailable = false;

    // Decode and print VisionData
    Serial.printf("[VISION] CX: %u | CY: %u | W: %u | H: %u\n",
                  incomingPacket.vision.cx,
                  incomingPacket.vision.cy,
                  incomingPacket.vision.w,
                  incomingPacket.vision.h);

    // Decode and print IMUData
    Serial.printf("[IMU]    AX: %.2f | AY: %.2f | AZ: %.2f | TZ: %.2f\n",
                  incomingPacket.imu.ax,
                  incomingPacket.imu.ay,
                  incomingPacket.imu.az,
                  incomingPacket.imu.tz);

    Serial.println("--------------------------------------------------");
  }
}