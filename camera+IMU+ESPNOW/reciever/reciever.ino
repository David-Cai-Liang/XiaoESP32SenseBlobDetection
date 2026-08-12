#include "Vision.h"
#include "IMU.h"
#include <esp_now.h>
#include <WiFi.h>

// Combined telemetry packet structure
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
} PacketData;

PacketData incomingPacket;

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(PacketData)) {
    memcpy(&incomingPacket, incomingData, sizeof(PacketData));
    
    // Write raw binary packet out via Serial
    Serial.write(incomingData, sizeof(PacketData));
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
  // Processing handled asynchronously via ESP-NOW callback
}