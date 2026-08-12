#include "Vision.h"
#include "IMU.h"
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Combined telemetry packet structure
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
} PacketData;

esp_now_peer_info_t peerInfo;

Vision vision;
IMU imu;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize hardware sensors and vision module
  imu.setup();
  vision.setup();
}

void loop() {
  // 1. Process Vision Frame
  FrameResult result = vision.processFrame();
  VisionData vData = {};

  if (result.valid) {
    vData = vision.buildVisionData(result.blob);
  }

  // 2. Read IMU Data
  IMUData iData = imu.readData();

  // 3. Package combined payload
  PacketData packet;
  packet.vision = vData;
  packet.imu = iData;

  // 4. Send message via ESP-NOW
  esp_err_t sendResult = esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
  if (sendResult == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(10);
}