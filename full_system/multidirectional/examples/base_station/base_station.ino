#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR BLIMP MAC ADDRESS
uint8_t blimpAddress[] = {0x68, 0xee, 0x8f, 0x50, 0x1e, 0xa0};

typedef struct __attribute__((packed)) {
  int16_t motors[4];
} ControlPacket;

typedef struct __attribute__((packed)) {
  uint16_t cx, cy, w, h;
  float ax, ay, az, tz;
} TelemetryPacket;

// Framed Protocol Markers
const uint8_t TELE_HEADER[4] = {0x00, 0xAA, 0x55, 0xFF};
const uint8_t TELE_FOOTER[2] = {0xEE, 0xFF};
const uint8_t CTRL_HEADER[4] = {0x00, 0xBB, 0x66, 0xFF};

esp_now_peer_info_t peerInfo;

TelemetryPacket latestTelemetry;
volatile bool newTelemetryReceived = false;

// Store packet in buffer instead of writing directly inside callback ISR
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&latestTelemetry, incomingData, sizeof(TelemetryPacket));
    newTelemetryReceived = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) return;

  esp_now_register_recv_cb(OnDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, blimpAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  // 1. Safely stream telemetry out over Serial from main loop thread
  if (newTelemetryReceived) {
    newTelemetryReceived = false;
    Serial.write(TELE_HEADER, 4);
    Serial.write((uint8_t *)&latestTelemetry, sizeof(TelemetryPacket));
    Serial.write(TELE_FOOTER, 2);
  }

  // 2. Read incoming motor controls from Python
  const size_t expected_size = 4 + sizeof(ControlPacket);
  if (Serial.available() >= expected_size) {
    if (Serial.peek() == CTRL_HEADER[0]) {
      uint8_t header[4];
      Serial.readBytes(header, 4);

      if (memcmp(header, CTRL_HEADER, 4) == 0) {
        ControlPacket control;
        Serial.readBytes((char *)&control, sizeof(ControlPacket));
        esp_now_send(blimpAddress, (uint8_t *)&control, sizeof(control));
      }
    } else {
      Serial.read(); // Discard unaligned byte
    }
  }
}