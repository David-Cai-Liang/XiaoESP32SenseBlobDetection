#include <Vision.h>
#define sensor_t adafruit_sensor_t
#include <IMU.h>
#undef sensor_t
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Corrected Motor GPIO Pin Mapping from ESP-FLY Wiring Diagram
const int MOTOR_M1_FR = 7; // Front Right (M1) -> Pin 7 (Purple Wire)
const int MOTOR_M2_RR = 4; // Rear Right  (M2) -> Pin 4 (Green Wire)
const int MOTOR_M3_RL = 3; // Rear Left   (M3) -> Pin 3 (Blue Wire)
const int MOTOR_M4_FL = 1; // Front Left  (M4) -> Pin 1 (Orange Wire)

// REPLACE WITH YOUR BASE STATION MAC ADDRESS
uint8_t baseStationAddress[] = {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C};

// Telemetry sent from Blimp to Base Station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
} TelemetryPacket;

// Motor control commands received from Base Station
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // Motor 1, 2, 3, 4 speed/direction inputs
} ControlPacket;

esp_now_peer_info_t peerInfo;
Vision vision;
IMU imu;

ControlPacket incomingControl = {{0, 0, 0, 0}};
volatile bool newControlAvailable = false;

unsigned long lastRecvTime = 0;
const unsigned long CONTROL_TIMEOUT_MS = 1000;

// Callback when telemetry is sent to Base Station
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // Optional: Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Telemetry Sent" : "Telemetry Send Fail");
}

// Callback when motor controls are received from Base Station
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlPacket)) {
    memcpy(&incomingControl, incomingData, sizeof(ControlPacket));
    newControlAvailable = true;
    lastRecvTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  // Set device in Wi-Fi Station Mode
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW Callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register Base Station as Peer
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, baseStationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Base Station peer");
    return;
  }

  // Initialize Sensors and Camera Hardware
  imu.setup();
  vision.setup();
}

void loop() {
  // 1. Process Vision & IMU Telemetry
  FrameResult result = vision.processFrame();
  VisionData vData = {};

  if (result.valid) {
    vData = vision.buildVisionData(result.blob);
  }

  IMUData iData = imu.readData();

  // 2. Transmit Telemetry Packet to Base Station
  TelemetryPacket telemetry;
  telemetry.vision = vData;
  telemetry.imu = iData;

  esp_now_send(baseStationAddress, (uint8_t *)&telemetry, sizeof(telemetry));

  bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

  // 3. Process Received Motor Commands
  if (newControlAvailable || stale) {
    newControlAvailable = false;
    
    int16_t m1 = stale ? 0 : incomingControl.motors[0];
    int16_t m2 = stale ? 0 : incomingControl.motors[1];
    int16_t m3 = stale ? 0 : incomingControl.motors[2];
    int16_t m4 = stale ? 0 : incomingControl.motors[3];
    // Apply motor inputs to hardware here
    Serial.printf("[MOTORS] M1: %d | M2: %d | M3: %d | M4: %d\n",
                  m1,
                  m2,
                  m3,
                  m4);

    analogWrite(MOTOR_M1_FR, m1);
    analogWrite(MOTOR_M2_RR, m2);
    analogWrite(MOTOR_M3_RL, m3);
    analogWrite(MOTOR_M4_FL, m4);
  }

  vTaskDelay(1);
}