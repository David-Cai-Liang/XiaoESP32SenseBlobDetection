#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) yield();
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.printf("Accel X: %.2f, Gyro X: %.2f\n", a.acceleration.x, g.gyro.x);
  Serial.printf("Accel Y: %.2f, Gyro Y: %.2f\n", a.acceleration.y, g.gyro.y);
  Serial.printf("Accel Z: %.2f, Gyro Z: %.2f\n", a.acceleration.z, g.gyro.z);
  delay(100);
}