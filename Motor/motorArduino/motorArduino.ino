// Corrected Motor GPIO Pin Mapping from ESP-FLY Wiring Diagram
const int MOTOR_M1_FR = 7; // Front Right (M1) -> Pin 7 (Purple Wire)
const int MOTOR_M2_RR = 4; // Rear Right  (M2) -> Pin 4 (Green Wire)
const int MOTOR_M3_RL = 3; // Rear Left   (M3) -> Pin 3 (Blue Wire)
const int MOTOR_M4_FL = 1; // Front Left  (M4) -> Pin 1 (Orange Wire)

const int MAX_PWM = 64;         // 25% max duty cycle (64 / 255)
const int STEP_DELAY_MS = 468;  // Ramps 0 to 64 over ~30 seconds (30,000ms / 64 steps)

void setup() {
  pinMode(MOTOR_M1_FR, OUTPUT);
  pinMode(MOTOR_M2_RR, OUTPUT);
  pinMode(MOTOR_M3_RL, OUTPUT);
  pinMode(MOTOR_M4_FL, OUTPUT);
}

void loop() {
  // Ramp power up from 0% to 25%
  for (int pwm = 0; pwm <= MAX_PWM; pwm++) {
    analogWrite(MOTOR_M1_FR, pwm);
    analogWrite(MOTOR_M2_RR, pwm);
    analogWrite(MOTOR_M3_RL, pwm);
    analogWrite(MOTOR_M4_FL, pwm);
    delay(STEP_DELAY_MS);
  }

  // Ramp power down from 25% to 0%
  for (int pwm = MAX_PWM; pwm >= 0; pwm--) {
    analogWrite(MOTOR_M1_FR, pwm);
    analogWrite(MOTOR_M2_RR, pwm);
    analogWrite(MOTOR_M3_RL, pwm);
    analogWrite(MOTOR_M4_FL, pwm);
    delay(STEP_DELAY_MS);
  }

  // Brief 1-second pause at 0% before repeating
  delay(1000);
}