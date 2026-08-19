#include <Arduino.h>

// Neck servo only. Drive motors and micro-ROS are deliberately absent from
// this firmware, so flashing it cannot command the robot base.
constexpr int SERVO_PIN = 2;
constexpr int SERVO_CHANNEL = 0;
constexpr int SERVO_FREQUENCY = 50;
constexpr int SERVO_RESOLUTION = 16;

// Conservative first sweep: widen these only after confirming the neck has
// clearance at both ends. Servo pulse limits match the existing robot code.
constexpr int MIN_ANGLE = 80;
constexpr int MAX_ANGLE = 150;
constexpr int STEP_DEGREES = 2;
constexpr unsigned long STEP_DELAY_MS = 80;
constexpr int MIN_PULSE_US = 500;
constexpr int MAX_PULSE_US = 2500;

int angle = MIN_ANGLE;

void setServoAngle(int value) {
  value = constrain(value, 0, 180);
  const int pulse_us = map(value, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
  const uint32_t duty = (uint32_t)pulse_us * ((1UL << SERVO_RESOLUTION) - 1) /
                        (1000000UL / SERVO_FREQUENCY);
  ledcWrite(SERVO_CHANNEL, duty);
}

void setup() {
  ledcSetup(SERVO_CHANNEL, SERVO_FREQUENCY, SERVO_RESOLUTION);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  setServoAngle(angle);
  delay(1000);
}

void loop() {
  setServoAngle(angle);
  if (angle < MAX_ANGLE) {
    delay(STEP_DELAY_MS);
    angle += STEP_DEGREES;
  } else {
    angle = MAX_ANGLE;
    delay(1000);
  }
}
