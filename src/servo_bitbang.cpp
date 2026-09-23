#include <Arduino.h>

// Direct GPIO test for the neck servo. This deliberately bypasses LEDC and
// micro-ROS: a high pulse is generated on GPIO 2 once per 20 ms frame.
constexpr uint8_t kServoPin = 2;
constexpr int kMinimumAngle = 80;
constexpr int kMaximumAngle = 150;
constexpr int kMinimumPulseUs = 500;
constexpr int kMaximumPulseUs = 2500;
constexpr uint8_t kFramesPerStep = 5;
constexpr uint32_t kFrameUs = 20000;

int angle = kMinimumAngle;
int direction = 1;
uint8_t frame_count = 0;

void emitServoPulse(int value) {
  const uint32_t pulse_us = map(value, 0, 180, kMinimumPulseUs,
                                kMaximumPulseUs);
  digitalWrite(kServoPin, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(kServoPin, LOW);
  delayMicroseconds(kFrameUs - pulse_us);
}

void setup() {
  pinMode(kServoPin, OUTPUT);
  digitalWrite(kServoPin, LOW);
}

void loop() {
  emitServoPulse(angle);
  if (++frame_count < kFramesPerStep) {
    return;
  }

  frame_count = 0;
  angle += direction;
  if (angle >= kMaximumAngle) {
    angle = kMaximumAngle;
    direction = -1;
  } else if (angle <= kMinimumAngle) {
    angle = kMinimumAngle;
    direction = 1;
  }
}
