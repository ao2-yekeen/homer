#include <Arduino.h>

// Standalone motor-driver diagnostic. This intentionally has no ROS, serial
// transport, encoder, or gamepad code: it tests only the configured ESP32
// output pins and the motor driver. Use only with wheels safely lifted.
namespace {

constexpr uint8_t kRightIn1 = 25;
constexpr uint8_t kRightIn2 = 33;
constexpr uint8_t kRightPwmPin = 32;
constexpr uint8_t kLeftIn1 = 27;
constexpr uint8_t kLeftIn2 = 14;
constexpr uint8_t kLeftPwmPin = 12;

constexpr uint8_t kRightPwmChannel = 0;
constexpr uint8_t kLeftPwmChannel = 1;
constexpr uint32_t kPwmFrequency = 5000;
constexpr uint8_t kPwmResolution = 8;
constexpr uint8_t kTestDuty = 140;

void setMotors(int8_t direction, uint8_t duty) {
  // Both values represent the physical forward/reverse direction. The left
  // drive is mirror-mounted, hence its reversed direction-pin polarity.
  digitalWrite(kRightIn1, direction >= 0 ? HIGH : LOW);
  digitalWrite(kRightIn2, direction >= 0 ? LOW : HIGH);
  digitalWrite(kLeftIn1, direction >= 0 ? LOW : HIGH);
  digitalWrite(kLeftIn2, direction >= 0 ? HIGH : LOW);
  ledcWrite(kRightPwmChannel, duty);
  ledcWrite(kLeftPwmChannel, duty);
}

void stopMotors() {
  ledcWrite(kRightPwmChannel, 0);
  ledcWrite(kLeftPwmChannel, 0);
}

}  // namespace

void setup() {
  pinMode(kRightIn1, OUTPUT);
  pinMode(kRightIn2, OUTPUT);
  pinMode(kLeftIn1, OUTPUT);
  pinMode(kLeftIn2, OUTPUT);
  ledcSetup(kRightPwmChannel, kPwmFrequency, kPwmResolution);
  ledcSetup(kLeftPwmChannel, kPwmFrequency, kPwmResolution);
  ledcAttachPin(kRightPwmPin, kRightPwmChannel);
  ledcAttachPin(kLeftPwmPin, kLeftPwmChannel);

  stopMotors();
  delay(1000);
  setMotors(1, kTestDuty);
  delay(4000);
  stopMotors();
  delay(1000);
  setMotors(-1, kTestDuty);
  delay(4000);
  stopMotors();
}

void loop() {
  // The completed diagnostic remains electrically stopped until reflashed.
  delay(1000);
}
