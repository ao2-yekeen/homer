#include <Arduino.h>

// Standalone full-power motor diagnostic. Both motor outputs stay at full PWM
// until the ESP32 is reset, reflashed, or motor power is removed.
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
constexpr uint8_t kTestDuty = 255;

void setMotors(uint8_t duty) {
  // Forward polarity matches the known-good original motor diagnostic.
  digitalWrite(kRightIn1, HIGH);
  digitalWrite(kRightIn2, LOW);
  digitalWrite(kLeftIn1, LOW);
  digitalWrite(kLeftIn2, HIGH);
  ledcWrite(kRightPwmChannel, duty);
  ledcWrite(kLeftPwmChannel, duty);
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

  setMotors(kTestDuty);
}

void loop() {
  // Hardware PWM continues at full duty.
  delay(1000);
}
