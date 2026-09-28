#include <Arduino.h>

#include "robot_config.h"

// Stationary calibration image: no drive, neck, or ROS command handling.
// Turn the wheels by hand. Motor driver inputs and PWM pins remain low.
namespace {

volatile int32_t left_count = 0;
volatile int32_t right_count = 0;
int32_t previous_left = 0;
int32_t previous_right = 0;
uint32_t previous_ms = 0;

void IRAM_ATTR countLeft() {
  left_count += digitalRead(homer::config::kLeftEncoderB) ? -1 : 1;
}

void IRAM_ATTR countRight() {
  right_count += digitalRead(homer::config::kRightEncoderB) ? 1 : -1;
}

void disableMotors() {
  const uint8_t pins[] = {
      homer::config::kLeftMotorIn1, homer::config::kLeftMotorIn2,
      homer::config::kLeftMotorPwmPin, homer::config::kRightMotorIn1,
      homer::config::kRightMotorIn2, homer::config::kRightMotorPwmPin,
  };
  for (const uint8_t pin : pins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
}

}  // namespace

void setup() {
  disableMotors();
  pinMode(homer::config::kLeftEncoderA, INPUT);
  pinMode(homer::config::kLeftEncoderB, INPUT);
  pinMode(homer::config::kRightEncoderA, INPUT);
  pinMode(homer::config::kRightEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(homer::config::kLeftEncoderA), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(homer::config::kRightEncoderA), countRight, RISING);
  Serial.begin(homer::config::kSerialBaud);
  Serial.println("elapsed_ms,left_count,right_count,left_delta,right_delta,left_m,right_m,linear_mps,angular_radps");
  previous_ms = millis();
}

void loop() {
  const uint32_t now_ms = millis();
  const uint32_t elapsed_ms = now_ms - previous_ms;
  if (elapsed_ms < homer::config::kOdometryPeriodMs) return;

  int32_t left = 0;
  int32_t right = 0;
  noInterrupts();
  left = left_count;
  right = right_count;
  interrupts();

  const int32_t left_delta = left - previous_left;
  const int32_t right_delta = right - previous_right;
  const double meters_per_tick =
      PI * homer::config::kWheelDiameterMeters /
      homer::config::kEncoderCountsPerWheelTurn;
  const double left_m = left_delta * meters_per_tick;
  const double right_m = right_delta * meters_per_tick;
  const double seconds = elapsed_ms / 1000.0;
  const double linear_mps = (left_m + right_m) / (2.0 * seconds);
  const double angular_radps =
      (right_m - left_m) / (homer::config::kTrackWidthMeters * seconds);

  Serial.printf("%lu,%ld,%ld,%ld,%ld,%.6f,%.6f,%.6f,%.6f\n",
                static_cast<unsigned long>(elapsed_ms), static_cast<long>(left),
                static_cast<long>(right), static_cast<long>(left_delta),
                static_cast<long>(right_delta), left_m, right_m, linear_mps,
                angular_radps);
  previous_left = left;
  previous_right = right;
  previous_ms = now_ms;
}
