#include <Arduino.h>

#include "robot_config.h"

// Stationary calibration image: no drive, neck, or ROS command handling.
// Turn the wheels by hand. Motor driver inputs and PWM pins remain low.
namespace {

volatile int32_t left_count = 0;
volatile int32_t right_count = 0;
volatile uint32_t left_b_edges = 0;
volatile uint32_t right_b_edges = 0;
int32_t previous_left = 0;
int32_t previous_right = 0;
uint32_t previous_ms = 0;

void IRAM_ATTR countLeft() {
  left_count += digitalRead(homer::config::kLeftEncoderB) ? -1 : 1;
}

void IRAM_ATTR countRight() {
  right_count += digitalRead(homer::config::kRightEncoderB) ? -1 : 1;
}

void IRAM_ATTR countLeftB() { ++left_b_edges; }

void IRAM_ATTR countRightB() { ++right_b_edges; }

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
  attachInterrupt(digitalPinToInterrupt(homer::config::kLeftEncoderB), countLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(homer::config::kRightEncoderB), countRightB, CHANGE);
  Serial.begin(homer::config::kSerialBaud);
  Serial.println("elapsed_ms,left_count,right_count,left_b_edges,right_b_edges,left_delta,right_delta,left_m,right_m,linear_mps,angular_radps");
  previous_ms = millis();
}

void loop() {
  const uint32_t now_ms = millis();
  const uint32_t elapsed_ms = now_ms - previous_ms;
  if (elapsed_ms < homer::config::kOdometryPeriodMs) return;

  int32_t left = 0;
  int32_t right = 0;
  uint32_t left_b = 0;
  uint32_t right_b = 0;
  noInterrupts();
  left = left_count;
  right = right_count;
  left_b = left_b_edges;
  right_b = right_b_edges;
  interrupts();

  const int32_t left_delta = left - previous_left;
  const int32_t right_delta = right - previous_right;
  const double wheel_circumference = PI * homer::config::kWheelDiameterMeters;
  const double left_m = left_delta * wheel_circumference /
                        homer::config::kLeftEncoderCountsPerWheelTurn;
  const double right_m = right_delta * wheel_circumference /
                         homer::config::kRightEncoderCountsPerWheelTurn;
  const double seconds = elapsed_ms / 1000.0;
  const double linear_mps = (left_m + right_m) / (2.0 * seconds);
  const double angular_radps =
      (right_m - left_m) / (homer::config::kTrackWidthMeters * seconds);

  Serial.printf("%lu,%ld,%ld,%lu,%lu,%ld,%ld,%.6f,%.6f,%.6f,%.6f\n",
                static_cast<unsigned long>(elapsed_ms), static_cast<long>(left),
                static_cast<long>(right), static_cast<unsigned long>(left_b),
                static_cast<unsigned long>(right_b), static_cast<long>(left_delta),
                static_cast<long>(right_delta), left_m, right_m, linear_mps,
                angular_radps);
  previous_left = left;
  previous_right = right;
  previous_ms = now_ms;
}
