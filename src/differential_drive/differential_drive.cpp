#include "differential_drive.h"

#include <Arduino.h>

#include "../config/robot_config.h"

namespace homer {

void DifferentialDrive::begin() {
  pinMode(config::kRightMotorIn1, OUTPUT);
  pinMode(config::kRightMotorIn2, OUTPUT);
  pinMode(config::kLeftMotorIn1, OUTPUT);
  pinMode(config::kLeftMotorIn2, OUTPUT);
  ledcSetup(config::kRightMotorPwmChannel, config::kMotorPwmFrequency,
            config::kMotorPwmResolution);
  ledcAttachPin(config::kRightMotorPwmPin, config::kRightMotorPwmChannel);
  ledcSetup(config::kLeftMotorPwmChannel, config::kMotorPwmFrequency,
            config::kMotorPwmResolution);
  ledcAttachPin(config::kLeftMotorPwmPin, config::kLeftMotorPwmChannel);
  stop();
}

void DifferentialDrive::apply(const DriveCommand& command) {
  const int left = static_cast<int>(
      (command.linear * config::kLinearCommandScale -
       command.angular * config::kAngularCommandScale) *
      config::kLeftDriveTrim);
  const int right = static_cast<int>(
      command.linear * config::kLinearCommandScale +
      command.angular * config::kAngularCommandScale);
  setWheelDuty(left, right);
}

void DifferentialDrive::stop() { setWheelDuty(0, 0); }

void DifferentialDrive::setWheelDuty(int left, int right) {
  constexpr int kMaxDuty = (1 << config::kMotorPwmResolution) - 1;
  left = constrain(left, -kMaxDuty, kMaxDuty);
  right = constrain(right, -kMaxDuty, kMaxDuty);
  digitalWrite(config::kRightMotorIn1, right >= 0 ? HIGH : LOW);
  digitalWrite(config::kRightMotorIn2, right >= 0 ? LOW : HIGH);
  digitalWrite(config::kLeftMotorIn1, left >= 0 ? LOW : HIGH);
  digitalWrite(config::kLeftMotorIn2, left >= 0 ? HIGH : LOW);
  ledcWrite(config::kRightMotorPwmChannel, abs(right));
  ledcWrite(config::kLeftMotorPwmChannel, abs(left));
}

}  // namespace homer
