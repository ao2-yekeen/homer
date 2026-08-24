#include "neck_servo.h"

#include "robot_config.h"

namespace homer {

void NeckServo::begin(uint32_t now_ms) {
  ledcSetup(config::kNeckServoPwmChannel, config::kNeckServoFrequency,
            config::kNeckServoResolution);
  ledcAttachPin(config::kNeckServoPin, config::kNeckServoPwmChannel);
  current_angle_ = config::kNeckSafeCenterAngle;
  target_angle_ = current_angle_;
  writeAngle(current_angle_);
  last_step_ms_ = now_ms;
  initialized_ = true;
}

void NeckServo::setTargetAngle(int angle) {
  target_angle_ = constrain(angle, config::kNeckMinimumAngle,
                            config::kNeckMaximumAngle);
}

void NeckServo::update(uint32_t now_ms) {
  if (!initialized_ || current_angle_ == target_angle_ ||
      now_ms - last_step_ms_ < config::kNeckStepIntervalMs) {
    return;
  }
  last_step_ms_ = now_ms;
  current_angle_ += current_angle_ < target_angle_ ? 1 : -1;
  writeAngle(current_angle_);
}

void NeckServo::writeAngle(int angle) {
  const int pulse_us = map(angle, 0, 180, config::kNeckMinimumPulseUs,
                           config::kNeckMaximumPulseUs);
  const uint32_t duty = static_cast<uint32_t>(pulse_us) *
      ((1UL << config::kNeckServoResolution) - 1) /
      (1000000UL / config::kNeckServoFrequency);
  ledcWrite(config::kNeckServoPwmChannel, duty);
}

}  // namespace homer
