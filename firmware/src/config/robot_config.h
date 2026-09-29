#pragma once

#include <Arduino.h>

namespace homer::config {

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr uint32_t kOdometryPeriodMs = 200;

// Physical sides confirmed by isolated hand turns on 2026-09-29.
constexpr uint8_t kLeftEncoderA = 23;
constexpr uint8_t kLeftEncoderB = 22;
constexpr uint8_t kRightEncoderA = 4;
constexpr uint8_t kRightEncoderB = 19;
// Marked wheel turns: physical left about 379, physical right about 502.
constexpr double kLeftEncoderCountsPerWheelTurn = 379.0;
constexpr double kRightEncoderCountsPerWheelTurn = 502.0;
constexpr double kWheelDiameterMeters = 0.065;
constexpr double kTrackWidthMeters = 0.33;

constexpr uint8_t kRightMotorIn1 = 25;
constexpr uint8_t kRightMotorIn2 = 33;
constexpr uint8_t kRightMotorPwmPin = 32;
constexpr uint8_t kLeftMotorIn1 = 27;
constexpr uint8_t kLeftMotorIn2 = 14;
constexpr uint8_t kLeftMotorPwmPin = 12;
constexpr int kRightMotorPwmChannel = 0;
constexpr int kLeftMotorPwmChannel = 1;
constexpr int kMotorPwmFrequency = 5000;
constexpr int kMotorPwmResolution = 8;
// The observed 0.675 m/s manual command maps to about 148/255 duty before
// turn correction, above the 140/255 used by the lifted-wheel diagnostic.
// Tune against measured wheel motion before treating this as calibrated speed.
constexpr float kLinearCommandScale = 220.0f;
// Map the maximum teleop angular command (0.4) to full 8-bit motor duty.
// The differential-drive output clamps at +/-255, so combined drive and
// steering commands may saturate one wheel while turning.
constexpr float kAngularCommandScale = 637.5f;
constexpr float kLeftDriveTrim = 1.00f;

constexpr uint8_t kNeckServoPin = 2;
constexpr int kNeckServoPwmChannel = 2;
constexpr int kNeckServoFrequency = 50;
constexpr int kNeckServoResolution = 16;
constexpr int kNeckMinimumPulseUs = 500;
constexpr int kNeckMaximumPulseUs = 2500;
constexpr int kNeckMinimumAngle = 80;
constexpr int kNeckMaximumAngle = 150;
constexpr int kNeckSafeCenterAngle = 120;
constexpr uint32_t kNeckStepIntervalMs = 50;

}  // namespace homer::config
