#pragma once

#include <Arduino.h>

namespace homer::config {

constexpr uint32_t kMicroRosBaud = 921600;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr uint32_t kOdometryPeriodMs = 50;

constexpr uint8_t kRightEncoderA = 23;
constexpr uint8_t kRightEncoderB = 22;
constexpr uint8_t kLeftEncoderA = 4;
constexpr uint8_t kLeftEncoderB = 19;
constexpr double kEncoderCountsPerWheelTurn = 377.0;
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
constexpr float kLinearCommandScale = 120.0f;
constexpr float kAngularCommandScale = 60.0f;
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
