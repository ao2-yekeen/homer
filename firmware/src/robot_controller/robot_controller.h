#pragma once

#include <Arduino.h>

#include "../differential_drive/differential_drive.h"
#include "../neck_servo/neck_servo.h"
#include "../odometry/odometry.h"
#include "../wheel_encoders/wheel_encoders.h"

namespace homer {

enum class RobotMode { kTeleop, kAutonomous, kStopped };

class RobotController {
 public:
  RobotController();
  void begin(uint32_t now_ms);
  void initializeNeck(uint32_t now_ms);
  void setMode(RobotMode mode, uint32_t now_ms);
  void setModeOverride(RobotMode mode, uint32_t now_ms);
  void clearModeOverride(uint32_t now_ms);
  void receiveDriveCommand(RobotMode source, const DriveCommand& command,
                           uint32_t now_ms);
  void receiveOverrideDriveCommand(const DriveCommand& command,
                                   uint32_t now_ms);
  void setNeckAngle(int angle);
  void stopDrive();
  void update(uint32_t now_ms);
  void enforceSafetyTimeout(uint32_t now_ms);
  const OdometryState& odometry() const { return odometry_.state(); }
  RobotMode mode() const { return mode_; }
  DriveCommand lastDriveCommand() const { return last_drive_command_; }
  uint32_t lastActiveCommandMs() const { return last_active_command_ms_; }

 private:
  DifferentialDrive drive_;
  WheelEncoders encoders_;
  Odometry odometry_;
  NeckServo neck_;
  RobotMode mode_ = RobotMode::kStopped;
  bool mode_override_ = false;
  uint32_t last_active_command_ms_ = 0;
  DriveCommand last_drive_command_{};
};

}  // namespace homer
