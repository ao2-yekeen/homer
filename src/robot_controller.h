#pragma once

#include <Arduino.h>

#include "differential_drive.h"
#include "neck_servo.h"
#include "odometry.h"
#include "wheel_encoders.h"

namespace homer {

enum class RobotMode { kTeleop, kAutonomous, kStopped };

class RobotController {
 public:
  RobotController();
  void begin(uint32_t now_ms);
  void initializeNeck(uint32_t now_ms);
  void setMode(RobotMode mode, uint32_t now_ms);
  void receiveDriveCommand(RobotMode source, const DriveCommand& command,
                           uint32_t now_ms);
  void setNeckAngle(int angle);
  void update(uint32_t now_ms);
  void enforceSafetyTimeout(uint32_t now_ms);
  const OdometryState& odometry() const { return odometry_.state(); }

 private:
  DifferentialDrive drive_;
  WheelEncoders encoders_;
  Odometry odometry_;
  NeckServo neck_;
  RobotMode mode_ = RobotMode::kStopped;
  uint32_t last_active_command_ms_ = 0;
};

}  // namespace homer
