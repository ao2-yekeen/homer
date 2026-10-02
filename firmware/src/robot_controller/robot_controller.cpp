#include "robot_controller.h"

#include "../config/robot_config.h"

namespace homer {

RobotController::RobotController() : odometry_(encoders_) {}

void RobotController::begin(uint32_t now_ms) {
  if (config::kEnableEncoderOdometry) {
    encoders_.begin();
    odometry_.begin(now_ms);
  }
  drive_.begin();
  // The Pi teleop coordinator may already be in TELEOP when the ESP32
  // reconnects after a reset. Start in that mode so its dead-man command
  // stream is not silently discarded until the operator presses A again.
  mode_ = RobotMode::kTeleop;
  last_active_command_ms_ = now_ms;
}

void RobotController::initializeNeck(uint32_t now_ms) {
  neck_.begin(now_ms);
}

void RobotController::setMode(RobotMode mode, uint32_t now_ms) {
  if (mode_override_) return;
  if (mode == mode_) return;
  mode_ = mode;
  last_active_command_ms_ = now_ms;
  last_drive_command_ = {};
  drive_.stop();
}

void RobotController::setModeOverride(RobotMode mode, uint32_t now_ms) {
  mode_override_ = true;
  if (mode == mode_) return;
  mode_ = mode;
  last_active_command_ms_ = now_ms;
  last_drive_command_ = {};
  drive_.stop();
}

void RobotController::clearModeOverride(uint32_t now_ms) {
  mode_override_ = false;
  setMode(RobotMode::kTeleop, now_ms);
}

void RobotController::receiveDriveCommand(RobotMode source,
                                           const DriveCommand& command,
                                           uint32_t now_ms) {
  if (mode_override_) return;
  if (source != mode_) return;
  last_active_command_ms_ = now_ms;
  last_drive_command_ = command;
  drive_.apply(command);
}

void RobotController::receiveOverrideDriveCommand(const DriveCommand& command,
                                                   uint32_t now_ms) {
  if (!mode_override_ || mode_ == RobotMode::kStopped) return;
  last_active_command_ms_ = now_ms;
  last_drive_command_ = command;
  drive_.apply(command);
}

void RobotController::setNeckAngle(int angle) {
  neck_.setTargetAngle(angle);
}

void RobotController::stopDrive() {
  drive_.stop();
  last_drive_command_ = {};
}

void RobotController::update(uint32_t now_ms) {
  neck_.update(now_ms);
  if (config::kEnableEncoderOdometry) odometry_.update(now_ms);
}

void RobotController::enforceSafetyTimeout(uint32_t now_ms) {
  if (mode_ != RobotMode::kStopped &&
      now_ms - last_active_command_ms_ > config::kCommandTimeoutMs) {
    drive_.stop();
    last_drive_command_ = {};
    // Keep the selected mode. A fresh dead-man command can resume after a
    // brief transport gap, while the motor output stays off during the gap.
  }
}

}  // namespace homer
