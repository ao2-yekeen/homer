#include "robot_controller.h"

#include "../config/robot_config.h"

namespace homer {

RobotController::RobotController() : odometry_(encoders_) {}

void RobotController::begin(uint32_t now_ms) {
  encoders_.begin();
  odometry_.begin(now_ms);
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
  mode_ = mode;
  last_active_command_ms_ = now_ms;
  drive_.stop();
}

void RobotController::receiveDriveCommand(RobotMode source,
                                           const DriveCommand& command,
                                           uint32_t now_ms) {
  // A live teleop stream is an explicit operator takeover.  Accept it even
  // if the Pi's mode message arrived before an ESP32 reconnect; otherwise a
  // stale AUTONOMOUS state silently discards every gamepad command.
  if (source == RobotMode::kTeleop && mode_ != RobotMode::kTeleop) {
    mode_ = RobotMode::kTeleop;
  } else if (source != mode_) {
    return;
  }
  last_active_command_ms_ = now_ms;
  drive_.apply(command);
}

void RobotController::setNeckAngle(int angle) {
  neck_.setTargetAngle(angle);
}

void RobotController::update(uint32_t now_ms) {
  neck_.update(now_ms);
  odometry_.update(now_ms);
}

void RobotController::enforceSafetyTimeout(uint32_t now_ms) {
  if (mode_ != RobotMode::kStopped &&
      now_ms - last_active_command_ms_ > config::kCommandTimeoutMs) {
    drive_.stop();
    mode_ = RobotMode::kStopped;
  }
}

}  // namespace homer
