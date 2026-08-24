#include "robot_controller.h"

#include "robot_config.h"

namespace homer {

RobotController::RobotController() : odometry_(encoders_) {}

void RobotController::begin(uint32_t now_ms) {
  encoders_.begin();
  odometry_.begin(now_ms);
  drive_.begin();
  mode_ = RobotMode::kStopped;
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
  if (source != mode_) return;
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
