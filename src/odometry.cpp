#include "odometry.h"

#include <math.h>

#include "robot_config.h"

namespace homer {

void Odometry::begin(uint32_t now_ms) {
  encoders_.readCounts(last_left_, last_right_);
  last_ms_ = now_ms;
  state_ = {};
}

bool Odometry::update(uint32_t now_ms) {
  const uint32_t elapsed_ms = now_ms - last_ms_;
  if (elapsed_ms == 0) return false;
  int32_t left = 0;
  int32_t right = 0;
  encoders_.readCounts(left, right);
  const double meters_per_tick =
      (config::kWheelDiameterMeters * PI) /
      config::kEncoderCountsPerWheelTurn;
  const double dl = (left - last_left_) * meters_per_tick;
  const double dr = (right - last_right_) * meters_per_tick;
  const double distance = (dl + dr) * 0.5;
  const double heading_delta = (dr - dl) / config::kTrackWidthMeters;
  state_.x += distance * cos(state_.heading + heading_delta * 0.5);
  state_.y += distance * sin(state_.heading + heading_delta * 0.5);
  state_.heading += heading_delta;
  state_.linear_velocity = distance * 1000.0 / elapsed_ms;
  state_.angular_velocity = heading_delta * 1000.0 / elapsed_ms;
  last_left_ = left;
  last_right_ = right;
  last_ms_ = now_ms;
  return true;
}

}  // namespace homer
