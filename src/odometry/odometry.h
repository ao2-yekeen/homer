#pragma once

#include <Arduino.h>

#include "../wheel_encoders/wheel_encoders.h"

namespace homer {

struct OdometryState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
};

class Odometry {
 public:
  explicit Odometry(const WheelEncoders& encoders) : encoders_(encoders) {}
  void begin(uint32_t now_ms);
  bool update(uint32_t now_ms);
  const OdometryState& state() const { return state_; }

 private:
  const WheelEncoders& encoders_;
  int32_t last_left_ = 0;
  int32_t last_right_ = 0;
  uint32_t last_ms_ = 0;
  OdometryState state_;
};

}  // namespace homer
