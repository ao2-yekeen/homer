#pragma once

#include <Arduino.h>

namespace homer {

class NeckServo {
 public:
  void begin(uint32_t now_ms);
  void setTargetAngle(int angle);
  void update(uint32_t now_ms);

 private:
  void writeAngle(int angle);
  bool initialized_ = false;
  int current_angle_ = 0;
  int target_angle_ = 0;
  uint32_t last_step_ms_ = 0;
};

}  // namespace homer
