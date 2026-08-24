#pragma once

#include <Arduino.h>

namespace homer {

class WheelEncoders {
 public:
  void begin();
  void readCounts(int32_t& left, int32_t& right) const;

 private:
  static void IRAM_ATTR onRightEdge();
  static void IRAM_ATTR onLeftEdge();
  void IRAM_ATTR countRight();
  void IRAM_ATTR countLeft();

  static WheelEncoders* instance_;
  volatile int32_t left_count_ = 0;
  volatile int32_t right_count_ = 0;
};

}  // namespace homer
