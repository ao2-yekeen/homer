#pragma once

#include <Arduino.h>

#include "../robot_controller/robot_controller.h"

namespace homer {

class SerialLink {
 public:
  explicit SerialLink(RobotController& robot) : robot_(robot) {}
  void begin();
  void spin(uint32_t now_ms);

 private:
  void receiveByte(char byte, uint32_t now_ms);
  void handleFrame(uint32_t now_ms);
  void send(const char* body);
  void publishOdometry(uint32_t now_ms);
  void publishStatus(uint32_t now_ms);

  RobotController& robot_;
  char input_[128]{};
  uint8_t input_size_ = 0;
  bool discarding_ = false;
  uint16_t tx_sequence_ = 0;
  uint32_t boot_id_ = 0;
  uint32_t last_ready_ms_ = 0;
  uint32_t last_odom_ms_ = 0;
  uint32_t last_status_ms_ = 0;
};

}  // namespace homer
