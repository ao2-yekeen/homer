#include <Arduino.h>

#include "micro_ros_node/micro_ros_node.h"
#include "robot_controller/robot_controller.h"

namespace {

homer::RobotController robot;
homer::MicroRosNode ros(robot);

}  // namespace

void setup() {
  const uint32_t now_ms = millis();
  robot.begin(now_ms);
  ros.begin();
  robot.initializeNeck(millis());
}

void loop() {
  const uint32_t now_ms = millis();
  ros.spin(now_ms);
  robot.update(now_ms);
  robot.enforceSafetyTimeout(now_ms);
}
