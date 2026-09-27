#include "serial_link/serial_link.h"
#include "robot_controller/robot_controller.h"

namespace {
homer::RobotController robot;
homer::SerialLink link(robot);
}

void setup() {
  const uint32_t now_ms = millis();
  robot.begin(now_ms);
  link.begin();
  robot.initializeNeck(millis());
}

void loop() {
  robot.enforceSafetyTimeout(millis());
  link.spin(millis());
  const uint32_t now_ms = millis();
  robot.update(now_ms);
  robot.enforceSafetyTimeout(now_ms);
}
