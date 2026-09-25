#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

namespace {

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32 message;
uint32_t last_publish_ms = 0;

void fail() {
  while (true) delay(100);
}

}  // namespace

void setup() {
  set_microros_transports();
  delay(2000);
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) fail();
  if (rclc_node_init_default(&node, "micro_ros_serial_check", "", &support) !=
      RCL_RET_OK) fail();
  if (rclc_publisher_init_default(
          &publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "micro_ros_serial_check") != RCL_RET_OK) fail();
}

void loop() {
  const uint32_t now_ms = millis();
  if (now_ms - last_publish_ms < 500) return;
  last_publish_ms = now_ms;
  message.data++;
  rcl_publish(&publisher, &message, nullptr);
}
