#pragma once

#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "robot_controller.h"

namespace homer {

class MicroRosNode {
 public:
  explicit MicroRosNode(RobotController& robot) : robot_(robot) {}
  void begin();
  void spin(uint32_t now_ms);

 private:
  static MicroRosNode* instance_;
  static void onAutonomousCommand(const void* message);
  static void onTeleopCommand(const void* message);
  static void onMode(const void* message);
  static void onNeck(const void* message);
  static void check(rcl_ret_t result);
  void publishOdometry(uint32_t now_ms);
  void fillOdometryMessage(uint32_t now_ms);

  RobotController& robot_;
  rcl_allocator_t allocator_{};
  rclc_support_t support_{};
  rcl_node_t node_{};
  rclc_executor_t executor_{};
  rcl_subscription_t autonomous_sub_{};
  rcl_subscription_t teleop_sub_{};
  rcl_subscription_t mode_sub_{};
  rcl_subscription_t neck_sub_{};
  rcl_publisher_t odom_pub_{};
  geometry_msgs__msg__Twist autonomous_message_{};
  geometry_msgs__msg__Twist teleop_message_{};
  std_msgs__msg__Bool mode_message_{};
  std_msgs__msg__Int32 neck_message_{};
  nav_msgs__msg__Odometry odom_message_{};
  uint32_t last_odom_publish_ms_ = 0;
};

}  // namespace homer
