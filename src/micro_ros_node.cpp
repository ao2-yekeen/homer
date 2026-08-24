#include "micro_ros_node.h"

#include <math.h>
#include <micro_ros_platformio.h>

#include "robot_config.h"

namespace homer {

MicroRosNode* MicroRosNode::instance_ = nullptr;

void MicroRosNode::begin() {
  instance_ = this;
  Serial.begin(config::kMicroRosBaud);
  set_microros_serial_transports(Serial);
  delay(2000);
  allocator_ = rcl_get_default_allocator();
  check(rclc_support_init(&support_, 0, nullptr, &allocator_));
  check(rclc_node_init_default(&node_, "mobile_robot", "", &support_));
  check(rclc_subscription_init_default(
      &autonomous_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  check(rclc_subscription_init_default(
      &teleop_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "teleop/cmd_vel"));
  check(rclc_subscription_init_default(
      &mode_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "set_autonomous"));
  check(rclc_subscription_init_default(
      &neck_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teleop/neck_angle"));
  check(rclc_publisher_init_default(
      &odom_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  odom_message_.header.frame_id.data = const_cast<char*>("odom");
  odom_message_.header.frame_id.size = 4;
  odom_message_.header.frame_id.capacity = 5;
  odom_message_.child_frame_id.data = const_cast<char*>("base_link");
  odom_message_.child_frame_id.size = 9;
  odom_message_.child_frame_id.capacity = 10;
  odom_message_.pose.pose.orientation.w = 1.0;
  check(rclc_executor_init(&executor_, &support_.context, 4, &allocator_));
  check(rclc_executor_add_subscription(
      &executor_, &autonomous_sub_, &autonomous_message_,
      &onAutonomousCommand, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &teleop_sub_, &teleop_message_, &onTeleopCommand, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &mode_sub_, &mode_message_, &onMode, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &neck_sub_, &neck_message_, &onNeck, ON_NEW_DATA));
}

void MicroRosNode::spin(uint32_t now_ms) {
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(2));
  publishOdometry(now_ms);
}

void MicroRosNode::publishOdometry(uint32_t now_ms) {
  if (now_ms - last_odom_publish_ms_ < config::kOdometryPeriodMs) return;
  last_odom_publish_ms_ = now_ms;
  fillOdometryMessage(now_ms);
  check(rcl_publish(&odom_pub_, &odom_message_, nullptr));
}

void MicroRosNode::fillOdometryMessage(uint32_t now_ms) {
  const OdometryState& state = robot_.odometry();
  odom_message_.header.stamp.sec = static_cast<int32_t>(now_ms / 1000);
  odom_message_.header.stamp.nanosec = (now_ms % 1000) * 1000000UL;
  odom_message_.pose.pose.position.x = state.x;
  odom_message_.pose.pose.position.y = state.y;
  odom_message_.pose.pose.orientation.z = sin(state.heading * 0.5);
  odom_message_.pose.pose.orientation.w = cos(state.heading * 0.5);
  odom_message_.twist.twist.linear.x = state.linear_velocity;
  odom_message_.twist.twist.angular.z = state.angular_velocity;
}

void MicroRosNode::onAutonomousCommand(const void* message) {
  const auto& twist = *static_cast<const geometry_msgs__msg__Twist*>(message);
  DriveCommand command;
  command.linear = static_cast<float>(twist.linear.x);
  command.angular = static_cast<float>(twist.angular.z);
  instance_->robot_.receiveDriveCommand(RobotMode::kAutonomous, command, millis());
}

void MicroRosNode::onTeleopCommand(const void* message) {
  const auto& twist = *static_cast<const geometry_msgs__msg__Twist*>(message);
  DriveCommand command;
  command.linear = static_cast<float>(twist.linear.x);
  command.angular = static_cast<float>(twist.angular.z);
  instance_->robot_.receiveDriveCommand(RobotMode::kTeleop, command, millis());
}

void MicroRosNode::onMode(const void* message) {
  const bool autonomous = static_cast<const std_msgs__msg__Bool*>(message)->data;
  instance_->robot_.setMode(autonomous ? RobotMode::kAutonomous : RobotMode::kTeleop,
                            millis());
}

void MicroRosNode::onNeck(const void* message) {
  instance_->robot_.setNeckAngle(
      static_cast<const std_msgs__msg__Int32*>(message)->data);
}

void MicroRosNode::check(rcl_ret_t result) {
  if (result != RCL_RET_OK) while (true) delay(100);
}

}  // namespace homer
