#include "micro_ros_node.h"

#include <math.h>
#include <micro_ros_arduino.h>

#include "../config/robot_config.h"

namespace homer {

MicroRosNode* MicroRosNode::instance_ = nullptr;

void MicroRosNode::begin() {
  instance_ = this;
  Serial.begin(config::kMicroRosBaud);
  set_microros_transports();
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
      &mode_override_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "mode_override"));
  check(rclc_subscription_init_default(
      &neck_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teleop/neck_angle"));
  // Telemetry must not hold up command reception on the serial transport.
  // Best-effort is appropriate for streaming odometry and status: a dropped
  // sample is harmless, whereas a reliable publish can delay the executor.
  check(rclc_publisher_init_best_effort(
      &odom_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  check(rclc_publisher_init_best_effort(
      &drive_status_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "drive/status"));
  odom_message_.header.frame_id.data = const_cast<char*>("odom");
  odom_message_.header.frame_id.size = 4;
  odom_message_.header.frame_id.capacity = 5;
  odom_message_.child_frame_id.data = const_cast<char*>("base_link");
  odom_message_.child_frame_id.size = 9;
  odom_message_.child_frame_id.capacity = 10;
  odom_message_.pose.pose.orientation.w = 1.0;
  check(rclc_executor_init(&executor_, &support_.context, 5, &allocator_));
  check(rclc_executor_add_subscription(
      &executor_, &autonomous_sub_, &autonomous_message_,
      &onAutonomousCommand, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &teleop_sub_, &teleop_message_, &onTeleopCommand, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &mode_sub_, &mode_message_, &onMode, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &mode_override_sub_, &mode_override_message_,
      &onModeOverride, ON_NEW_DATA));
  check(rclc_executor_add_subscription(
      &executor_, &neck_sub_, &neck_message_, &onNeck, ON_NEW_DATA));
}

void MicroRosNode::spin(uint32_t now_ms) {
  rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(2));
  publishOdometry(now_ms);
  publishDriveStatus(millis());
}

void MicroRosNode::publishDriveStatus(uint32_t now_ms) {
  if (now_ms - last_drive_status_publish_ms_ < 200) return;
  last_drive_status_publish_ms_ = now_ms;
  const DriveCommand command = robot_.lastDriveCommand();
  drive_status_message_.linear.x = command.linear;
  drive_status_message_.linear.y = ledcRead(config::kLeftMotorPwmChannel);
  drive_status_message_.linear.z = ledcRead(config::kRightMotorPwmChannel);
  drive_status_message_.angular.z = command.angular;
  drive_status_message_.angular.x = static_cast<int>(robot_.mode());
  drive_status_message_.angular.y =
      now_ms - robot_.lastActiveCommandMs();
  // Telemetry must never prevent the executor from receiving drive commands.
  // A transient serial/Agent error is recoverable; the next cycle can publish
  // again, while control and the dead-man timeout must continue to run.
  (void)rcl_publish(&drive_status_pub_, &drive_status_message_, nullptr);
}

void MicroRosNode::publishOdometry(uint32_t now_ms) {
  if (now_ms - last_odom_publish_ms_ < config::kOdometryPeriodMs) return;
  last_odom_publish_ms_ = now_ms;
  fillOdometryMessage(now_ms);
  (void)rcl_publish(&odom_pub_, &odom_message_, nullptr);
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
  const uint32_t now_ms = millis();
  instance_->robot_.receiveOverrideDriveCommand(command, now_ms);
  instance_->robot_.receiveDriveCommand(RobotMode::kAutonomous, command, now_ms);
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

void MicroRosNode::onModeOverride(const void* message) {
  const int32_t requested =
      static_cast<const std_msgs__msg__Int32*>(message)->data;
  const uint32_t now_ms = millis();
  if (requested < 0) {
    instance_->robot_.clearModeOverride(now_ms);
    return;
  }
  if (requested > static_cast<int32_t>(RobotMode::kStopped)) return;
  instance_->robot_.setModeOverride(static_cast<RobotMode>(requested), now_ms);
}

void MicroRosNode::onNeck(const void* message) {
  instance_->robot_.setNeckAngle(
      static_cast<const std_msgs__msg__Int32*>(message)->data);
}

void MicroRosNode::check(rcl_ret_t result) {
  if (result != RCL_RET_OK) while (true) delay(100);
}

}  // namespace homer
