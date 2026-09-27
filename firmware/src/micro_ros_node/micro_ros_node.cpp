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
}

bool MicroRosNode::createEntities() {
  allocator_ = rcl_get_default_allocator();
  if (!check(rclc_support_init(&support_, 0, nullptr, &allocator_))) return false;
  support_initialized_ = true;
  if (!check(rclc_node_init_default(&node_, "mobile_robot", "", &support_))) return false;
  if (!check(rclc_subscription_init_default(
      &autonomous_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"))) return false;
  if (!check(rclc_subscription_init_default(
      &teleop_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "teleop/cmd_vel"))) return false;
  if (!check(rclc_subscription_init_default(
      &mode_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "set_autonomous"))) return false;
  if (!check(rclc_subscription_init_default(
      &mode_override_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "mode_override"))) return false;
  if (!check(rclc_subscription_init_default(
      &neck_sub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teleop/neck_angle"))) return false;
  // Odometry is larger than one 512-byte XRCE serial frame. Reliable transport
  // fragments it; best-effort transport cannot deliver this message.
  if (!check(rclc_publisher_init_default(
      &odom_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"))) return false;
  if (rmw_uros_set_publisher_session_timeout(
      rcl_publisher_get_rmw_handle(&odom_pub_), 150) != RMW_RET_OK) return false;
  // The small drive status message fits in a best-effort frame.
  if (!check(rclc_publisher_init_best_effort(
      &drive_status_pub_, &node_,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "drive/status"))) return false;
  odom_message_.header.frame_id.data = const_cast<char*>("odom");
  odom_message_.header.frame_id.size = 4;
  odom_message_.header.frame_id.capacity = 5;
  odom_message_.child_frame_id.data = const_cast<char*>("base_link");
  odom_message_.child_frame_id.size = 9;
  odom_message_.child_frame_id.capacity = 10;
  odom_message_.pose.pose.orientation.w = 1.0;
  if (!check(rclc_executor_init(&executor_, &support_.context, 5, &allocator_))) return false;
  executor_initialized_ = true;
  if (!check(rclc_executor_add_subscription(
      &executor_, &autonomous_sub_, &autonomous_message_,
      &onAutonomousCommand, ON_NEW_DATA))) return false;
  if (!check(rclc_executor_add_subscription(
      &executor_, &teleop_sub_, &teleop_message_, &onTeleopCommand, ON_NEW_DATA))) return false;
  if (!check(rclc_executor_add_subscription(
      &executor_, &mode_sub_, &mode_message_, &onMode, ON_NEW_DATA))) return false;
  if (!check(rclc_executor_add_subscription(
      &executor_, &mode_override_sub_, &mode_override_message_,
      &onModeOverride, ON_NEW_DATA))) return false;
  if (!check(rclc_executor_add_subscription(
      &executor_, &neck_sub_, &neck_message_, &onNeck, ON_NEW_DATA))) return false;
  return true;
}

void MicroRosNode::spin(uint32_t now_ms) {
  if (!connected_ && now_ms - last_agent_check_ms_ >= 500U) {
    last_agent_check_ms_ = now_ms;
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      connected_ = createEntities();
      if (!connected_) destroyEntities();
    }
  }
  if (!connected_) return;
  const rcl_ret_t spin_result = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(2));
  if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
    destroyEntities();
    connected_ = false;
    return;
  }
  if (now_ms - last_odom_publish_ms_ >= config::kOdometryPeriodMs) {
    last_odom_publish_ms_ = now_ms;
    if (publishOdometry(now_ms) == RCL_RET_OK) {
      odom_publish_failures_ = 0;
    } else if (++odom_publish_failures_ >= 3) {
      destroyEntities();
      connected_ = false;
      odom_publish_failures_ = 0;
      return;
    }
  }
  publishDriveStatus(millis());
}

void MicroRosNode::destroyEntities() {
  if (!support_initialized_) return;
  rmw_context_t* context = rcl_context_get_rmw_context(&support_.context);
  if (context != nullptr) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
  }
  if (executor_initialized_) (void)rclc_executor_fini(&executor_);
  if (rcl_publisher_is_valid(&drive_status_pub_))
    (void)rcl_publisher_fini(&drive_status_pub_, &node_);
  if (rcl_publisher_is_valid(&odom_pub_))
    (void)rcl_publisher_fini(&odom_pub_, &node_);
  if (rcl_subscription_is_valid(&neck_sub_))
    (void)rcl_subscription_fini(&neck_sub_, &node_);
  if (rcl_subscription_is_valid(&mode_override_sub_))
    (void)rcl_subscription_fini(&mode_override_sub_, &node_);
  if (rcl_subscription_is_valid(&mode_sub_))
    (void)rcl_subscription_fini(&mode_sub_, &node_);
  if (rcl_subscription_is_valid(&teleop_sub_))
    (void)rcl_subscription_fini(&teleop_sub_, &node_);
  if (rcl_subscription_is_valid(&autonomous_sub_))
    (void)rcl_subscription_fini(&autonomous_sub_, &node_);
  if (rcl_node_is_valid(&node_)) (void)rcl_node_fini(&node_);
  (void)rclc_support_fini(&support_);
  support_initialized_ = false;
  executor_initialized_ = false;
  support_ = {};
  node_ = {};
  executor_ = {};
  autonomous_sub_ = {};
  teleop_sub_ = {};
  mode_sub_ = {};
  mode_override_sub_ = {};
  neck_sub_ = {};
  odom_pub_ = {};
  drive_status_pub_ = {};
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

rcl_ret_t MicroRosNode::publishOdometry(uint32_t now_ms) {
  fillOdometryMessage(now_ms);
  return rcl_publish(&odom_pub_, &odom_message_, nullptr);
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

bool MicroRosNode::check(rcl_ret_t result) {
  return result == RCL_RET_OK;
}

}  // namespace homer
