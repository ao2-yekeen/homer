#include <Arduino.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

namespace {

constexpr uint32_t kMicroRosBaud = 921600;
constexpr uint32_t kCommandTimeoutMs = 1000;

class WheelEncoders {
 public:
  void begin() {
    pinMode(kRightA, INPUT);
    pinMode(kRightB, INPUT);
    pinMode(kLeftA, INPUT);
    pinMode(kLeftB, INPUT);
    right_state_ = read(kRightA, kRightB);
    left_state_ = read(kLeftA, kLeftB);
    instance_ = this;
    attachInterrupt(digitalPinToInterrupt(kRightA), onRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kRightB), onRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kLeftA), onLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kLeftB), onLeft, CHANGE);
  }

  void readCounts(int32_t& left, int32_t& right) {
    noInterrupts();
    left = left_count_;
    right = right_count_;
    interrupts();
  }

 private:
  static constexpr uint8_t kRightA = 23;
  static constexpr uint8_t kRightB = 22;
  static constexpr uint8_t kLeftA = 4;
  static constexpr uint8_t kLeftB = 19;
  volatile int32_t left_count_ = 0;
  volatile int32_t right_count_ = 0;
  volatile uint8_t left_state_ = 0;
  volatile uint8_t right_state_ = 0;
  static inline WheelEncoders* instance_ = nullptr;

  static uint8_t read(uint8_t a, uint8_t b) { return (digitalRead(a) << 1) | digitalRead(b); }
  static int8_t delta(uint8_t index) {
    static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1,
                                     -1, 0, 0, 1, 0, 1, -1, 0};
    return table[index];
  }
  static void IRAM_ATTR onRight() { instance_->rightCount(); }
  static void IRAM_ATTR onLeft() { instance_->leftCount(); }
  void rightCount() {
    const uint8_t state = read(kRightA, kRightB);
    right_count_ += delta((right_state_ << 2) | state);
    right_state_ = state;
  }
  void leftCount() {
    const uint8_t state = read(kLeftA, kLeftB);
    left_count_ += delta((left_state_ << 2) | state);
    left_state_ = state;
  }
};

class Odometry {
 public:
  void begin(WheelEncoders& encoders) {
    encoders_ = &encoders;
    encoders_->readCounts(last_left_, last_right_);
    last_ms_ = millis();
  }

  void update(nav_msgs__msg__Odometry& message) {
    const uint32_t now = millis();
    if (now == last_ms_) return;
    const uint32_t elapsed_ms = now - last_ms_;
    int32_t left, right;
    encoders_->readCounts(left, right);
    const double dl = (left - last_left_) * kMetersPerTick;
    const double dr = (right - last_right_) * kMetersPerTick;
    const double distance = (dl + dr) * 0.5;
    const double heading_delta = (dr - dl) / kTrackWidth;
    x_ += distance * cos(theta_ + heading_delta * 0.5);
    y_ += distance * sin(theta_ + heading_delta * 0.5);
    theta_ += heading_delta;
    last_left_ = left;
    last_right_ = right;
    last_ms_ = now;
    message.header.stamp.sec = static_cast<int32_t>(now / 1000);
    message.header.stamp.nanosec = (now % 1000) * 1000000UL;
    message.pose.pose.position.x = x_;
    message.pose.pose.position.y = y_;
    message.pose.pose.orientation.z = sin(theta_ * 0.5);
    message.pose.pose.orientation.w = cos(theta_ * 0.5);
    message.twist.twist.linear.x = distance * 1000.0 / elapsed_ms;
    message.twist.twist.angular.z = heading_delta * 1000.0 / elapsed_ms;
  }

 private:
  // Calibration placeholders: verify encoder CPR, wheel diameter, and track.
  static constexpr double kMetersPerTick = (0.065 * PI) / 360.0;
  static constexpr double kTrackWidth = 0.23;
  WheelEncoders* encoders_ = nullptr;
  int32_t last_left_ = 0, last_right_ = 0;
  uint32_t last_ms_ = 0;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;
};

class DifferentialDrive {
 public:
  void begin() {
    pinMode(kRightIn1, OUTPUT);
    pinMode(kRightIn2, OUTPUT);
    pinMode(kLeftIn1, OUTPUT);
    pinMode(kLeftIn2, OUTPUT);
    ledcSetup(kRightPwmChannel, kPwmFrequency, kPwmResolution);
    ledcAttachPin(kRightPwmPin, kRightPwmChannel);
    ledcSetup(kLeftPwmChannel, kPwmFrequency, kPwmResolution);
    ledcAttachPin(kLeftPwmPin, kLeftPwmChannel);
    stop();
  }

  void command(float linear, float angular) {
    // The robot consistently drifts left under equal commands. Apply a small
    // open-loop correction to the left motor until the wheels are physically
    // realigned. This is a drive trim, not an odometry calibration.
    const int left = static_cast<int>(
        (linear * kLinearScale - angular * kAngularScale) * kLeftDriveTrim);
    const int right = static_cast<int>(linear * kLinearScale + angular * kAngularScale);
    setWheelDuty(left, right);
  }

  void stop() { setWheelDuty(0, 0); }

 private:
  static constexpr int kRightIn1 = 25;
  static constexpr int kRightIn2 = 33;
  static constexpr int kRightPwmPin = 32;
  static constexpr int kLeftIn1 = 27;
  static constexpr int kLeftIn2 = 14;
  static constexpr int kLeftPwmPin = 12;
  static constexpr int kRightPwmChannel = 0;
  static constexpr int kLeftPwmChannel = 1;
  static constexpr int kPwmFrequency = 5000;
  static constexpr int kPwmResolution = 8;
  static constexpr int kMaxDuty = (1 << kPwmResolution) - 1;
  // Conservative initial scale; tune only after a lifted-wheel test.
  static constexpr float kLinearScale = 120.0f;
  static constexpr float kAngularScale = 60.0f;
  // Start at +5% because the robot veers left. Increase/decrease in 0.02
  // steps after a short straight-line floor test.
  static constexpr float kLeftDriveTrim = 1.05f;

  void setWheelDuty(int left, int right) {
    left = constrain(left, -kMaxDuty, kMaxDuty);
    right = constrain(right, -kMaxDuty, kMaxDuty);
    // Left motor is mirror mounted, so identical physical directions need
    // opposite pin levels.
    digitalWrite(kRightIn1, right >= 0 ? HIGH : LOW);
    digitalWrite(kRightIn2, right >= 0 ? LOW : HIGH);
    digitalWrite(kLeftIn1, left >= 0 ? LOW : HIGH);
    digitalWrite(kLeftIn2, left >= 0 ? HIGH : LOW);
    ledcWrite(kRightPwmChannel, abs(right));
    ledcWrite(kLeftPwmChannel, abs(left));
  }
};

class NeckServo {
 public:
  void begin() {
    ledcSetup(kPwmChannel, kFrequency, kResolution);
    ledcAttachPin(kPin, kPwmChannel);
    current_angle_ = kSafeCenterAngle;
    target_angle_ = kSafeCenterAngle;
    writeAngle(current_angle_);
    last_step_ms_ = millis();
  }

  void setTargetAngle(int angle) {
    target_angle_ = constrain(angle, kMinimumAngle, kMaximumAngle);
  }

  void update() {
    if (!initialized_ || millis() - last_step_ms_ < kStepIntervalMs || current_angle_ == target_angle_) return;
    last_step_ms_ = millis();
    current_angle_ += current_angle_ < target_angle_ ? 1 : -1;
    writeAngle(current_angle_);
  }

 private:
  void writeAngle(int angle) {
    const int pulse_us = map(angle, 0, 180, kMinimumPulseUs, kMaximumPulseUs);
    const uint32_t duty = static_cast<uint32_t>(pulse_us) * ((1UL << kResolution) - 1) /
                          (1000000UL / kFrequency);
    ledcWrite(kPwmChannel, duty);
  }

  static constexpr int kPin = 2;
  static constexpr int kPwmChannel = 2;
  static constexpr int kFrequency = 50;
  static constexpr int kResolution = 16;
  static constexpr int kMinimumPulseUs = 500;
  static constexpr int kMaximumPulseUs = 2500;
  // The installed neck was only mechanically tested through this interval.
  static constexpr int kMinimumAngle = 80;
  static constexpr int kMaximumAngle = 150;
  static constexpr int kSafeCenterAngle = 120;
  static constexpr uint32_t kStepIntervalMs = 50;
  bool initialized_ = false;
  int current_angle_ = 90;
  int target_angle_ = kSafeCenterAngle;
  uint32_t last_step_ms_ = 0;
};

enum class RobotMode { kTeleop, kAutonomous };

class RobotController {
 public:
  void begin() {
    encoders_.begin();
    odometry_.begin(encoders_);
    drive_.begin();
    last_autonomous_command_ms_ = millis();
    last_teleop_command_ms_ = last_autonomous_command_ms_;
  }

  // Enable the servo only after micro-ROS has connected. Its startup current
  // surge must not brownout-reset the ESP32 before the Pi discovers the node.
  void enableNeck() { neck_.begin(); }

  void setMode(RobotMode mode) {
    mode_ = mode;
    drive_.stop();
  }

  void onAutonomousCommand(const geometry_msgs__msg__Twist& command) {
    last_autonomous_command_ms_ = millis();
    if (mode_ == RobotMode::kAutonomous) drive_.command(command.linear.x, command.angular.z);
  }

  void onTeleopCommand(const geometry_msgs__msg__Twist& command) {
    last_teleop_command_ms_ = millis();
    if (mode_ == RobotMode::kTeleop) drive_.command(command.linear.x, command.angular.z);
  }

  void onNeckCommand(int angle) { neck_.setTargetAngle(angle); }
  void update() { neck_.update(); }
  void updateOdometry(nav_msgs__msg__Odometry& message) { odometry_.update(message); }

  void enforceTimeout() {
    const uint32_t last_command = mode_ == RobotMode::kTeleop
                                      ? last_teleop_command_ms_
                                      : last_autonomous_command_ms_;
    if (millis() - last_command > kCommandTimeoutMs) drive_.stop();
  }

 private:
  DifferentialDrive drive_;
  WheelEncoders encoders_;
  Odometry odometry_;
  NeckServo neck_;
  RobotMode mode_ = RobotMode::kAutonomous;
  uint32_t last_autonomous_command_ms_ = 0;
  uint32_t last_teleop_command_ms_ = 0;
};

class MicroRosBridge {
 public:
  explicit MicroRosBridge(RobotController& controller) : controller_(controller) {}

  void begin() {
    instance_ = this;
    Serial.begin(kMicroRosBaud);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator_ = rcl_get_default_allocator();
    check(rclc_support_init(&support_, 0, nullptr, &allocator_));
    check(rclc_node_init_default(&node_, "mobile_robot", "", &support_));
    check(rclc_subscription_init_default(&autonomous_sub_, &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    check(rclc_subscription_init_default(&teleop_sub_, &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "teleop/cmd_vel"));
    check(rclc_subscription_init_default(&mode_sub_, &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "set_autonomous"));
    check(rclc_subscription_init_default(&neck_sub_, &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "teleop/neck_angle"));
    check(rclc_publisher_init_default(&odom_pub_, &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    odom_message_.header.frame_id.data = const_cast<char*>("odom");
    odom_message_.header.frame_id.size = 4;
    odom_message_.header.frame_id.capacity = 5;
    odom_message_.child_frame_id.data = const_cast<char*>("base_link");
    odom_message_.child_frame_id.size = 9;
    odom_message_.child_frame_id.capacity = 10;
    odom_message_.pose.pose.orientation.w = 1.0;
    check(rclc_executor_init(&executor_, &support_.context, 4, &allocator_));
    check(rclc_executor_add_subscription(&executor_, &autonomous_sub_, &autonomous_message_, &onAutonomous, ON_NEW_DATA));
    check(rclc_executor_add_subscription(&executor_, &teleop_sub_, &teleop_message_, &onTeleop, ON_NEW_DATA));
    check(rclc_executor_add_subscription(&executor_, &mode_sub_, &mode_message_, &onMode, ON_NEW_DATA));
    check(rclc_executor_add_subscription(&executor_, &neck_sub_, &neck_message_, &onNeck, ON_NEW_DATA));
  }

  void spin() {
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(2));
    if (millis() - last_odom_publish_ms_ >= kOdomPeriodMs) {
      last_odom_publish_ms_ = millis();
      controller_.updateOdometry(odom_message_);
      rcl_publish(&odom_pub_, &odom_message_, nullptr);
    }
  }

 private:
  static inline MicroRosBridge* instance_ = nullptr;
  RobotController& controller_;
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
  static constexpr uint32_t kOdomPeriodMs = 50;

  static void check(rcl_ret_t result) {
    if (result != RCL_RET_OK) while (true) delay(100);
  }
  static void onAutonomous(const void* message) {
    instance_->controller_.onAutonomousCommand(*static_cast<const geometry_msgs__msg__Twist*>(message));
  }
  static void onTeleop(const void* message) {
    instance_->controller_.onTeleopCommand(*static_cast<const geometry_msgs__msg__Twist*>(message));
  }
  static void onMode(const void* message) {
    instance_->controller_.setMode(static_cast<const std_msgs__msg__Bool*>(message)->data
                                      ? RobotMode::kAutonomous : RobotMode::kTeleop);
  }
  static void onNeck(const void* message) {
    instance_->controller_.onNeckCommand(static_cast<const std_msgs__msg__Int32*>(message)->data);
  }
};

RobotController robot;
MicroRosBridge ros(robot);

}  // namespace

void setup() {
  robot.begin();
  ros.begin();
  robot.enableNeck();
}

void loop() {
  ros.spin();
  robot.update();
  robot.enforceTimeout();
}
