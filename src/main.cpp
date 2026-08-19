#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

// Motor A = right side, Motor B = left side (mirror-mounted, so its
// direction pins are flipped relative to Motor A for the same physical
// direction of travel).
const int AIN1 = 25;
const int AIN2 = 33;
const int PWMA = 32;

const int BIN1 = 27;
const int BIN2 = 14;
const int PWMB = 12;

const int PWM_CH_A = 0;
const int PWM_CH_B = 1;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int PWM_MAX_DUTY = (1 << PWM_RES) - 1;

// Servo on a separate LEDC channel/timer (50Hz, 16-bit) so it doesn't
// collide with the motor PWM channels above.
const int SERVO_PIN = 2;
const int PWM_CH_SERVO = 2;
const int SERVO_FREQ = 50;
const int SERVO_RES = 16;

// Most hobby servos respond to a wider pulse range than the "standard"
// 1000-2000us, giving closer to the full mechanical 0-180 degree sweep.
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const unsigned long MICRO_ROS_BAUD = 921600; // must match `micro_ros_agent serial -b <baud>` on the Pi

// AUTONOMOUS = obey /cmd_vel. TELEOP = obey /teleop/cmd_vel. Keeping the
// sources separate prevents a gamepad command driving in autonomous mode.
enum RobotMode { MODE_TELEOP, MODE_AUTONOMOUS };
RobotMode mode = MODE_TELEOP; // safe default: robot stays put until the Pi explicitly opts in

unsigned long lastCmdVelMs = 0;
unsigned long lastTeleopCmdVelMs = 0;
const unsigned long CMDVEL_TIMEOUT_MS = 1000; // stop if active Pi stream goes quiet

// cmd_vel -> wheel duty conversion. Untuned placeholders: linear.x in m/s,
// angular.z in rad/s, scaled directly to PWM duty. Retune SPEED_SCALE/
// TURN_SCALE once you know real wheel radius/track width.
const float SPEED_SCALE = 120.0f; // duty per m/s
const float TURN_SCALE = 60.0f;   // duty per rad/s

// ---- micro-ROS ----
rcl_subscription_t cmdVelSub;
geometry_msgs__msg__Twist cmdVelMsg;
rcl_subscription_t teleopCmdVelSub;
geometry_msgs__msg__Twist teleopCmdVelMsg;
rcl_subscription_t setAutonomousSub;
std_msgs__msg__Bool setAutonomousMsg;
rcl_subscription_t neckAngleSub;
std_msgs__msg__Int32 neckAngleMsg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Serial (USB) is dedicated to the micro-ROS transport, so debug output goes
// out UART2 (pins 16/17) instead. TODO: remove once micro-ROS bring-up is confirmed working.
void errorLoop(const char *stage, rcl_ret_t rc) {
  Serial2.printf("RCCHECK failed at: %s (rc=%ld)\n", stage, (long)rc);
  while (true) {
    delay(100);
  }
}
#define RCCHECK(stage, fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { errorLoop(stage, rc); } }

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  int duty = (int)((float)pulseUs / (1000000.0f / SERVO_FREQ) * ((1 << SERVO_RES) - 1));
  ledcWrite(PWM_CH_SERVO, duty);
}

// duty sign is "forward" in the direction of travel; the A/B pin-level
// mirroring that makes that true is handled here, once.
void driveMotorA(int duty) {
  digitalWrite(AIN1, duty >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, duty >= 0 ? LOW : HIGH);
  ledcWrite(PWM_CH_A, abs(duty));
}

void driveMotorB(int duty) {
  digitalWrite(BIN1, duty >= 0 ? LOW : HIGH);
  digitalWrite(BIN2, duty >= 0 ? HIGH : LOW);
  ledcWrite(PWM_CH_B, abs(duty));
}

void setMotors(int leftDuty, int rightDuty) {
  leftDuty = constrain(leftDuty, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  rightDuty = constrain(rightDuty, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  driveMotorB(leftDuty);
  driveMotorA(rightDuty);
}

void stopMotors() {
  setMotors(0, 0);
}

void setMode(RobotMode newMode) {
  mode = newMode;
  stopMotors();
}

void applyTwist(const geometry_msgs__msg__Twist *msg) {
  float linear = (float)msg->linear.x;
  float angular = (float)msg->angular.z;
  int leftDuty = (int)(linear * SPEED_SCALE - angular * TURN_SCALE);
  int rightDuty = (int)(linear * SPEED_SCALE + angular * TURN_SCALE);
  setMotors(leftDuty, rightDuty);
}

void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  lastCmdVelMs = millis();
  if (mode == MODE_AUTONOMOUS) applyTwist(msg);
}

void teleopCmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  lastTeleopCmdVelMs = millis();
  if (mode == MODE_TELEOP) applyTwist(msg);
}

void setAutonomousCallback(const void *msgin) {
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
  setMode(msg->data ? MODE_AUTONOMOUS : MODE_TELEOP);
}

void neckAngleCallback(const void *msgin) {
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  // This is the conservative clearance range established by servo_sweep.
  setServoAngle(constrain(msg->data, 80, 150));
}

void setupMicroRos() {
  Serial.begin(MICRO_ROS_BAUD); // set_microros_serial_transports() does not call begin() itself
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK("support_init", rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK("node_init", rclc_node_init_default(&node, "esp32_robot_node", "", &support));
  RCCHECK("sub_init", rclc_subscription_init_default(
      &cmdVelSub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));
  RCCHECK("teleop_sub_init", rclc_subscription_init_default(
      &teleopCmdVelSub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "teleop/cmd_vel"));
  RCCHECK("mode_sub_init", rclc_subscription_init_default(
      &setAutonomousSub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "set_autonomous"));
  RCCHECK("neck_sub_init", rclc_subscription_init_default(
      &neckAngleSub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "teleop/neck_angle"));
  RCCHECK("executor_init", rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK("executor_add_sub", rclc_executor_add_subscription(&executor, &cmdVelSub, &cmdVelMsg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK("executor_add_teleop_sub", rclc_executor_add_subscription(&executor, &teleopCmdVelSub, &teleopCmdVelMsg, &teleopCmdVelCallback, ON_NEW_DATA));
  RCCHECK("executor_add_mode_sub", rclc_executor_add_subscription(&executor, &setAutonomousSub, &setAutonomousMsg, &setAutonomousCallback, ON_NEW_DATA));
  RCCHECK("executor_add_neck_sub", rclc_executor_add_subscription(&executor, &neckAngleSub, &neckAngleMsg, &neckAngleCallback, ON_NEW_DATA));
}

void setup() {
  Serial2.begin(115200); // debug only; errorLoop() reports here if wired to a debug adapter

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, PWM_CH_B);

  ledcSetup(PWM_CH_SERVO, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, PWM_CH_SERVO);
  setServoAngle(90); // safe initial neck position; teleop/neck_angle controls it

  stopMotors();

  setupMicroRos(); // takes over Serial (USB) for the Pi link; no more Serial.print after this

  lastCmdVelMs = millis();
  lastTeleopCmdVelMs = lastCmdVelMs;
}

void loop() {
  // Not RCCHECK'd: spin_some legitimately returns non-OK (e.g. timeout) when
  // there's simply nothing to process, which isn't a fatal condition.
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));

  // Dead-man switch: either active command source timing out stops the base.
  unsigned long lastActiveCmdMs = mode == MODE_AUTONOMOUS ? lastCmdVelMs : lastTeleopCmdVelMs;
  if (millis() - lastActiveCmdMs > CMDVEL_TIMEOUT_MS) {
    stopMotors();
  }
}
