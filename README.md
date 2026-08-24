# ESP32 mobile robot

This repository contains the ESP32 micro-ROS firmware and the Raspberry Pi
teleoperation coordinator.

## Gamepad guide

The complete operator control map, dead-man rules, ROS outputs, and pre-motion
verification procedure are in [`docs/gamepad-control-map.md`](docs/gamepad-control-map.md).
Read that guide before enabling teleoperation. In short: **A** selects Teleop,
**B** selects Autonomous/Stopped, hold **L1** for base drive, and hold **R1**
for arm and neck controls.

## Runtime graph

The Pi starts these user services automatically (user lingering is required):

- `gamepad-joy.service` publishes `/joy`.
- `robot-teleop.service` publishes `/teleop/cmd_vel`, `/teleop/neck_angle`,
  `/soarm/command_delta_ticks`, and `/set_autonomous`.
- `micro-ros-agent.service` connects the ESP32 on `/dev/robot-esp32` at 921600
  baud.

The ESP32 node is `/mobile_robot`. It subscribes to `/cmd_vel`,
`/teleop/cmd_vel`, `/set_autonomous`, and `/teleop/neck_angle`, and publishes
encoder-based `nav_msgs/msg/Odometry` on `/odom` with frames `odom` and
`base_link`.

## Firmware layout

The firmware is split by responsibility: hardware drivers are in
`wheel_encoders.*`, `differential_drive.*`, and `neck_servo.*`; motion
estimation is in `odometry.*`; robot modes and command timeout safety are in
`robot_controller.*`; and ROS topics are isolated in `micro_ros_node.*`.
Physical pins and calibration values are collected in `robot_config.h`.

## Safety

Test with the wheels lifted, the arm supported, and an emergency power
disconnect available. The neck starts at 120 degrees and is slew-limited. Keep
the servo on an adequately rated supply with a common ground; rate limiting
does not fix an undersized power rail. Odometry wheel CPR, diameter, and track
width are provisional until calibrated.

## Checks on the Pi

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
systemctl --user is-active micro-ros-agent.service robot-teleop.service gamepad-joy.service
```

The coordinator source and unit file are in `pi/robot_teleop/`. Run its tests
without ROS with:

```bash
PYTHONPATH=pi/robot_teleop/src python3 -m unittest discover -s pi/robot_teleop/tests -v
```
