# Unified gamepad control map

This is the intended operator layout for the whole robot. It is implemented by
the Pi coordinator in `pi/robot_teleop`; each subsystem publishes only its own
ROS command topic.

## Safety model

- The coordinator starts in **Autonomous/Stopped** mode. It will not move the
  base, arm, or neck until Teleop has been selected.
- A control only moves hardware while its matching dead-man button is held.
  Releasing it immediately stops base motion and arm/neck commands.
- Base and arm use separate dead-man buttons.
- Arm deltas are bounded; neck commands are limited to 80--150 degrees and
  slew-limited in the Pi and ESP32 firmware.

## Operator controls

| Control | Hold | Action | ROS output |
| --- | --- | --- | --- |
| **A** | none | Select Teleop mode | `/set_autonomous = false` |
| **B** | none | Select Autonomous/Stopped mode | `/set_autonomous = true` |
| Left stick up/down | **L1** | Drive forward/reverse | `/teleop/cmd_vel.linear.x` |
| Left stick left/right | **L1** | Turn left/right | `/teleop/cmd_vel.angular.z` |
| Left stick left/right | **R1** | Shoulder pan | SO-ARM joint 1 |
| Left stick up/down | **R1** | Shoulder lift | SO-ARM joint 2 |
| Right stick left/right | **R1** | Elbow flex/extend | SO-ARM joint 3 |
| Right stick up/down | **R1** | Wrist flex/extend | SO-ARM joint 4 |
| D-pad left/right | **R1** | Wrist roll | SO-ARM joint 5 |
| D-pad up/down | **R1** | Open/close gripper | SO-ARM joint 6 |
| **R2** (`Joy.axes[5]`) | **R1** | Neck up; axis moves from about `+1` toward `-1` while pressed | `/teleop/neck_angle` (`std_msgs/msg/Int32`, 80--150°) |
| **L2** (`Joy.axes[2]`) | **R1** | Neck down; axis moves from about `+1` toward `-1` while pressed | `/teleop/neck_angle` (`std_msgs/msg/Int32`, 80--150°) |

The signs above are the desired physical directions. If the receiver reports
different indices or reversed axes, update the controller profile rather than
changing the operator map.

## Verify before motion

With wheels lifted, arm supported, and an emergency stop available:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joy
```

Confirm A, B, L1, R1, L2, R2, both sticks, and all four D-pad
directions one at a time. Then verify `/teleop/cmd_vel`,
`/teleop/neck_angle`, and `/soarm/command_delta_ticks` before testing motion.

For the active `robot_teleop` service, the exact neck command is:

- Hold **R1** + press **R2**: neck angle increases toward 150°.
- Hold **R1** + press **L2**: neck angle decreases toward 80°.
- Release **R1**: no further neck commands are sent.

The axis numbers and polarity are the current Aurora/Xbox profile, so confirm
them with `ros2 topic echo /joy` before enabling motion on a different
receiver.
