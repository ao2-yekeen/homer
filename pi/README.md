# Gamepad teleoperation

The service starts the ROS 2 joystick driver and `gamepad_teleop.py`. There is
no controller mode and no **Select** action: every input has one direct robot
action.

| Gamepad control (Xbox-labelled Aurora receiver) | Robot action |
| --- | --- |
| Hold **Y** | Drive forward |
| Hold **A** | Drive backward |
| Hold **X** | Turn left |
| Hold **B** | Turn right |
| Hold **LT** + left stick left/right | Shoulder pan |
| Hold **LT** + left stick up/down | Shoulder lift |
| Hold **LT** + right stick left/right | Elbow flex |
| Hold **LT** + right stick up/down | Wrist flex |
| Hold **LT** + D-pad left/right | Wrist roll |
| Hold **LT** + D-pad up/down | Gripper |
| Hold **LT** + **LB** / **RB** | Lower / raise neck, limited to 80--150 degrees |

LT is the arm dead-man: releasing it immediately stops arm and neck commands.
The ESP32 also stops the base if its active command stream is stale for one
second. The gamepad uses the standard Xbox-labelled layout; check each joint's
direction in a clear workspace before normal use because a motor's installed
direction is mechanical.

Before driving, confirm that `/joy` is present without moving the robot:

```bash
ros2 topic echo /joy
```

## Install on the Pi

Copy `gamepad_teleop.py` to the Pi user's home directory and
`gamepad-teleop.service` to `~/.config/systemd/user/`, then run:

```bash
systemctl --user daemon-reload
systemctl --user disable --now mode-toggle.service
systemctl --user enable --now gamepad-teleop.service
```

For an arm-only test, leave the base off the floor or remap the drive topic to
an unused name. Verify the topics before moving hardware:

```bash
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /teleop/neck_angle
ros2 topic echo /soarm/command_delta_ticks
```
