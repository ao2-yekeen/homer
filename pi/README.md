# Gamepad teleoperation

The service starts the ROS 2 joystick driver and `gamepad_teleop.py`, which
uses `/joy` to publish every manual control path:

| Gamepad control (default PS3 mapping) | Robot action |
| --- | --- |
| Hold **L1** + left stick up/down | Drive forward/reverse |
| Hold **L1** + left stick left/right | Turn |
| D-pad up/down | Move the neck within the tested 80--150 degree range |
| **Select** | Enter Teleop mode |
| **L3** | Enter Autonomous mode |

The firmware accepts `/teleop/cmd_vel` only in Teleop mode. Autonomous
movement remains on `/cmd_vel`, so the gamepad cannot command the base in
Autonomous mode. Releasing L1 sends zero drive commands. The ESP32 also stops
the base if its active command stream is stale for one second.

The button and axis values are parameters in `gamepad_teleop.py`. Before
driving, confirm the controller's indices without moving the robot:

```bash
ros2 topic echo /joy
```

Override an index in the systemd service by appending, for example,
`--ros-args -p drive_enable_button:=<index>` to its `ExecStart` command.

## Install on the Pi

Copy `gamepad_teleop.py` to the Pi user's home directory and
`gamepad-teleop.service` to `~/.config/systemd/user/`, then run:

```bash
systemctl --user daemon-reload
systemctl --user disable --now mode-toggle.service
systemctl --user enable --now gamepad-teleop.service
```

After flashing the matching ESP32 firmware, verify the topics before placing
the robot on the floor:

```bash
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /teleop/neck_angle
```
