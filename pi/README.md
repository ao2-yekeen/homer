# Raspberry Pi runtime

The active gamepad path is `robot_teleop`, not the retired
`gamepad_teleop.py` service. Install `pi/robot_teleop/` under
`~/mobile-robot/pi/robot_teleop/`, install its systemd unit as
`~/.config/systemd/user/robot-teleop.service`, then run:

```bash
systemctl --user daemon-reload
systemctl --user enable --now robot-teleop.service
```

The joystick service must publish `/joy`. Before enabling motion, keep the
wheels lifted and verify:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joy
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /teleop/neck_angle
ros2 topic echo /soarm/command_delta_ticks
```

The ESP32 firmware subscribes to `/teleop/cmd_vel` and publishes `/odom`.
Verify startup with:

```bash
ros2 topic echo /odom --once
systemctl --user is-active micro-ros-agent.service robot-teleop.service gamepad-joy.service
```
