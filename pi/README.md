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

The `rplidar.service` unit publishes `/scan` from the RPLIDAR A1 using the
stable `/dev/robot-lidar` device link and restarts automatically if the USB
device or driver temporarily disappears.

## SO-ARM named poses (Day 1)

`soarm_ros_bridge.py` provides manual jog commands on
`/soarm/command_delta_ticks` and guarded named-pose commands on
`/soarm/command_named_pose`. Named poses are stored in
`soarm_named_poses.json`; they are intentionally disabled by default and no
sample coordinates are supplied.

Only with the arm supported, an accessible emergency power disconnect, and the
specific physical pose already checked for mast, base, camera, platform, and
cable clearance, capture it without moving the arm:

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/soarm_pose_capture.py home --confirm-safe
```

Repeat for `approach`, `grasp`, and `lift`. Keep
`motion_enabled: false` while measuring and checking the transitions. After
all required paths have been physically tested at the configured conservative
rate, set it to `true` in `~/soarm_named_poses.json` and restart the bridge.

Then call one stored pose by name:

```bash
ros2 topic pub --once /soarm/command_named_pose std_msgs/msg/String "{data: home}"
```

Stop an active named-pose move immediately (the bridge holds the live joint
positions):

```bash
ros2 topic pub --once /soarm/command_named_pose std_msgs/msg/String "{data: stop}"
```

`REQUIRES_HARDWARE_TEST`: capture, transition validation, collision checks,
and stop verification remain physical tasks; do not treat software checks as
evidence that a pose is safe.
