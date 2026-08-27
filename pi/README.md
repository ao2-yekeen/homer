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

## Gripper contact feedback

The SO-ARM bridge passively reads servo ID 6 feedback at 10 Hz and publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/soarm/gripper/telemetry` | `std_msgs/msg/String` | JSON containing position, commanded target, position error, signed raw load, raw current, moving flag, calibrated object gap, and contact state |
| `/soarm/gripper/contact` | `std_msgs/msg/String` | `DISABLED`, `UNKNOWN`, `CLOSING`, or latched `GRIPPED` |

`GRIPPED` means the gripper encountered evidence consistent with an object. It
does **not** by itself prove grasp success; post-lift vision or another
independent signal must still confirm that the object moved with the hand.

Detection is disabled by default in `gripper_contact.json`. Do not enable the
placeholder configuration. It must first be calibrated on the installed
gripper servo, with the arm supported and stationary over a raised platform,
an immediate power disconnect available, and no floor-level target:

1. Keep `enabled` false and observe telemetry while stationary.
2. Under direct supervision, record at least five slow empty closes and opens.
   Determine the actual tick direction for closing and the repeatable
   `empty_closed_position_ticks` value; do not infer direction from the
   gamepad label.
3. Record several slow closes on representative rigid objects that cannot be
   crushed. Compare raw load, raw current, target error, and the difference
   from the empty-closed position.
4. Choose conservative thresholds only if the empty and object-contact samples
   have clear separation. Keep `minimum_object_gap_ticks` larger than endpoint
   repeatability/noise so the empty mechanical stop cannot look like an object.
5. Set `confirmation_samples` to require sustained evidence, enable the config,
   restart the bridge, and repeat supervised empty/object tests.

If the observations do not separate cleanly, leave detection disabled and add
a suitable fingertip force/contact sensor instead. Raw current is intentionally
reported in device-native units because its physical scale depends on the exact
servo model and firmware.

Read the passive stream with:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /soarm/gripper/telemetry
ros2 topic echo /soarm/gripper/contact
```
