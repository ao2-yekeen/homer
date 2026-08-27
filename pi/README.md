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

## SO-ARM named poses (Day 1)

`soarm_ros_bridge.py` provides manual jog commands on
`/soarm/command_delta_ticks` and guarded named-pose commands on
`/soarm/command_named_pose`. Named poses are stored in
`soarm_named_poses.json`.

At startup the bridge reads the travel limits directly from each servo EEPROM;
it does not apply separate guessed collision limits. Last verified on
2026-08-25:

| Joint | EEPROM tick range |
| --- | --- |
| shoulder_pan | 730–3444 |
| shoulder_lift | 1450–2446 |
| elbow_flex | 890–2506 |
| wrist_flex | 2315–3233 |
| wrist_roll | 0–4095 |
| gripper | 2034–3504 |

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

Named poses command all six recorded joint states, including the gripper. The
gripper value captured with each pose is therefore part of that pose's physical
configuration.

Stop an active named-pose move immediately (the bridge holds the live joint
positions):

```bash
ros2 topic pub --once /soarm/command_named_pose std_msgs/msg/String "{data: stop}"
```

## Manual trajectory playback

For a torque-off manual recording that contains `/soarm/state_ticks`, replay
the recorded absolute poses only after first reaching the same APPROACH pose:

```bash
source /opt/ros/jazzy/setup.bash
python3 ~/soarm_trajectory_playback.py ~/soarm_trajectories/manual_grasp_lift_<timestamp>
```

The player publishes `/soarm/command_pose_ticks`; the bridge validates every
pose against the live EEPROM limits and follows it through the same slow
incremental motion path as named poses. `Ctrl-C` stops playback and holds the
live pose. This is a hardware-validation step, not evidence that a recorded
trajectory is safe in a changed workspace.

By default the player removes a leading held-pose section smaller than 10
ticks, so manual preparation time before the first real movement is not
replayed. Use `--leading-idle-threshold 0` to preserve that delay.

`REQUIRES_HARDWARE_TEST`: capture, transition validation, collision checks,
and stop verification remain physical tasks; do not treat software checks as
evidence that a pose is safe.
