# Raspberry Pi runtime

## Automatic startup

The Pi user has systemd lingering enabled, so enabled user services start
without a login. Apply the tracked boot units with
`./pi/install-boot-services.sh` from the repository root. It enables the
ESP32 serial bridge, LiDAR, RGB camera, gamepad, teleop coordinator, ToF
publisher, device discovery, and an SO-ARM device watcher. It disables the
old micro-ROS Agent and direct gamepad teleop service so they cannot compete
for the same serial port or command topics.

The SO-ARM watcher starts its bridge only when `/dev/robot-soarm` exists.
LiDAR startup checks the previously identified port and recreates its runtime
link when the sensor responds. An unplugged arm or late LiDAR does not prevent
the ESP32 bridge from starting. Check boot state with:

```bash
systemctl --user is-enabled esp32-serial-bridge.service rplidar.service \
  rgb-object-detection.service soarm-bridge.path gamepad-joy.service \
  robot-teleop.service tof-depth-publisher.service
systemctl --user is-active esp32-serial-bridge.service rplidar.service \
  rgb-object-detection.service soarm-bridge.path
```

There are two gamepad implementations in this repository. The active boot
configuration uses the newer `robot_teleop/` coordinator and
`robot-teleop.service`; do not enable both services at once.

The active coordinator starts in Autonomous mode:

| Gamepad control | Robot action |
| --- | --- |
| Press **A** | Select Teleop mode |
| Press **B** | Select Autonomous mode |
| Hold **L1** + left stick | Drive the base |
| Hold **R1** + left stick | Shoulder pan/lift |
| Hold **R1** + right stick | Elbow/wrist flex |
| Hold **R1** + D-pad | Wrist roll/gripper |
| Hold **R1** + **R2** | Neck up; publishes `/teleop/neck_angle` (`std_msgs/msg/Int32`) |
| Hold **R1** + **L2** | Neck down; publishes `/teleop/neck_angle` (`std_msgs/msg/Int32`) |

L1 and R1 are separate dead-man switches. Base commands are published on
`/teleop/cmd_vel`; arm deltas use `/soarm/command_delta_ticks`; neck commands
use `/teleop/neck_angle`; and mode selection uses `/set_autonomous`. The
receiver indices and directions for R1, A, B, the right stick, D-pad, L2, and
R2 still require physical verification on the target controller. In the
current Aurora/Xbox profile, the expected trigger axes are **R2** `axis 5`
and **L2** `axis 2`; both normally rest near `+1.0` and move towards `-1.0`
when pressed. See
the [active coordinator guide](robot_teleop/README.md) and the
[full control map](../docs/gamepad-control-map.md).

`gamepad_teleop.py` with `gamepad-teleop.service` is the older direct
teleoperation path and is retained for reference/diagnostics only. Its
different face-button mapping must not be used alongside the active
coordinator.

### Legacy direct-path mapping

The following controls apply only to `gamepad_teleop.py`, not to the active
`robot_teleop` coordinator:

| Gamepad control (Xbox-labelled Aurora receiver) | Robot action |
| --- | --- |
| Press **A** while Autonomous | Enter Teleop |
| Hold **Y** while Teleop | Drive forward |
| Hold **A** while Teleop | Drive backward |
| Hold **X** while Teleop | Turn left |
| Briefly hold **B** while Teleop | Turn right |
| Hold **B** for 1.5 seconds while Teleop | Return to Autonomous |
| Hold **LT** + left stick left/right | Shoulder pan |
| Hold **LT** + left stick up/down | Shoulder lift |
| Hold **LT** + right stick left/right | Elbow flex |
| Hold **LT** + right stick up/down | Wrist flex |
| Hold **LT** + D-pad left/right | Wrist roll |
| Hold **LT** + D-pad up/down | Gripper |
| Hold **LT** + **LB** / **RB** | Lower / raise neck, limited to 80--150 degrees |

LT is the arm dead-man: releasing it immediately stops arm and neck commands.

## RGB camera and first-object detection

`rgb_object_detection.py` publishes the Logitech C930e stream as
`/rgb_camera/image_raw` and publishes a compact JSON result on
`/rgb_camera/detection`. It is deliberately independent of the unavailable ToF
camera. The first detector identifies the largest low-value **black** region
on the raised platform, reporting `detected`, `confidence`, `bbox_xywh`, and
`center_uv`; it is not a general object detector.

Install and start it on the Pi after this repository has been deployed at
`~/mobile-robot`:

```bash
mkdir -p ~/.config/systemd/user
cp pi/rgb-object-detection.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now rgb-object-detection.service
source /opt/ros/jazzy/setup.bash
ros2 topic echo /rgb_camera/detection
```

The camera identity is the Logitech C930e's stable `/dev/v4l/by-id/...index0`
path; do not substitute an arbitrary `/dev/videoN`. Confirm stream and target
colour under the actual platform lighting before relying on its output.

To test one camera tilt position, with the arm clear and emergency stop
available, issue an explicit bounded command:

```bash
source /opt/ros/jazzy/setup.bash
python3 pi/camera_tilt.py --angle 100
```

`REQUIRES_HARDWARE_TEST`: record the tested tilt angle, platform visibility,
and any camera/arm/cable clearance limit before treating a named view as safe.
The ESP32 also stops the base if its active command stream is stale for one
second. The gamepad uses the standard Xbox-labelled layout; check each joint's
direction in a clear workspace before normal use because a motor's installed
direction is mechanical.

Before driving, confirm that `/joy` is present without moving the robot:

```bash
ros2 topic echo /joy
```

## Install the legacy direct path on the Pi

Only use this section when deliberately testing the legacy path. Copy
`gamepad_teleop.py` to the Pi user's home directory and
`gamepad-teleop.service` to `~/.config/systemd/user/`, then run:

```bash
systemctl --user daemon-reload
systemctl --user enable --now gamepad-teleop.service
```

For an arm-only test, leave the base off the floor or remap the drive topic to
an unused name. Verify the topics before moving hardware:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joy
ros2 topic echo /teleop/cmd_vel
ros2 topic echo /teleop/neck_angle
ros2 topic echo /soarm/command_delta_ticks
```

The ESP32 serial bridge subscribes to `/teleop/cmd_vel` and publishes `/odom`.
Verify startup with:

```bash
ros2 topic echo /odom --once
systemctl --user is-active esp32-serial-bridge.service robot-teleop.service gamepad-joy.service
```

The `rplidar.service` unit publishes `/scan` from the RPLIDAR A1 using the
stable `/dev/robot-lidar` device link and restarts automatically if the USB
device or driver temporarily disappears.
