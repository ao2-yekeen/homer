# Raspberry Pi runtime

There are two gamepad implementations in this repository:

- `gamepad_teleop.py` with `gamepad-teleop.service` is the older direct
  teleoperation path. Its controls are documented below.
- `robot_teleop/` with `robot-teleop.service` is the newer coordinator. It has
  a different mapping and topic contract; use its
  [separate guide](robot_teleop/README.md) when testing it.

Do not enable both services at once. The safe default for the direct path is
Autonomous. Press the labelled **A** button once to enter Teleop; hold the
labelled **B** button for 1.5 seconds to return to Autonomous. There is no
**Select** action.

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
camera. The first detector identifies the largest saturated **yellow** region
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

## Install the direct path on the Pi

Copy `gamepad_teleop.py` to the Pi user's home directory and
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

The ESP32 firmware subscribes to `/teleop/cmd_vel` and publishes `/odom`.
Verify startup with:

```bash
ros2 topic echo /odom --once
systemctl --user is-active micro-ros-agent.service robot-teleop.service gamepad-joy.service
```

The `rplidar.service` unit publishes `/scan` from the RPLIDAR A1 using the
stable `/dev/robot-lidar` device link and restarts automatically if the USB
device or driver temporarily disappears.
