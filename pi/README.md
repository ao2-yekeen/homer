# Raspberry Pi runtime

For the service and ROS data-flow diagrams, see the
[system architecture README](../docs/architecture/README.md). This page covers
installation, hardware paths, and runtime checks.

## Automatic startup

The Pi user has systemd lingering enabled, so enabled user services start
without a login. Apply the tracked boot units with
`./pi/install-boot-services.sh` from the repository root. It enables the
ESP32 serial bridge, LiDAR, scan watchdog, RGB camera (on a fresh install),
gamepad, teleop coordinator, ToF publisher, device discovery, and an SO-ARM
device watcher. It preserves an RGB service that was already disabled. It
disables the old micro-ROS Agent and direct gamepad teleop service so they
cannot compete for the same serial port or command topics.

The SO-ARM watcher starts its bridge only when `/dev/robot-soarm` exists.
Check boot state with:

```bash
systemctl --user is-enabled esp32-serial-bridge.service rplidar.service \
  lidar-scan-watchdog.service \
  rgb-object-detection.service soarm-bridge.path gamepad-joy.service \
  robot-teleop.service tof-depth-publisher.service
systemctl --user is-active esp32-serial-bridge.service rplidar.service \
  lidar-scan-watchdog.service \
  rgb-object-detection.service soarm-bridge.path
```

## Device identities and USB safety

[`device_paths.py`](device_paths.py) defines this Pi's three fixed device
locations:

| Device | Constant | Path |
| --- | --- | --- |
| RPLIDAR A1 | `LIDAR_PORT` | `/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-port0` |
| ESP32 | `ESP32_PORT` | `/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2.3:1.0-port0` |
| Logitech C930e | `RGB_CAMERA` | `/dev/v4l/by-id/usb-046d_Logitech_Webcam_C930e-video-index0` |

Both CP2102 adapters report the same serial number (`0001`), so `by-id` is
ambiguous and `ttyUSB` numbering is unstable. Keep LiDAR on physical USB path
`1.1` and ESP32 on `1.2.3`. The old ESP32 bridge searched every serial port
and opened the LiDAR, stopping its motor. The bridge now uses **only** its
fixed path. `device_discovery.py` and `wait_for_lidar.py` check paths without
opening serial ports; they create runtime links in
`/run/user/<uid>/mobile-robot-devices/`. The old `device-routes.json` no longer
selects serial ports.

Check the mapping on the Pi without opening either port:

```bash
ls -l /dev/serial/by-path/
readlink -f /run/user/$(id -u)/mobile-robot-devices/lidar
readlink -f /run/user/$(id -u)/mobile-robot-devices/esp32
```

If cabling changes, identify the new sockets one device at a time, update
`device_paths.py`, then rerun `./pi/install-boot-services.sh` when safe to
restart services. Update `rgb-object-detection.service` too if its camera path
changes. Other paths live in their units: SO-ARM at `/dev/robot-soarm`, gamepad
at `/dev/input/js0`; the CSI ToF service is outside this repository.

The read-only `lidar-scan-watchdog.service` starts at boot and monitors actual
`/scan` messages. After a 60-second startup grace period, 15 seconds without
a scan triggers an evidence snapshot; while down, it repeats at most once per
minute. It records robot services, ROS graph, USB/serial ownership, Pi power
and temperature, processes, and recent logs. It never opens the LiDAR port or
restarts the motor. Snapshots are kept under
`~/.local/state/mobile-robot/lidar-watchdog/` (newest 30 files retained). Check
status with `systemctl --user status lidar-scan-watchdog.service` and recent
events with `journalctl --user -u lidar-scan-watchdog.service -b`.

There are two gamepad implementations in this repository. The active boot
configuration uses the newer `robot_teleop/` coordinator and
`robot-teleop.service`; do not enable both services at once. The canonical
mapping, dead-man rules, and physical verification steps are in the
[gamepad control map](../docs/gamepad-control-map.md). Installation details
for the coordinator are in [its guide](robot_teleop/README.md).

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
runtime `mobile-robot-devices/lidar` link, which points to its fixed USB
by-path location. The service restarts automatically if the driver exits.
