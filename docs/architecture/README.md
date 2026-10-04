# Robot system architecture

This page shows the robot's runtime components. For installation commands and fixed USB
paths, use the [Pi runtime guide](../../pi/README.md); for building and
flashing, use the [main README](../../README.md#build-esp32-firmware).

Current priority: [live SLAM and RViz goal navigation](live-slam-rviz-goal-navigation-plan.md),
validated in simulation before hardware deployment. The
[peer-reviewed assumptions audit](slam-navigation-peer-reviewed-assumptions-2026-10-04.md)
records which claims are supported and which still require experiments.

For longer-term staged development, see the [unknown-environment object-search and pickup
roadmap](autonomous-mapping-object-navigation-plan.md) and the current
[Section 1 MVP checklist](slam-section-1-validation-plan.md).

Current SLAM work requires simulation and offline replay before deployment.
Remote automatic movement tests are stopped after the 50 cm / 9 cm discrepancy.
See the [camera–lidar literature review](slam-camera-fusion-literature-review-2026-10-04.md),
[simulation-first validation plan](slam-simulation-validation-plan.md), and
[physical test results](slam-accuracy-2026-10-04.md).

## Data and control flow

```mermaid
flowchart LR
  gamepad[Gamepad] --> joy[gamepad-joy.service]
  joy -->|/joy| teleop[robot-teleop.service]
  teleop -->|/teleop/cmd_vel, /set_autonomous, /teleop/neck_angle| bridge[esp32-serial-bridge.service]
  autonomy[Autonomous ROS node, when present] -->|/cmd_vel| bridge
  operator[Terminal operator] -->|/mode_override| bridge
  teleop -->|/soarm/command_delta_ticks| arm[soarm-bridge.service]
  arm <--> soarm[SO-ARM 101 USB controller]
  bridge <-->|HOM1 serial on fixed ESP32 USB path| esp32[ESP32 firmware]
  esp32 --> base[Drive motors, neck; encoders disabled]
  bridge -->|/drive/status, /drive/link| ros[ROS 2 consumers]
  driver -->|/scan| rf2o[RF2O LiDAR odometry]
  rf2o -->|/odom and odom to base_link| slam[SLAM Toolbox]
  driver -->|/scan| slam
  slam -->|/map and map to odom| viewer[Laptop RViz over hotspot]

  lidar[RPLIDAR A1] -->|fixed LiDAR USB path| driver[rplidar.service]
  driver -->|/scan| ros
  driver -->|/scan| watchdog[lidar-scan-watchdog.service]
  watchdog -->|baseline, outage, recovery| evidence[Local evidence files and journal]

  rgb[RGB webcam] --> camera[rgb-object-detection.service]
  camera -->|/rgb_camera/image_raw, /rgb_camera/detection| ros
  tof[CSI ToF camera] --> depth[tof-depth-publisher.service]
  depth -->|depth topics| ros
```

The ESP32 bridge routes either `/teleop/cmd_vel` or `/cmd_vel` according to
the selected mode; `/mode_override` takes priority. The ESP32 has a one-second
motor-command timeout. The SO-ARM bridge starts only when `/dev/robot-soarm`
appears. The ToF publisher's implementation is outside this repository.
The RGB service can be disabled for diagnosis and is not required for LiDAR
scans. The ToF path is optional when its separate service is installed.

## Boot and fault observation

```mermaid
flowchart TD
  boot[Pi boot and user systemd] --> discovery[device-core-discovery.service]
  paths[pi/device_paths.py: fixed physical USB paths] --> discovery
  discovery -->|runtime ESP32 link| bridge[esp32-serial-bridge.service]
  discovery -->|runtime LiDAR link| lidar[rplidar.service]
  boot --> joy[gamepad-joy.service]
  joy --> teleop[robot-teleop.service]
  boot --> armwatch[soarm-bridge.path]
  armwatch -->|/dev/robot-soarm appears| arm[soarm-bridge.service]
  boot --> rgb[rgb-object-detection.service if enabled]
  boot --> tof[tof-depth-publisher.service if installed and enabled]
  lidar -->|/scan messages| watchdog[lidar-scan-watchdog.service]
  watchdog -->|scan stale: capture diagnostics| snapshot[~/.local/state/mobile-robot/lidar-watchdog/]
```

Discovery checks paths without opening serial devices. The ESP32 bridge opens
only its fixed port; the LiDAR driver alone opens the LiDAR port. The watchdog
starts with the Pi, allows 60 seconds for startup, then reports 15 seconds
without `/scan`. It takes a healthy baseline and, during an outage, saves at
most one snapshot per minute (latest 30 retained). Each snapshot includes
service states, ROS graph, serial ownership, USB state, Pi temperature and
throttling, relevant processes, and recent logs. Recovery is logged, but the
watchdog does **not** restart anything.

Check the live system with `systemctl --user status lidar-scan-watchdog.service`
and `journalctl --user -u lidar-scan-watchdog.service -b`. The [Pi runtime
guide](../../pi/README.md#device-identities-and-usb-safety) lists the exact
device paths and safe verification commands.

For encoder-free mapping, the lightweight viewer, and explicit hotspot/Tailscale
profiles, see [the SLAM guide](../../pi/homer_slam/README.md). SLAM startup is
manual until its physical checks pass.
