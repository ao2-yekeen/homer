# Homer mobile manipulator

Homer is an experimental mobile manipulator built around an ESP32, a Raspberry
Pi, ROS 2 Jazzy, differential drive, an SO-ARM 101, and an RPLIDAR A1. This
repository contains the firmware, Pi-side ROS integration, operator controls,
and a lightweight simulation in one place.

> **Hardware safety:** do not enable motion from a fresh checkout. Test with
> the drive wheels lifted, the arm supported, and an accessible emergency power
> disconnect. Confirm controller mappings, USB device identities, motor wiring,
> and power capacity on the specific robot first.

## What lives here

| Area | Purpose | Starting point |
| --- | --- | --- |
| ESP32 firmware | micro-ROS node, drive control, encoders, odometry, neck servo | [`src/`](src/) |
| Raspberry Pi runtime | systemd units, micro-ROS agent, SO-ARM bridge, LiDAR support | [`pi/`](pi/) |
| Gamepad coordinator | dead-man controls and ROS command translation | [`pi/robot_teleop/`](pi/robot_teleop/) |
| Simulation | PyBullet model with simulated odometry and LiDAR | [`simulation/`](simulation/) |
| Operator controls | control map and pre-motion checks | [`docs/gamepad-control-map.md`](docs/gamepad-control-map.md) |

## Repository status

The software is an integration project for a specific physical robot, not a
general-purpose robot distribution. Hardware calibration, controller mapping,
and USB device names must be verified on the target system before deployment.
The drive calibration values in firmware and the SO-ARM EEPROM travel limits
are robot-specific.

Use `main` for the current integrated code. The remote `simulation-sync`
branch was merged into `main` and has no remaining unique commits; it can be
deleted once its history is no longer needed.

## Quick start

### Run the simulation

The simulator has no ROS or hardware dependency. From the repository root:

```bash
python3 -m pip install --user pybullet
python3 simulation/pybullet_sim.py --seconds 20
```

Use `--gui`, `--2d`, `--2d-live`, or `--2d-gui` for visual modes. See the
[simulation guide](simulation/README.md) for dependencies and options.

### Build ESP32 firmware

Install PlatformIO, connect the intended ESP32, and build the production image:

```bash
python3 -m platformio run -e esp32dev
```

The diagnostic environments (`servo_sweep`, `servo_idle`, and
`motor_diagnostic`) intentionally bypass parts of the production runtime. Use
them only with the matching physical checks and safety setup.

[`deploy.sh`](deploy.sh) builds locally and flashes the ESP32 attached to the
Pi over `/dev/ttyUSB0`. It stops the user-level micro-ROS agent while the port
is in use and restarts it only for the production image. Review the script and
confirm the serial device before running it.

### Prepare the Pi runtime

The Pi services and their verification commands are documented in the
[Pi runtime guide](pi/README.md). The gamepad coordinator has its own
[installation and mapping guide](pi/robot_teleop/README.md).

Before enabling any motion, verify the ROS graph and controller input:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joy
ros2 topic echo /odom --once
systemctl --user is-active micro-ros-agent.service robot-teleop.service gamepad-joy.service
```

## Development checks

Run the coordinator tests without ROS or hardware:

```bash
PYTHONPATH=pi/robot_teleop/src python3 -m unittest discover -s pi/robot_teleop/tests -v
```

For any firmware or runtime change, also check the relevant physical interface
on the robot with motion inhibited before attempting a lifted-wheel test.

## Documentation map

- [Gamepad controls and safety checks](docs/gamepad-control-map.md)
- [Reachable-workspace characterisation](docs/calibration/reachable-workspace.md)
- [URDF workspace visualisation](simulation/README.md#visualise-the-calibrated-arm-workspace)
- [Raspberry Pi runtime](pi/README.md)
- [Gamepad coordinator](pi/robot_teleop/README.md)
- [Simulation](simulation/README.md)

## Contributing

Keep changes small and reviewable, avoid committing generated files or
credentials, and document any robot-specific calibration change with its
measurement source. Do not merge or deploy motion-related changes without a
proportionate hardware safety check.
