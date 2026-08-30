# Homer mobile manipulator

Homer is a custom mobile manipulator built around an ESP32, Raspberry Pi,
ROS 2 Jazzy, differential-drive base, SO-ARM 101, and RPLIDAR A1. This
repository contains the firmware, Raspberry Pi integration, operator controls,
calibration material, and PyBullet simulation for that robot.

This is an active engineering repository for one physical robot. It is not a
ready-to-run robot kit: several values still depend on the wiring, controller,
servo calibration, and clearances of the target machine.

## Read this first

The current work is split into two related areas:

- **Arm safety baseline:** named SO-ARM poses and slow, repeatable motion are
  still the first physical prerequisite for automated grasping.
- **Reachable-workspace model:** the current HOM-8 implementation generates a
  conservative workspace estimate from the calibrated URDF. It helps compare
  target positions with the model; it does not authorise a physical move.

In this repository, “HOM-8” refers to the current reachable-workspace issue.
Earlier project notes used HOM-8 for camera viewpoints. Those are different
pieces of work; use the section title and acceptance criteria, not the number
alone, when cross-referencing project notes.

Words such as `REQUIRES_HARDWARE_TEST` mean that software checks cannot prove
the claim. Do not replace that marker with a success claim unless the stated
physical test has actually been performed and recorded.

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

## Status and safety

This is an integration project for one physical robot, not a general-purpose
robot distribution. Hardware calibration, controller mappings, serial-device
names, and wiring must be checked on the target robot before deployment. Drive
calibration and SO-ARM EEPROM travel limits are robot-specific.

The reachable-workspace visualisation is a model-derived planning aid. It is
not a motion command or proof that a physical move is safe. Before a workspace
region is used for planning, physically check the raised platform position,
clearances, and servo-to-URDF calibration. The current robot cannot reach the
floor from its present arm mount, so workspace and grasp tests use a raised
rigid platform only.

## Quick start

### Visual preview

The repository includes this generated simulation preview so readers can see
what the lightweight LiDAR test produces. It is simulation output, not a
photograph or physical validation result.

![PyBullet simulation showing the robot and simulated RPLIDAR scan](docs/assets/simulation-lidar.png)

### Run the simulation

The simulator has no ROS or hardware dependency. From the repository root:

```bash
python3 -m pip install --user pybullet
python3 simulation/pybullet_sim.py --seconds 20
```

Use `--gui`, `--2d`, `--2d-live`, or `--2d-gui` for visual modes. See the
[simulation guide](simulation/README.md) for dependencies and options.

### Inspect the arm workspace

The workspace view samples the calibrated URDF, excludes points behind the
robot, and rejects configurations colliding with the modelled mast, base,
arm mount, or neck:

```bash
python3 simulation/pybullet_sim.py --workspace --gui --seconds 60
```

Use the PyBullet right-panel camera controls, especially **Camera yaw (turn
around)**, to inspect all sides of the model. For assumptions, limitations,
and the required physical validation procedure, see
[reachable-workspace characterisation](docs/calibration/reachable-workspace.md).

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

## Repository map

| Path | Contents |
| --- | --- |
| `src/` | ESP32 firmware for the base, encoders, odometry, neck servo, and micro-ROS |
| `pi/` | Raspberry Pi services, ROS bridges, diagnostics, and tests |
| `simulation/` | PyBullet robot model and workspace/LiDAR simulation |
| `config/` | Measured or pending calibration values; `null` means not measured |
| `docs/` | Operator controls and physical calibration procedures |

## Documentation map

- [Gamepad controls and safety checks](docs/gamepad-control-map.md)
- [Reachable-workspace characterisation](docs/calibration/reachable-workspace.md)
- [Raspberry Pi runtime](pi/README.md)
- [Gamepad coordinator](pi/robot_teleop/README.md)
- [Simulation](simulation/README.md)

For a new contributor, the recommended reading order is this file, then the
[simulation guide](simulation/README.md), then the relevant Pi or calibration
guide. Search for `REQUIRES_HARDWARE_TEST` before treating a result as a
validated robot capability.

## Contributing

Keep changes small and reviewable, avoid committing generated files or
credentials, and document any robot-specific calibration change with its
measurement source. Do not merge or deploy motion-related changes without a
proportionate hardware safety check.
