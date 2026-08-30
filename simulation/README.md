# Lightweight robot simulation

This simulation loads the calibrated v6 robot model committed at
`simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf` by default.
The `--urdf` option or `ROBOT_URDF` environment variable can select another
model, but results from another model are not automatically comparable.

It uses PyBullet in `DIRECT` mode by default, so it does not require a GPU,
Gazebo, ROS, or RViz. The model includes the actual base plate dimensions,
wheel dimensions, casters, mast, arm approximations, neck, and RPLIDAR A1
placement from the URDF.

## Run

Install the one dependency in a user environment:

```bash
python3 -m pip install --user pybullet
python3 simulation/pybullet_sim.py --seconds 20
```

To inspect the model visually:

```bash
python3 simulation/pybullet_sim.py --gui --seconds 60
```

The simulator drives the base kinematically and prints a simulated 360-ray
LiDAR minimum range. It does not simulate motor control, ROS, or physical
servo behaviour. The noisy wheel-odometry parameters are available for
experiments, but this repository does not yet provide an occupancy-grid SLAM
consumer.

## Workspace estimate

Generate the collision-filtered gripper-centre estimate used by the HOM-8
reachable-workspace work:

```bash
python3 simulation/pybullet_sim.py --workspace --gui --seconds 60
```

The result is written to `data/reachability/urdf_workspace.ply`. The display
shows lower points in blue and higher points in red. An optional horizontal
slice can be added after measuring the real raised platform height:

```bash
python3 simulation/pybullet_sim.py --workspace --gui \
  --workspace-slice-z-m PLATFORM_HEIGHT_IN_METRES --seconds 60
```

This estimate uses URDF geometry, URDF joint limits, and selected modelled
collision volumes. It does not include the real platform, every cable, or all
possible self-collision details. Read
[`docs/calibration/reachable-workspace.md`](../docs/calibration/reachable-workspace.md)
before using it near the physical robot.

Set `ROBOT_URDF` if you need to test a different model:

```bash
ROBOT_URDF=/path/to/robot.urdf python3 simulation/pybullet_sim.py
```
