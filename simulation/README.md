# Lightweight robot simulation

This simulation loads the supplied v6 robot model directly from this
repository: `simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf`.

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

To inspect the model in 3D:

```bash
python3 simulation/pybullet_sim.py --gui --seconds 60
```

## Visualise the calibrated arm workspace

The URDF joint limits are the calibrated limits for this robot. Generate a
dense, repeatable kinematic workspace estimate and view it around the 3D robot
model with:

```bash
python3 simulation/pybullet_sim.py --workspace --gui --seconds 60
```

Green points are gripper-centre positions in `base_footprint` that result from
sampling the allowed arm joint configurations. The command also writes a PLY
point cloud to `data/reachability/urdf_workspace.ply`; open it in MeshLab or
CloudCompare if a standalone view is useful.

For a finer cloud, increase the samples and reduce voxel size:

```bash
python3 simulation/pybullet_sim.py --workspace --gui \
  --workspace-samples 50000 --workspace-voxel-m 0.005 --seconds 90
```

This is a model-derived *kinematic* workspace. It does not claim platform,
cable, or self-collision clearance where that collision geometry is absent
from the URDF. Add such geometry before using the cloud as an automatic motion
or collision decision.

For the low-resource top-down view (robot path and simulated LiDAR points):

```bash
python3 simulation/pybullet_sim.py --2d --seconds 60
```

For a genuine realtime flat 2D window (no 3D renderer):

```bash
python3 simulation/pybullet_sim.py --2d-live --seconds 60
```

Install its small display dependency once:

```bash
python3 -m pip install --user pygame
```

For a realtime view directly in the PyBullet window, with the actual robot
geometry shown from above:

```bash
python3 simulation/pybullet_sim.py --2d-gui --seconds 60
```

The orange points are live simulated LiDAR returns, blue is the path, and
green is the robot heading. The flat 2D mode does not require Matplotlib.

This requires Matplotlib in addition to PyBullet:

```bash
python3 -m pip install --user matplotlib
```

The simulator drives the base kinematically, but separates ground truth from
the sensor estimate. PyBullet ray casts use the true pose; the reported
360-ray LiDAR ranges have Gaussian noise (default standard deviation 1 cm).
Wheel odometry is reconstructed from noisy left/right wheel travel, with a
small fixed scale error representing unequal wheel diameter or encoder
calibration. The 2D views use this odometry pose, while the terminal prints
the true pose and odometry error.

The defaults are repeatable (`--seed 7`). Tune the errors for experiments:

```bash
python3 simulation/pybullet_sim.py --2d-live --seconds 60 \
  --lidar-noise-std 0.02 --wheel-noise-fraction 0.04 \
  --wheel-scale-error 0.02
```

This is the sensor layer needed before adding an occupancy-grid SLAM
consumer; the simulator still does not perform SLAM itself.

Set `ROBOT_URDF`, or pass `--urdf`, to use a different model:

```bash
ROBOT_URDF=/path/to/robot.urdf python3 simulation/pybullet_sim.py
# equivalent:
python3 simulation/pybullet_sim.py --urdf /path/to/robot.urdf
```
