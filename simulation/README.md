# Lightweight robot simulation

This simulation loads the supplied v6 robot model directly:

`/mnt/c/Users/abdul/Downloads/home_mobile_manipulator_v6_real_base_layout.urdf`

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

The simulator currently drives the base kinematically and prints a simulated
360-ray LiDAR minimum range. This is intentional: it keeps the first SLAM
test light and makes the sensor output independent of PyBullet motor tuning.
The next layer will add noisy wheel odometry and an occupancy-grid SLAM
consumer using the same scan data.

Set `ROBOT_URDF` if the file is moved:

```bash
ROBOT_URDF=/path/to/robot.urdf python3 simulation/pybullet_sim.py
```
