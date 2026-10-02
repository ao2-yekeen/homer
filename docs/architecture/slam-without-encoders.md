# Encoder-free SLAM with RViz over the hotspot

Implementation: `pi/homer_slam`. Read its [operational guide](../../pi/homer_slam/README.md)
for build, network setup, mapping, saving, localization, replay, and rollback.

- ESP32 encoder capture/odometry disabled; Pi bridge encoder publication off by default.
- Pi: existing sensor drivers, pinned RF2O LiDAR odometry, asynchronous SLAM Toolbox.
- Laptop: Jazzy Docker container with WSLg RViz and optional recording.
- Hotspot is primary; Tailscale is a separately selected network profile.
- Canonical robot URDF supplies the LiDAR mount transform and base outline.
  RViz displays the existing primitive RobotModel using separate illustrative
  display frames; it can be disabled for the smallest rendering workload.
- One owner per estimator transform: `map -> odom -> base_link -> laser`.
- 0.05 m map, 3-second map updates, 10 FPS RViz, 5 Hz lightweight robot markers.
- Save both occupancy data and pose graph; reload the pose graph for localization.
- RGB/ToF inspection and optional recording are supported. Camera fusion and autonomous
  navigation remain later milestones, after calibration and physical validation.

`REQUIRES_HARDWARE_TEST`: confirm scan origin/orientation from the model, stationary
and moving drift, loop closures, map dimensions, saved-map localization, hotspot-only
ROS traffic, and compute gates. Implementation/test results are recorded separately;
compilation or synthetic scans do not count as physical validation.
