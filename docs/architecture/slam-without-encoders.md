# SLAM without wheel encoders

Date: 2026-10-01. Status: implementation plan; `REQUIRES_HARDWARE_TEST`.

## Goal and decision

Build and save an indoor map, estimate the base pose, and reload the map for
localization using the existing RPLIDAR A1, RGB camera, and ToF camera.
Wheel encoders stay disabled. Start with manually driven mapping; autonomous
navigation follows only after localization and obstacle detection are validated.

Recommended first implementation: **RF2O LiDAR odometry + SLAM Toolbox** on
ROS 2 Jazzy. RF2O estimates planar motion from consecutive laser scans;
SLAM Toolbox uses the resulting odometry transform and scans to build a 2D
map and correct drift through loop closure. This is a proposed architecture,
not a measured performance result on Homer.

No encoders does not mean no odometry: sensor-derived odometry supplies the
continuous local motion estimate. Do not substitute a constant
`odom -> base_link` transform or integrate `/cmd_vel` as measured motion.

## Existing system and prerequisites

- `pi/rplidar.service` uses ROS 2 Jazzy and publishes `/scan` in frame `laser`.
- `pi/rgb_object_detection.py` publishes `/rgb_camera/image_raw`, stamped at
  processing time, with frame `rgb_camera`. It currently publishes no
  `CameraInfo`; acquisition timing and optical-frame conventions need review.
- `tof-depth-publisher.service` is referenced in the repository, but its
  implementation is external. The user confirms the ToF camera is available;
  its model, driver, topics, calibration, range, and live health remain to be
  inspected.
- Encoder disabling is implemented on branch `disable-encoder-odometry`,
  commit `d3d602d`. Apply that change and flash its built firmware before the
  hardware trial; flashing has not been verified in this planning task.
- Preserve the existing teleop path, fixed USB identities, drive status,
  command timeout, and immediate stop path.

## Sensor roles

| Sensor | First mapping milestone | Later integration |
| --- | --- | --- |
| 2D LiDAR | Local odometry, occupancy map, loop closure | Localization and planar obstacles |
| ToF depth | Record and characterize alongside mapping | Obstacles above/below the laser plane; depth for RGB-D mapping if suitable |
| RGB | Record synchronized evidence of the route | Appearance-based loop closure and object context with RTAB-Map |

All three sensors need not enter the first pose estimator. A working LiDAR
baseline provides a reference for measuring whether camera integration helps.

## Proposed data flow and transform ownership

```mermaid
flowchart LR
  lidar[RPLIDAR driver] -->|/scan| odometry[RF2O]
  lidar -->|/scan| slam[SLAM Toolbox]
  odometry -->|/odom and odom to base_link TF| slam
  slam --> map[/map and map to odom TF]
  rgb[RGB image and calibration] --> recording[Recorded sensor dataset]
  tof[ToF depth and calibration] --> recording
  lidar --> recording
```

Use `map -> odom -> base_link -> laser`. RF2O owns `odom -> base_link`;
SLAM Toolbox owns `map -> odom`. A measured static sensor transform owns
`base_link -> laser`. Check any existing `base_footprint` chain before adding
publishers; each child frame must have one parent and each transform one owner.
The ESP32 bridge must not supply encoder odometry during these trials.

Camera frames must include the measured mount and optical transforms.
For initial experiments, hold the neck at one measured fixed angle. If the
camera moves later, publish its time-varying transform with validated angle
and timing information; servo commands alone are not measured joint feedback.

## Stage 1 — Sensor inventory and recorded baseline

Inspect the Pi read-only before installing anything. Record ROS distribution,
Pi model, memory, CPU load, temperature/throttling, sensor models, topic types,
rates, timestamps, frame IDs, depth encoding/units, valid range, and intrinsics.
Check services and topic publishers without opening either serial device a
second time. Confirm TF direction and LiDAR orientation against the robot.

Capture a stationary bag and then a slow manually driven room loop with
`/scan`, `/tf`, `/tf_static`, drive/link status, RGB, available depth, and
camera calibration topics. Discover actual depth topic names first. Set
recording rates/storage limits so camera recording does not interrupt scans.
For replay, use bag clock and `use_sim_time` consistently.

Acceptance: usable scans and timestamps throughout a 10-minute baseline;
measured base-to-LiDAR transform; sensor inventory saved; camera/depth gaps
explicitly recorded. RGB-D calibration gaps do not block the LiDAR stage.

## Stage 2 — Replace encoder odometry with LiDAR odometry

Build and test a pinned revision of the upstream RF2O ROS 2 branch against
Jazzy and the Pi architecture. Its ROS 2 branch exists, but compatibility and
runtime performance on this Pi must be demonstrated before adopting it.
Configure `/scan`, `laser`, `base_link`, `odom`, and `/odom`; check the actual
upstream parameters rather than copying a launch file without inspection.

Replay the bag before live operation. Check stationary drift, straight motion,
reverse motion, gradual turns, and a measured return to the starting mark.
Verify the published base pose compensates for the LiDAR's mounting offset.
Log scan age, processing latency, dropped data, tracking loss, and pose jumps.

Initial proposed acceptance targets, to refine from baseline data: stationary
drift below 0.05 m and 3 degrees over 60 seconds; end-to-end odometry age below
0.2 seconds for at least 95% of samples; no unexplained jumps or lost tracking
on three slow room loops. Record measured results; these are not guarantees.
If RF2O cannot meet these gates, evaluate RTAB-Map ICP odometry on the same
bags before committing to an alternative.

## Stage 3 — Build and save a 2D map

Add SLAM Toolbox in asynchronous mapping mode with scan matching and loop
closure enabled. Set `scan_topic=/scan`, `base_frame=base_link`,
`odom_frame=odom`, and `map_frame=map`. Begin with 0.05 m map resolution;
tune range and motion thresholds from recorded scan quality and room geometry.
Keep RF2O as the sole local odometry transform publisher.

Map one furnished room slowly, return to the starting area, then repeat across
a doorway into a second room. Save both the occupancy grid with YAML metadata
and the serialized SLAM Toolbox pose graph. Record versions, parameters, bags,
and manual reference measurements.

Acceptance: repeatable walls and doorways without duplicated geometry, loop
closure without persistent map distortion, and three completed test loops.
Proposed targets: measured wall/door distances within 0.10 m and return pose
within 0.20 m / 5 degrees after loop closure. Revisit targets if reference
measurement uncertainty or sensor behavior makes them unsuitable.

## Stage 4 — Reload and localize

Reload the saved pose graph in SLAM Toolbox localization mode while continuing
RF2O odometry. Start at a known marked pose; test relocalization using a supplied
initial pose before trying arbitrary starts. Do not run another `map -> odom`
publisher concurrently.

Acceptance: restart, recover the correct map pose, and traverse the room three
times with consistent localization. Deliberately test scan dropout and verify
that lost localization is surfaced. Mapping success alone is not this gate.

## Stage 5 — Add ToF and RGB where measurements justify them

First calibrate each camera's intrinsics and the RGB-to-ToF extrinsics. Verify
overlapping fields of view, depth validity, and timestamp skew during motion.
Reproject ToF depth into the RGB optical frame with visibility handling; do not
treat separate RGB and depth pixel coordinates as already registered. Publish
appropriate `CameraInfo` and synchronized image/depth pairs.

Use ToF point clouds for local obstacle coverage after filtering floor, robot,
invalid depth, and out-of-range returns. A forward-facing ToF camera only
covers its field of view; check coverage for reversing and turning separately.

If 3D mapping or visual loop closure is needed, trial RTAB-Map with registered
RGB-D input and LiDAR-derived external odometry. Compare its map and tracking
against the baseline on identical bags. Trial RGB-D odometry separately only
if calibrated depth, scene texture, synchronization, and compute allow it.
For each experiment, select one SLAM backend to own `map -> odom`.
Do not average camera and LiDAR poses without a defined estimator and validated
uncertainty; they may share data and have correlated errors.

Acceptance: demonstrate an actual improvement in obstacle coverage or mapping
quality without scan loss, unbounded processing backlog, or thermal throttling.
Keep the LiDAR baseline if the cameras do not pass these gates.

## Failure behavior and eventual navigation

Expect reduced observability in long uniform corridors, open spaces, and
dynamic crowds; rapid rotation also reduces scan overlap and distorts scans.
Test these conditions explicitly. Visual tracking adds its own lighting,
texture, occlusion, and depth-range limits. Slow motion and a fixed camera
mount are initial experimental controls, not a guarantee of tracking.

Before autonomous navigation, add an odometry/localization freshness and
validity gate that suppresses autonomous drive commands on stale or invalid
pose. The existing scan watchdog records outages after 15 seconds and does
not stop motion; it is not this gate. The ESP32's one-second timeout only
protects against missing commands, not a node continuing to send bad commands.

Nav2 is a subsequent milestone: validate footprint, obstacle layers, controller
limits, sensor coverage, localization-loss stop behavior, and motor speed
response first. Existing PWM scales are open-loop commands and must not be
treated as calibrated physical velocities.

## Implementation deliverables and next action

Create a small ROS bringup package with pinned dependencies, RF2O/SLAM configs,
measured sensor transforms, RViz config, and separate mapping/localization
launch files. Add startup integration only after manual launches pass. Store
the sensor inventory and trial results under `docs/calibration/`; store map
metadata and reproducible run instructions alongside the bringup configuration.

Next action: inspect the live Pi topics/services and ToF model, measure sensor
mounts, and record the baseline bag. This plan does not install packages, flash
firmware, start SLAM, or command robot motion.

## Primary references

- [RF2O upstream ROS 2 implementation](https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros2)
  — planar motion estimation from laser scans; check its launch and code for TF/output configuration.
- [SLAM Toolbox documentation](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md)
  — odometry and scan inputs, mapping, serialization, and localization.
- [RTAB-Map ROS 2 launch configuration](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py)
  — RGB-D, external odometry, scan inputs, and TF ownership options.
