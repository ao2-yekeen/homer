# Camera–lidar SLAM literature review for Homer

Reviewed 2026-10-04. Scoped engineering review of primary papers and author
implementations, not a systematic review or a claim of reproduced benchmark
results. Companion: [simulation-first validation](slam-simulation-validation-plan.md).

## Decision

The first candidate to evaluate offline is **RTAB-Map with calibrated RGB-D
odometry and the existing 2D lidar**, on the laptop. This is conditional on the
Logitech RGB camera and Arducam ToF providing sufficient calibrated, synchronized
overlap for tracking. If they do not, reject that configuration rather than
invent missing depth. Monocular visual tracking with lidar-based scale constraints
is a second, more research-heavy option. Neither has been implemented here.

Camera appearance matching alone is useful for recognizing revisited places,
but is not an independent metric motion estimate. It cannot by itself solve the
short-distance stopping failure. A camera-derived metric estimate must first be
validated independently, then compared/fused with lidar odometry.

## What failed on Homer

The operator measured approximately **50 cm physical travel** in the last
attempt intended to cover 30 cm. RF2O reported approximately **9 cm forward**;
the trial stopped for heading change, not because it reached its distance target.
This is roughly 82% underestimation relative to the reported tape travel, subject
to the approximate measurement. The operator also reported backward movement
near the end. Motor telemetry recorded commands -0.35 and -0.45 followed by zero;
it does not establish actual wheel direction, coasting, or the cause of reversal.

The current estimator/control combination therefore failed. Stationary drift
results do not establish moving accuracy, and this test does not establish that
all lidar-only SLAM is inaccurate. Mounting, timing, scan matching, estimator
freshness, actual motor response and sign conventions remain possible causes.
Do not correct this with a global 50/9 multiplier from one failed trial.

**No further remotely commanded movement tests.** The user requires simulation
before deployment. Research, offline replay, simulation and read-only live
sensing may continue. Nav2 is paused. Any later physical validation must use an
explicitly agreed operator-controlled procedure; earlier movement authorization
must not be reused to resume these automatic tests.

## Hardware and integration constraints

| Component | Evidence in this project | Consequence |
| --- | --- | --- |
| LDS-01 | 2D `/scan`, roughly 5 Hz, range metadata 0.12–3.5 m | Not a multi-beam 3D lidar; substantial time passes between scans |
| RGB camera | Logitech C930e; `/rgb_camera/image_raw`; source stamps after capture/colour detection | Need measured capture latency, intrinsics and CameraInfo, not just images |
| ToF | Live Pi implementation: 240×180, 10 Hz requested, `32FC1` values in millimetres | Requires unit conversion and RGB/depth registration before RGB-D use |
| Mounts | URDF selected by user; camera head can tilt; temporary lidar rigid mounting not confirmed | Verify scan origin; hold camera head fixed initially; a changing extrinsic invalidates fixed-camera odometry |
| IMU | No calibrated IMU stream established in the reviewed handover/code | Do not choose a method assuming one is already available |
| Encoders | Disabled; no accepted current wheel-odometry stream | Not currently an independent motion reference |
| Compute | Pi spot temperatures exceeded 80°C; RGB/ToF services consume CPU; laptop available | Prefer camera/fusion evaluation on laptop; measure end-to-end latency |
| Drive | Twist numbers map to PWM; speed is not calibrated | A nominal “0.1 m/s” message does not establish actual 0.1 m/s travel |

The ToF publisher was inspected read-only on the Pi; its implementation is not
tracked in this branch. The C930e camera resolution/FPS, acquisition timestamps,
rolling-shutter behaviour, depth confidence and usable overlap still need
measurement. URDF geometry is a starting model, not proof of current sensor
calibration.

ROS's canonical `32FC1` depth represents metres along the optical Z axis.
Convert valid ToF millimetres to metres in a separate adapter, preserve the raw
stream and mark invalid pixels appropriately. Establish whether this SDK returns
axial depth or radial range before projecting it. Never just resize ToF pixels to
RGB dimensions or index ToF with unregistered RGB coordinates.
[ROS REP 118](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0118.rst).

## Findings from published methods

### RTAB-Map — Labbé and Michaud, Journal of Field Robotics, 2019

The authors combine graph SLAM, appearance-based loop closure, scan registration
and externally supplied odometry. Their comparison includes 2D lidar and visual
configurations and examines practical navigation outputs. The paper was published
in 2019; the arXiv copy was uploaded in 2024, which is not a new 2024 method.
[Paper](https://doi.org/10.1002/rob.21831),
[author manuscript](https://arxiv.org/html/2403.06341v1).

The maintained ROS 2 implementation exposes RGB-D, stereo and ICP odometry nodes,
synchronization and occupancy-map outputs. External odometry can be supplied
instead. That makes it a useful comparison platform for Homer, rather than a
reason to assume its camera automatically repairs poor odometry.
[Author implementation](https://github.com/introlab/rtabmap_ros),
[maintainer explanation of external odometry](https://github.com/introlab/rtabmap_ros/issues/344).

**Homer assessment:** best first integration experiment if registered ToF depth
passes calibration and coverage checks. Evaluate RGB-D motion separately first.
Adding only RGB place recognition to the same failed RF2O input is insufficient.
Its suitability for this particular separate-camera pair is an engineering
hypothesis, not a demonstrated result.

### ORB-SLAM3 — Campos et al., IEEE Transactions on Robotics, 2021

Supports monocular, stereo, RGB-D and visual-inertial configurations, with tracking,
map reuse and loop closure. A plain monocular configuration has an unknown metric
scale; stereo, calibrated depth, or sufficiently observable inertial constraints
supply additional metric information. Its published accuracy is measured on its
benchmark sensor configurations, not Homer's hardware.
[Paper](https://arxiv.org/abs/2007.11898),
[author code](https://github.com/UZ-SLAMLab/ORB_SLAM3).

**Homer assessment:** useful visual-tracking comparator. A bare monocular C930e
feed is not a drop-in centimetre ruler. ORB-SLAM3 does not provide a native LDS-01
fusion pipeline in the reviewed implementation; ROS 2 integration and metric
scale assistance would need separate work.

### Laser-assisted monocular scale — Sensors, 2018

“Scale Estimation and Correction of the Monocular SLAM Based on Fusion of 1D
Laser Range Finder and Vision Data” uses laser distance information to estimate
and correct monocular scale through image reconstruction. It demonstrates a
mechanism for obtaining metric constraints without stereo, rather than simply
averaging two odometry topics.
[Paper/abstract](https://pubmed.ncbi.nlm.nih.gov/29914114/),
[publisher](https://doi.org/10.3390/s18061948).

**Homer assessment:** relevant to an RGB-plus-range research route. A 2D scan can
supply range constraints only where camera and laser observations correspond to
the same scene geometry. Homer's low laser plane and high camera make that
association an explicit challenge. This paper uses a 1D rangefinder; it is not
validated LDS-01 software. Full-text retrieval was blocked during this review;
the assessment is limited to the accessible abstract.

### 2D lidar scale assistance — Takahashi, Hosei University bulletin, 2023

The published abstract describes matching monocular reconstruction and 2D lidar
measurements by viewing direction, then estimating scale from their distance
ratios with robust estimation. This directly addresses the sensor combination
closer to Homer.
[Publication record and abstract](https://cir.nii.ac.jp/crid/1390859525240834048),
[primary publication DOI](https://doi.org/10.15002/00026359).

**Homer assessment:** useful research lead, not a production recommendation. This
is a short departmental bulletin publication. Its full Japanese PDF was blocked
in this session; no reproducible ROS 2 package, quantitative performance claim
or integration readiness was established. Do not imply the abstract proves it
will work here.

### LVI-SAM — Shan et al., ICRA, 2021

A factor graph couples visual-inertial and lidar-inertial subsystems. Lidar depth
helps visual features and initialization; vision helps scan matching and proposes
loops which lidar refines. The design addresses complementary failure modes.
[Paper](https://arxiv.org/abs/2104.10831).

The authors' implementation and datasets use a Velodyne VLP-16 3D lidar, camera
and MicroStrain IMU; its documented build targets ROS 1.
[Author implementation](https://github.com/TixiaoShan/LVI-SAM).

**Homer assessment:** useful architectural evidence for cross-sensor assistance,
but a poor immediate hardware fit. An LDS-01 LaserScan is not the expected 3D
point cloud, and Homer has no established IMU. Adapting it would be a separate
research project.

### FAST-LIVO2 — Zheng et al., 2024 preprint / T-RO accepted work

Fuses lidar, images and IMU using an iterated error-state Kalman filter. Raw lidar
registration and direct image alignment share a voxel map, so both geometry and
appearance constrain state estimation. The author repository was released in
2025 and documents synchronization/calibration equipment.
[Paper](https://arxiv.org/abs/2408.14035),
[author implementation](https://github.com/hku-mars/FAST-LIVO2).

**Homer assessment:** informative modern tightly coupled method, but its 3D
mapping/input assumptions and inertial requirements do not match the current
2D-lidar setup. “Runs on some ARM platforms” is not evidence of performance on
this thermally loaded Pi. Defer unless the sensor platform changes.

## Comparing the practical choices

| Candidate | Adds camera-derived metric motion? | Present hardware fit | Decision |
| --- | --- | --- | --- |
| RGB appearance + current RF2O in a graph | No independent short-term metric estimate | Relatively close | Useful for loops; inadequate as sole repair |
| Registered RGB+ToF odometry + 2D lidar | Yes, if calibration/depth are valid | Conditional | First offline candidate |
| Monocular VO with explicit lidar scale constraints | Potentially | Possible but correspondence is difficult | Second research route |
| Monocular ORB-SLAM3 alone | Not intrinsically metric | RGB available | Comparator only, not distance cutoff |
| LVI-SAM / FAST-LIVO2 | Yes with their intended sensors | 3D lidar/IMU absent | Defer |

## Proposed experiment and estimator ownership

1. Establish fixed sensor mounts, camera intrinsics and optical frames, depth
   units/type, RGB–ToF extrinsics and capture timing. Hold the head at one pose.
2. Evaluate camera odometry and lidar odometry separately on identical simulated
   and recorded inputs. Keep RF2O as a comparator; do not disguise the failure.
3. If camera odometry has valid metric scale and adequate tracking, evaluate
   planar fusion with calibrated covariances and disagreement/failure rejection.
   `robot_localization` is one possible loosely coupled implementation; its
   documentation warns against duplicate correlated information and describes
   `two_d_mode`. It cannot make unscaled monocular data metric or repair bad
   calibration. [Official configuration guide](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/ros2/doc/configuring_robot_localization.rst).
4. Give each raw estimate a separate topic. Only the accepted fusion estimator
   owns `/odom` and `odom → base_link`; raw estimators must stop broadcasting that
   transform. Choose **either** RTAB-Map **or** SLAM Toolbox as the map backend,
   with one `map → odom` owner. Do not start both live on the same frames.
5. Compare endpoint translation/yaw, unscaled trajectory error, wall alignment,
   tracking loss, recovery behaviour, source timestamps and end-to-end age.
   Reject apparent improvement caused by trajectory rescaling.

These steps are a proposed evaluation, not permission to resume physical motion.
Simulation/replay gates and an operator-controlled validation procedure precede
any physical deployment. A slower map publication setting (currently 3 s) and
network/display lag are separate measurements from estimator accuracy. Neither
was changed during this review.
