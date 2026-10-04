# Live SLAM and RViz goal navigation

Date: 2026-10-04. Status: plan only; implementation and physical qualification
remain outstanding. Current priority supersedes the earlier object-search and
pickup MVP. Pickup and autonomous frontier exploration are deferred.

## Goal and first demonstration

Start Homer without loading a map. SLAM builds a live map. The operator selects
a destination and facing direction in RViz; Homer plans and follows a route,
avoids lidar-visible obstacles, and stops at the goal. If the goal is occupied
or unreachable, reject it or fail with an explanation. If an obstacle appears,
replan when a safe alternative exists; otherwise stop.

The first demonstration uses one level room, one obstacle requiring a detour,
and goals inside discovered free space. Unknown space is initially blocked for
planning. Choose successive reachable goals as SLAM discovers more space; this
MVP does not autonomously explore toward an arbitrary point in unseen space.
No saved map, printed tags, extra sensors or object detector is required.

An occupancy map published every three seconds is not itself a navigation
failure. Robot pose, scans and local obstacle processing must remain timely.
Do not make faster RViz map rendering the prerequisite for this work.

## Current evidence and blockers

- Working baseline: LDS-01 driver, RF2O odometry, SLAM Toolbox mapping and RViz.
  Exactly one owner publishes each estimator transform.
- A previous trial travelled approximately 50 cm while odometry estimated
  approximately 9 cm. Drive direction and an end-of-trial reversal were also
  reported. These failures block autonomous hardware goal navigation.
- Firmware maps command values through linear scale 220 and angular scale
  637.5 into PWM. This is not verified closed-loop control in metres/second.
  Nav2 expects metric velocity commands; direct connection is not qualified.
- Existing visualization transport is read-only. A tested bidirectional goal,
  action, cancel and command path will be needed for laptop navigation.
- The existing PyBullet base movement and stationary ROS smoke test cannot
  establish moving-estimator accuracy or physical stopping performance.
- Remote automatic motion remains stopped. This planning request does not
  restart hardware driving. Simulation precedes deployment.

## Sections and acceptance gates

Complete each section before relying on it in the next. Targets below are
proposed engineering acceptance criteria, not measured performance.

### 1. Small simulation and an accurate pose

Deliver one laptop-only Gazebo Harmonic/ROS 2 Jazzy room, Homer geometry with
stowed arm, actual LDS-01 timing/range and a rendered RGB camera. Keep the
camera head fixed. Isolate commands from all real robot bridges and devices.
Use production estimator code, TF conventions and simulated time. Simulator
truth is recorded only for grading and never supplied as navigation odometry.

First implementation steps:

1. Audit existing URDF, geometry override, sensor metadata, camera calibration,
   wheel spacing and firmware command signs. Record unknowns rather than
   inventing calibrated values. Dynamics require inertias absent from the URDF.
2. Create one room with a textured wall and one box that intersects the laser
   plane. Include a plain/repetitive geometry variant for tracking failures.
3. Bridge scan, RGB and clock. Run the existing lidar estimator and SLAM Toolbox
   on known stationary, 30 cm straight and small-turn trajectories.
4. Score estimated trajectory without fitting away scale error. Inspect raw
   scans, mounting axes, TF, capture timestamps, processing delay and scan
   matching before selecting an estimator replacement.
5. If necessary, compare one alternative: RTAB-Map ICP odometry. Evaluate RGB
   tracking as an additional candidate cue only with calibrated timestamps and
   geometry; do not claim a single RGB stream provides metric odometry or that
   RGB loop closure automatically repairs local movement estimation.
6. Replay available real sensor recordings. If synchronized moving raw data
   were not recorded, report that gap; existing pose JSON cannot substitute for
   camera/scan replay. Keep all replay commands remapped away from real control.

Pass: known 30 cm movement has endpoint error at most 3 cm and heading error
at most 3 degrees in the initial tests; no duplicate TF ownership; nominal pose
and scan age at most 250 ms. Report stationary drift, trajectory error and
latency individually. Repeat movement cases over five fixed noise/delay seeds.
Reproduce the 50 cm/9 cm underestimation as a regression. A fresh-but-wrong pose
cannot be detected by a timestamp check alone: demonstrate estimator-quality or
independent consistency checks, or explicitly block deployment where the
failure remains undetectable. Simulation truth may score this, not trigger stops.

Deliverables: simulation launch/world/profile, baseline and candidate reports,
input recordings, calibration provenance and a documented estimator decision.

### 2. Verified command conversion and stopping

Reuse or extract actual firmware command conversion, mode and watchdog logic
for a simulation actuator adapter. Test parity rather than substituting an
ideal Gazebo velocity controller and calling it equivalent to Homer.

Define a validated conversion from Nav2 m/s and rad/s into the existing motor
interface, or implement and qualify actual speed control. Without encoders,
open-loop conversion is approximate and must be tested across measured battery,
load and left/right motor response; failure to achieve bounded response blocks
hardware deployment. Maintain a single final command authority and teleop
priority. Reject non-finite values; clamp velocity/acceleration; require fresh
inputs; output zero on cancellation, invalid tracking, takeover or link loss.
Use a watchdog local to the Pi for laptop failure, with the firmware timeout as
an additional layer. Hold a fault stop until deliberate restart.

Pass in simulation: correct signed movement and turns; controlled 30 cm movement
ends between 27 and 33 cm across tested motor/delay uncertainty; no unintended
reverse at completion. Target zero command within 250 ms of detected invalid
input. Separately measure wheel stopping time and residual travel. Use these
results plus sensor age and footprint to determine stopping clearance.

Deliverables: adapter/control configuration, watchdog, command parity tests and
stop report. Real conversion remains REQUIRES_HARDWARE_TEST until measured.

### 3. Live SLAM with RViz goals in simulation

Run Nav2 alongside SLAM Toolbox in mapping mode, with no AMCL or loaded map.
Use the accepted odometry from Section 1. Global planning uses the live map;
a rolling local costmap receives scans directly for obstacle updates between
occupancy-map publications. Configure the real footprint, inflation and scan
marking/clearing. Start with NavFn planning and a forward-only DWB controller;
verify exact plugin options against installed Jazzy versions.

Set proposed limits of 0.10 m/s and 0.25 rad/s only after Section 2 makes these
metric commands meaningful. Disable automatic backup and recovery spins;
ordinary planned turns remain allowed. Bound goal duration and lack of
progress. Match Twist versus TwistStamped interfaces explicitly at each hop.

Route controller output through smoothing, lidar Collision Monitor and the
final freshness/watchdog gate. No autonomous publisher may bypass that chain.
RViz shows scans, map, robot footprint, global/local costmaps, path, goal and
navigation status. Configure its Nav2 goal tool and cancellation control.

Pass: start with an empty map, click a nearby known-free goal, reach within
10 cm and 10 degrees, then settle without reversal. Report independent true
endpoint error as well as Nav2's own success result. Repeat three short goals.

Deliverables: Nav2 parameters, launch file, RViz configuration and launch guide.

### 4. Obstacles and failure cases in simulation

Keep tests short and targeted:

| Case | Required outcome |
| --- | --- |
| Box blocks the direct route, a free detour exists | Plan around it; reach the goal without contact |
| New obstacle appears along the route | Local obstacle response slows/stops before contact; replan if possible |
| Destination is occupied, outside known free space or unreachable | Reject or abort; do not nudge toward it indefinitely |
| Passage becomes blocked with no alternative | Stop and report failure within the configured progress/time limit |
| Goal cancelled or teleop takes over | Zero autonomous output; no unexpected restart |
| Scan, pose, TF, laptop link or command producer fails | Watchdog stops output; log the reason |
| Fresh timestamps conceal an inaccurate pose | Quality/consistency gate rejects it, or record a deployment-blocking limitation |
| SLAM occupancy publication remains at three seconds | Live lidar obstacle response still passes; no dependence on display refresh |

Use obstacles intersecting the lidar plane. A 2D lidar does not cover every
low object, overhang or drop-off; initial hardware testing uses a bounded level
area and the stowed robot envelope. Repeat core goal/detour cases over three
start/obstacle arrangements and preserve failures.

Pass: zero simulated contacts, clearance consistent with the validated stopping
envelope, accurate endpoints and all applicable stop cases passing.

### 5. Hardware observation and short qualification

Only after simulated estimator/control/navigation gates pass, run the candidate
stack on real inputs with motor output disconnected. Check resource load,
pose/scan age, transform health and independent accuracy. Test bidirectional ROS
transport, cancellation and communication loss; the read-only RViz relay does
not establish command transport readiness.

Then establish a separate operator-coordinated physical procedure before any
automatic motion. Measure direction, response and braking with independent tape
or video references. Verify a 30 cm movement, then a short free-space goal, then
one stationary obstacle detour, preserving immediate stop and teleoperation.
Never use the unvalidated lidar distance as the sole travel limiter. A simulated
pass does not validate real PWM-to-speed calibration or stopping distance.

Pass: independently measured goal error at most 10 cm/10 degrees, no contact or
unintended reversal, and real stopping/clearance within the validated envelope.
If these cannot be demonstrated, return to the failed section.

### 6. Freeze the MVP

Record exact revisions, launch commands, geometry and calibration, speed limits,
stop parameters, evidence and rollback. Final demonstration starts without a
saved map, accepts an RViz goal in discovered free space, avoids a lidar-visible
obstacle and stops accurately. Save maps only as optional evidence.

Deferred: automatic exploration, arbitrary goals through unseen space, camera
object recognition, pickup, ToF integration and broad environment qualification.

## References and relationship to earlier plans

- [Nav2 Jazzy: navigating while mapping](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/).
- [Nav2 Jazzy Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/).
- [RTAB-Map odometry modalities](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/README.md).
- [Existing simulation/replay plan](slam-simulation-validation-plan.md): reuse
  reality-gap and isolation requirements. This plan enables Nav2 simulation
  work and uses RGB/lidar only; earlier RGB-D proposals are optional later work.
- [Physical discrepancy evidence](slam-accuracy-2026-10-04.md).
- [Longer-term object search and pickup](autonomous-mapping-object-navigation-plan.md).
