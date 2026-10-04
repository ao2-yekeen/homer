# Agent handover: camera–lidar odometry and live RViz navigation

Date: 2026-10-04. Status: implementation plan; **REQUIRES_HARDWARE_TEST**.
This document is the entry point for the next implementation agent. It records
the latest user scope and supersedes older simulation-first execution steps.

## 1. Deliver the requested result

Start SLAM in an unknown environment with an empty map. The user selects a
reachable point in discovered free space using RViz; Homer navigates there,
avoids lidar-visible obstacles and stops. Use the available monocular RGB
camera and temporary LDS-01 lidar to establish reliable metric odometry.
Do not require a previously saved map.

Implement in sections and qualify each before connecting it to physical
navigation. The immediate work is sensor calibration, recorded-data fusion
and independently measured motion accuracy. Simulation is deferred by the
user. Keep tests short, normally 20–30 cm movements. Automatic exploration,
object search/pickup, tags, ToF and wheel encoders are outside this MVP.

The user has already authorized connecting through `ssh pi`. Routine read-only
inspection does not need another permission question. Physical trials require
operator coordination for measurements and readiness; do not rerun the failed
automatic movement script or treat this handover as an instruction to move now.

Read `/home/ao2-yekeen/AGENTS.md` and the complete
`/home/ao2-yekeen/PROJECT_PLAN.md` first. The latter describes the wider grasping
research project; the current user instructions above set this task's scope.
Preserve existing teleoperation and arm control.

## 2. Established failure and unresolved causes

The previous stack used RF2O scan-to-scan range-flow odometry, with SLAM Toolbox
building the map. It did not fuse camera measurements into odometry.

- A positive drive command physically moved backward, according to the user.
- The final distance trial reported about **9 cm estimated movement**, while the
  user measured **50 cm actual travel**. Unexpected movement toward the end was
  also reported. Estimated heading changed by about 9.6 degrees.
- The trial sent commands for roughly 5.8 seconds before zero. Its command log
  does not establish the cause of the physical direction change.
- The map published approximately every three seconds. Faster visualization
  would not correct the displacement error; local obstacle processing must use
  live scans independently of map rendering.
- Stationary results do not establish moving accuracy. The historical 60-second
  stationary trial had about 1.1 cm translation and 3.7 degrees rotation drift.

The failure could involve frames, scan registration, sensor mounting/timing,
drive polarity or command scaling. None is a confirmed root cause. Do not
multiply odometry by 50/9, invert firmware globally, or declare lidar unusable
from this single trial. Do not use failed RF2O displacement as the sole stop
condition for another automatic trial.

Historical evidence: [accuracy results](slam-accuracy-2026-10-04.md), Pi
`~/homer-evidence/section-1-mvp/`, and workstation
`~/.local/state/homer-slam/section-1-mvp-20261004/`. The final record is
`measured30-20261004-092340.json`; these motion records are not equivalent to a
synchronized raw camera/lidar bag. Verify what raw recordings actually exist.
Never run the legacy `/tmp/homer-measured-30cm*.py` or
`/tmp/homer-short-motion*.py` scripts.

## 3. Code and runtime starting points

These are repository findings or historical runtime observations, not a fresh
inspection of the Pi. Verify deployed revisions and live ownership first.

| Component | Entry point / observation | Implementation consequence |
| --- | --- | --- |
| SLAM bringup | `pi/homer_slam/homer_slam/bringup.py` | Existing bringup starts RF2O; add a narrowly scoped external-odometry mode if fusion is selected |
| Lidar odometry | `pi/homer_slam/dependencies.repos` pins RF2O commit `b38c68e46387b98845ecbfeb6660292f967a00d3` | Keep as a comparison baseline, not accepted distance truth |
| SLAM parameters | `pi/homer_slam/config/slam.yaml` | Retain SLAM Toolbox mapping; real operation uses `use_sim_time=false` |
| Geometry | `pi/homer_slam/homer_slam/model.py`, `pi/homer_slam/config/geometry.yaml` | User selected URDF geometry; metadata does not prove replacement sensor mounting |
| Canonical model | `simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf` | Use its lidar position as requested; verify actual rigid mount and facing direction |
| RGB capture | `pi/rgb_object_detection.py` | Publishes `/rgb_camera/image_raw`; currently no CameraInfo and stamps after capture/detection processing |
| Recording | `pi/homer_slam/homer_slam/operations.py` | Default recording omits RGB; explicitly include image, calibration and command topics |
| Motor bridge | `pi/esp32_serial_bridge.py` | Consumes autonomous `/cmd_vel` and teleop `/teleop/cmd_vel`; preserve takeover and mode priority |
| Firmware | `firmware/src/config/robot_config.h` | Encoders disabled; velocity-like inputs map to PWM; existing timeout is one second |
| Teleoperation | `pi/robot_teleop/src/robot_teleop/` | Reuse working controls for operator-measured data collection |

Historical Pi source is `~/mobile-robot`, deployed by copying files rather than
Git; workspace is `~/homer_slam_ws`. Verify before deploying. ROS is Jazzy.
The unit named `rplidar.service` was overridden to run the working LDS-01
`hlds_laser_publisher` at 230400 baud. Preserve that driver and fixed discovered
USB path; do not replace it merely because the unit name says RPLIDAR.
The weekend driver override was
`~/.config/systemd/user/rplidar.service.d/90-weekend-lds01.conf`.

The existing laptop RViz relay was read-only: scan/map/odometry/TF transport
does not establish RGB transport or working Nav2 actions/cancellation.
Nav2 availability, camera rate, Pi thermal headroom, mounting and calibration
remain to be checked. Keep the camera head fixed during odometry trials.

## 4. Evidence-based estimator decision

Use the [peer-reviewed assumptions audit](slam-navigation-peer-reviewed-assumptions-2026-10-04.md)
as the source register, including its limitations. Principal evidence:

| Decision | Peer-reviewed support | What it does not establish |
| --- | --- | --- |
| Evaluate lidar odometry rather than assume ICP fixes it | Jaimez et al., ICRA 2016 (RF2O); Censi, ICRA 2008; Pomerleau et al., Autonomous Robots 2013 | A winning implementation for LDS-01 on Homer; the last benchmark uses 3D data |
| Constrain monocular scale with calibrated range observations | Chiodini et al., IEEE TIM 2020, DOI `10.1109/TIM.2020.2964066`; Scaramuzza/Fraundorfer, IEEE RAM 2011 | A ready ROS 2 package or adequate runtime on Homer; the tutorial is not independent replication of this fusion method |
| Handle weak geometry, timing and wrong associations | Labbé/Michaud, JFR 2019; Cadena et al., IEEE TRO 2016 | The root cause of the failed trial |
| Separate mapping, planning and obstacle control | Macenski et al., IROS 2020; Fox et al., IEEE RAM 1997 | A collision guarantee or correct motor dynamics for Homer |

Chiodini's planar-range/monocular method is direct feasibility evidence.
Usable public ROS 2 code has **not** been verified. Survey/tutorial agreement
supports principles, not two independent reproductions on equivalent hardware.
Record provisional assumptions explicitly. Never describe optical flow,
appearance loop closure, or lidar odometry plus an RGB display as metric fusion.

## 5. Section A — sensor and recording foundation

Initial allocation: 15–30 minutes of inspection, then calibration as needed.
Autonomous motor output remains blocked.

1. Inspect services, device ownership, TF tree, publishers and deployed configs
   over `ssh pi`. Record exact versions and current driver parameters. Confirm
   one `/scan` publisher and identify every `/odom` and TF owner.
2. Inspect scan axes with known front/left/right objects and one measured range.
   Use URDF lidar geometry as requested. Verify rigid attachment; the sensor was
   previously described as sitting separately, so attachment is not established
   by a valid URDF. Correct only demonstrated discrepancies and document them.
3. Reuse the existing RGB capture owner. Add calibrated CameraInfo, resolution,
   distortion model and a valid optical-frame transform. Do not open the webcam
   from a second competing process. Calibrate at the actual capture resolution;
   measure camera-to-lidar extrinsics and validate by projecting scan points
   onto visible physical surfaces. No invented intrinsics or extrinsics.
4. Timestamp near acquisition, quantify capture/buffer delay and synchronization
   error, and record scan timing. If hardware acquisition timestamps are
   unavailable, document the approximation and measured bound. A receipt-time
   stamp or larger synchronization window must not disguise unknown delays.
5. Demonstrate common field of view and useful range/visual correspondence.
   Account for occlusion: a projected scan point near an image feature does not
   establish that both observe the same surface. A planar scan cannot supply
   depth to every image pixel. Distinguish radial laser range from optical z.
6. Record `/scan`, RGB, CameraInfo, `/tf`, `/tf_static`, candidate/baseline odometry,
   diagnostics, drive status/link and teleop/autonomous commands. Record on the
   Pi if RGB transport is absent; check storage and actual bag contents/rates.

**Exit:** reproducible calibrated raw recording, defensible timing and useful
shared observations. If physical overlap is inadequate, report the required
mount/view adjustment before attempting fusion. Do not silently fall back to
lidar-only and call it completion of the requested fused estimator.

## 6. Section B — measured data and fused odometry

Initial allocation: 10–15 minutes operator recording; 45–90 minutes initial
implementation assessment. These are timeboxes for assessment, not a promise
to finish a new estimator in 90 minutes.

1. Collect one short stationary segment, an operator-controlled 20–30 cm
   translation, and a small measured turn using existing teleop. Record tape
   endpoints and an independent heading/video reference with timestamps. The
   operator stops the motion; do not recreate an RF2O-controlled distance test.
2. Spend at most an initial 30 minutes assessing reusable implementations.
   Record source, license, pinned revision, build instructions, sensor inputs
   and output contract. Many visual packages require stereo, depth or IMU;
   do not assume they accept this monocular/planar setup. Keep one viable fusion
   candidate and RF2O/ICP comparison baselines, not a package installation sweep.
3. A usable fused candidate must track visual landmarks, associate valid metric
   range constraints, and optimize pose/scale consistently. If no reusable
   candidate qualifies, implement a bounded prototype using an existing visual
   frontend where possible: robust feature tracks, triangulated landmarks,
   calibrated range association and joint pose/landmark optimization with robust
   losses. Check scale observability and outliers. Do not use an EKF to average
   unscaled monocular translation into metric odometry.
4. Publish candidate output on isolated topics, for example proposed
   `/fusion/odom` and `/fusion/status`. Report association/inlier counts,
   residuals, input age, processing latency and tracking/scale validity. Those
   topics do not currently exist. Unknown uncertainty is not zero covariance.
5. Replay the same bag for all candidates with no path to physical commands.
   Isolate replay clocks/topics from the live robot and compare independent
   displacement/heading references. Use fixed metric scale, not similarity
   alignment that removes scale error. Fix demonstrated frame/timing issues
   before attributing an improvement to the algorithm.
6. Measure processing rate and delay on target hardware. Use laptop processing
   if necessary, but include network loss/latency in qualification and keep the
   final watchdog on the Pi. Test failure indications with recorded weak
   geometry, poor visual tracking and bad association; message freshness alone
   is insufficient.

**Exit:** fused odometry within the proposed **3 cm over a measured 30 cm** and
**3 degrees heading** gates, plus usable latency and truthful failure status.
Record stationary drift separately. One passing short trial only qualifies
that operating envelope; it does not prove arbitrary-room reliability. If no
candidate passes, document the blocker and continue odometry work, not Nav2
physical movement. A passing lidar-only baseline is an explicitly labelled
interim result, not fulfilment of camera–lidar fusion.

## 7. Section C — integrate one pose owner and qualify actuation

Initial allocation: 20–45 minutes after the odometry gate.

Add an external-odometry launch mode with explicit ownership and duplicate
checks. Exactly one accepted estimator publishes `/odom` and `odom → base_link`;
SLAM Toolbox owns `map → odom`. Retain RF2O mode for rollback; comparison nodes
must not publish competing live transforms. Check the bringup lock/preflight
logic rather than removing protection to accommodate a second estimator.

The current command-to-PWM factors do not establish m/s or rad/s. Independently
verify polarity, low-speed response, deadband, turning response, acceleration
and residual stopping travel. Implement/document the minimum metric-command
adapter required by the measured envelope. Log battery/load conditions. Never
derive motor speed from the failed odometry. Build any firmware changes on the
PC and flash only the finished compatible image on the Pi.

Before autonomous tests, qualify cancellation, teleop priority, command expiry
and a Pi-local final watchdog. Start link-loss/stale-pose/sensor-loss tests with
motor output blocked; then perform bounded operator-observed stopping checks.
The proposed 250 ms software response target is not a physical stop guarantee;
record actual stopping distance separately. Faults must produce zero and require
deliberate restart, without automatic reversal. Existing firmware's one-second
timeout is not sufficient evidence that the proposed watchdog gate passes.

**Exit:** one pose owner, measured velocity/stopping envelope and tested final
command authority. No unexplained polarity or stop behaviour remains.

## 8. Section D — live SLAM, Nav2 and RViz with output blocked

Initial allocation: 45–90 minutes, after pose and drive gates.

Retain SLAM Toolbox in mapping mode with an empty-map startup. Add the smallest
Jazzy-compatible Nav2 launch/config supporting `NavigateToPose`, a global
costmap from the live map and a local obstacle costmap from `/scan`. Set footprint,
inflation, controller limits and collision stopping zones using actual robot
dimensions, sensor age and measured braking. Unknown cells remain blocked;
initial goals must lie in discovered reachable free space. Disable automatic
backup and recovery spins; planned turns are allowed.

Use a single command chain: controller → optional smoother → collision monitor
→ Pi watchdog/qualified adapter → serial bridge. Adapt message types to the
installed Jazzy components (`Twist` versus `TwistStamped`) explicitly. No
parallel publisher may bypass the final gate through the bridge's autonomous
input. Preserve teleop priority and do not rely on collision monitor alone to
detect inaccurate-but-fresh odometry.

Verify bidirectional ROS transport for actions, feedback and cancellation;
the historical visualization relay is insufficient evidence. Configure RViz
map, scan, robot/TF, pose, planned route, goal tool and status. Check sensor QoS
and transient-local map/TF delivery. With physical output blocked, exercise a
free goal, occupied/unreachable goal, cancellation and data/link loss, checking
the final output and absence of bypass paths.

**Exit:** reproducible launch, live map and pose, working goal/action/cancel
transport, correct rejection and zero-output fault handling.

## 9. Section E — short physical demonstration

Initial allocation: 10–20 minutes with the operator. Use the already established
clearance and stop arrangements; raise a new issue only if a contrary observation
appears. Keep the arm stowed and camera head fixed.

1. One nearby reachable free-space goal, initially around 20–30 cm where the
   measured stopping envelope permits it. Measure endpoint independently;
   proposed goal tolerance is 10 cm and 10 degrees, not a published guarantee.
2. One stationary lidar-visible obstacle with a genuinely reachable detour;
   allow only the bounded path needed for this test. No contact or unintended
   reversal is acceptable.
3. One occupied/unreachable goal, which must fail cleanly. Check cancellation
   and teleop takeover with bounded motion after the blocked-output checks.

Stop on tracking invalidity, unexpected direction, excessive residual motion
or a failed gate. Do not automatically retry. Low obstacles, overhangs and
drop-offs outside the lidar plane remain unqualified; this MVP does not claim
general obstacle coverage from a monocular camera.

## 10. Deliverables, checks and rollback

For each section, update a project results document with PASS / FAIL /
REQUIRES_HARDWARE_TEST; measured values; exact revision/deployed files; calibration
provenance; launch commands; bag/reference locations; latency and stop results;
known limitations; and the next unresolved gate. Keep large raw bags outside
Git and link their locations. Do not fabricate missing measurements.

Deliver the calibrated sensor configuration, record/replay commands, reproducible
fused-estimator build/launch, external-odometry mode, qualified command adapter
and watchdog, minimal Nav2/RViz configuration, and the actual physical results.
Add meaningful checks for projection/frame/units, rejected invalid calibration,
time handling, ownership, watchdog expiry and cancellation as those components
are implemented. Run package/build checks appropriate to changed code and
`git diff --check`; compilation does not replace hardware qualification.

Back up changed Pi service/config files before deployment. Rollback: stop new
Nav2/fusion command producers, verify zero at the final bridge input, disable
their startup, restore changed files and the existing driver/teleop services,
then restore the original RF2O mapping mode only for its documented uses.
Do not delete working camera, arm, driver or teleop components. Keep autonomous
startup disabled until all required gates pass.

Work on the existing task branch `live-slam-rviz-navigation-plan` when continuing
this handover; verify current Git state before changes. At handover creation the
preceding plan revision was `f39f023`. Follow workspace Git instructions for new
tasks, push checked changes for review, and do not merge physically unqualified
implementation into main.

Completion means **measured fused odometry plus empty-map RViz goal navigation,
qualified stopping and obstacle avoidance**, with evidence. A package installed,
a moving RViz marker, a fast map redraw or a successful saved-map reload alone
does not complete this task.

## Reading order

1. This handover, then the [time-boxed MVP plan](slam-rviz-navigation-fast-mvp-plan.md).
2. [Peer-reviewed assumptions and source register](slam-navigation-peer-reviewed-assumptions-2026-10-04.md).
3. [Failed movement evidence](slam-accuracy-2026-10-04.md) and
   [weekend LDS-01 setup](slam-lds01-weekend-2026-10-02.md).
4. [SLAM runtime guide](../../pi/homer_slam/README.md) and the code listed above.
5. [Full navigation plan](live-slam-rviz-goal-navigation-plan.md) for deferred
   detail. Older camera-fusion, simulation and object-pickup plans are historical
   context; they do not override this real-world MVP scope.
