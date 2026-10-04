# Time-boxed MVP: real-world SLAM and RViz goal navigation

Date: 2026-10-04. Status: plan only; implementation and physical qualification
remain outstanding. Latest user instruction: skip simulation and test in reality.
This supersedes the simulation prerequisite in earlier plans. Offline replay
and motor-disconnected checks remain useful real-data checks, not simulation.

## Outcome

Start with an empty map, choose a destination in RViz, navigate around a
lidar-visible obstacle and stop at the goal. Reject occupied/unreachable goals.
Use short tests and retain independent measurements. The previous 50 cm actual /
approximately 9 cm estimated failure is established; do not repeat its automatic
distance-limited script. Map publication speed is secondary.

## Camera–lidar odometry direction

Latest discussion: evaluate actual RGB/2D-lidar fusion for metric odometry,
rather than assume another lidar-only matcher solves the failure. Camera
landmark tracking supplies visual motion constraints; calibrated associations
between lidar ranges and visual landmarks supply metric scale constraints.
Joint pose/scale optimization must report tracking/association quality.
Chiodini et al. (IEEE TIM 2020), reviewed in the assumptions audit, establish
method feasibility; usable ROS 2 code, rate and accuracy on Homer remain unverified.
First check shared field of view, rigid mounting, camera intrinsics/extrinsics
and timing. If useful associations are absent, this method cannot simply be
enabled with the current mounting. Keep RF2O/ICP as comparison baselines.
A genuine fused estimator prototype may exceed the earlier 45–90 minute budget;
that budget covers initial feasibility work, not promised completion.

## Real-world implementation sequence

Budgets are initial work allocations, not completion guarantees. Assess each
result before the next stage. Estimator installation, motor calibration or
processing problems may require another session; time pressure does not turn
a failed result into a pass.

| Order | Initial budget | Work | Decision / deliverable |
| --- | --- | --- | --- |
| 1. Inspect and prepare | 15–30 minutes | Read current Pi services/configuration; verify estimator/TF ownership, scan axes/timing and rigid sensor mounting; prepare scan/RGB/TF/status recording | Identify demonstrated configuration issues and missing calibration; motor output remains blocked |
| 2. Obtain measured real data | 10–15 minutes with operator | Record a short operator-controlled movement of 20–30 cm and a small turn, using tape/video references and existing teleop | Raw sensor bag plus independent displacement/heading reference; no automatic distance stop based on RF2O |
| 3. Correct/select odometry | 45–90 minutes | Fix demonstrated frame/timing errors; check camera/range association; evaluate a fused-estimator implementation on the same bag, with RF2O/ICP as baselines | Select only a candidate passing accuracy and quality checks; one live odometry/TF owner |
| 4. Qualify drive and stops | 20–45 minutes with operator | Verify command polarity, actual speed response, braking, cancellation and Pi watchdog with bounded commands and independent measurements | Document a low-speed operating envelope and metric-command conversion; no reversal at completion |
| 5. Add Nav2 and RViz goals | 45–90 minutes | Connect accepted odometry, live SLAM, footprint, lidar local costmap, collision monitor, watchdog, goal tool and cancellation; check output with motors blocked | Reproducible launch and working bidirectional goals/actions/cancel transport |
| 6. Short navigation checks | 10–20 minutes with operator | One nearby free-space goal; one stationary obstacle detour; one occupied goal; cancellation | Independent endpoint measurement, no contact/unintended reverse, correct failure and stop behaviour |

First action is read-only Pi inspection and recording setup. The operator
participates in movement and clearance checks; this plan change alone does not
launch motors or reuse the previously stopped automatic-test script.

## Pose gate

Retain the LDS-01 driver and SLAM Toolbox. Compare corrected RF2O with
RTAB-Map ICP; neither is a predetermined winner. A proposed first criterion
is displacement error <=3 cm over a measured 30 cm movement, with heading
error <=3 degrees. Report input age, drift and tracking quality separately.
Do not fit a scale factor from the single failed trial or align trajectories
with a free scale that hides the error. Tape measures endpoint displacement;
use recorded observations for the motion path and turns where needed.

Camera: record and synchronize RGB, keep the head fixed and check calibration.
Evaluate calibrated range-constrained visual odometry as the desired fused
estimate. Any lidar-only interim result must be labelled accordingly. Select
the fused estimate only after independent accuracy and failure checks; the
literature supports feasibility, not a guaranteed fast installation.

Fresh messages and agreeing estimators are not independent proof of accuracy.
Validate quality checks against weak-geometry and bad-registration recordings.
If reliable tracking loss detection is unavailable, keep the operating envelope
limited and report the unresolved deployment blocker rather than promise
arbitrary-room navigation.

## Motor and obstacle gates

The firmware's command-to-PWM scales do not establish real m/s or rad/s.
Qualify a measured conversion and response across the tested battery/load
conditions. Account for deadband, unequal sides, acceleration and braking.
Use a watchdog local to the Pi so a laptop failure produces zero output.
Teleop takeover and cancellation must work before an autonomous goal.

Fault injection, estimator stalls and link-loss tests first run with motor
output blocked. Verify zero commands and final command authority. Then perform
bounded operator-observed stopping checks; report residual travel separately
from software response time. A proposed 250 ms software reaction target remains
an engineering target, not a measured pass or physical stopping guarantee.

Local obstacles come directly from live scans, independent of the three-second
SLAM occupancy publication. Set footprint, inflation and stop clearance from
real dimensions, sensor age and measured stopping distance. Unknown cells remain
blocked for planning; initial goals lie in discovered free space. Preserve a
single final command path with collision handling and watchdog gates.
Disable automatic backup and recovery spins. Planned turns remain allowed.

## MVP acceptance

- Empty-map startup; RViz displays live scans, pose, map, route and goal status.
- One short free-space goal ends within a proposed 10 cm and 10 degrees of an
  independent physical reference; no unintended reversal at completion.
- One stationary obstacle with a reachable detour is avoided without contact.
- Occupied/unreachable goal fails cleanly; cancellation and takeover stop motion.
- Sensor/pose/link loss produces zero output; stale map rendering alone does not
  determine obstacle response.
- All results include actual measurements, revision/configuration and failures.

REQUIRES_HARDWARE_TEST. Initial tests use a level bounded area, stowed arm and
obstacles intersecting the lidar plane. Low objects, overhangs and drop-offs are
outside this sensor coverage; the demonstration does not qualify all obstacles.

## Scope and evidence

Defer simulation, automatic exploration, object detection/pickup, tags, ToF,
broad parameter sweeps and extra estimator alternatives. Fusion feasibility
and an implementation assessment are now part of the odometry task. Do not assume encoders.
Reuse the [peer-reviewed audit](slam-navigation-peer-reviewed-assumptions-2026-10-04.md)
and [existing physical evidence](slam-accuracy-2026-10-04.md). Literature supports
method selection; measured real data must establish this robot's performance.

The [full plan](live-slam-rviz-goal-navigation-plan.md) retains earlier simulator
work as historical/deferred material. This real-world sequence governs current
execution. Save launch commands, calibration provenance, bags, independent
references, stop results and rollback. A failed pose or stopping gate sends work
back to that component, not onward to longer movements.
