# Time-boxed MVP: live SLAM and RViz goal navigation

Date: 2026-10-04. Status: planned, not implemented or qualified.

## Outcome

Start with an empty map. Select a reachable destination in RViz. Homer plans
a route, avoids a lidar-visible obstacle, and stops at the goal. Occupied or
unreachable goals fail cleanly. Cancellation and invalid inputs stop commands.
First demonstrate this in isolated simulation, then qualify it on the robot.

The known 50 cm actual / approximately 9 cm estimated failure is established.
The first task is to correct or replace its failed pose source, not repeat the
same automatic tape test. Map display refresh is secondary.

## Schedule and decisions

Budgets below are work allocation targets, not completion guarantees. Aim for
one 4–6 hour software session, assuming dependencies install cleanly. Pause at
each deliverable to assess the next stage. Robot qualification is a separate
session whose duration depends on measured motor response and stopping.

| Order | Budget | Concrete work | Required result / decision |
| --- | --- | --- | --- |
| 1. Evidence and compatibility | 30 minutes | Inspect existing failure logs, mounting axes/TF, scan stamps and firmware commands; inventory raw recordings and available ROS/Gazebo dependencies | List demonstrated faults, missing data and what can be replayed; do not infer a root cause from pose JSON |
| 2. Minimum simulator | 90–120 minutes | One room, Homer geometry, LDS-01 scan timing/range, known 30 cm and turn trajectories, isolated ROS clock/network; bridge real estimator inputs and record independent truth | Moving sensor/estimator run with scored pose; no perfect simulator odometry supplied to navigation |
| 3. Select pose source | 60–90 minutes | Correct demonstrated input/config errors; compare RF2O and RTAB-Map ICP on identical nominal and weak-geometry inputs | Select an estimator only if movement accuracy and tracking-quality checks pass; neither is predetermined to win |
| 4. Goal navigation and actuator boundary | 60–90 minutes | Retain SLAM Toolbox; configure Nav2, footprint, direct-scan local obstacles, RViz goal/cancel, watchdog; simulate bounded actuator response and actual command conversion | Clear-space goal and obstacle detour; explicitly report unknown real PWM-to-speed calibration |
| 5. Short regression suite and handover | 30–45 minutes | Run blocked-goal, cancellation, scan/pose loss and injected underestimation/reversal cases; save commands, configs and reports | No contacts or unintended reverse; stop/failure behaviour passes; document limits and rollback |

These budgets prioritize a small prototype. If simulator/adapter setup exceeds
its budget, use the next session to finish that dependency rather than hide it
with ideal velocity/ground-truth feedback. A simpler sensor-only simulator is
useful progress but does not complete goal-navigation simulation.

## Scope kept small

- Keep the working LDS-01 driver and SLAM Toolbox. Compare just one alternative
  to RF2O. Use one active odometry/TF publisher.
- Initial goals lie in discovered free space; unknown cells remain blocked.
  No saved-map prerequisite and no automatic frontier exploration.
- Record RGB if available; keep the camera head fixed. Camera fusion is not a
  required new subsystem if corrected lidar odometry passes in the tested room.
- Do not spend this session implementing custom monocular/lidar optimization,
  object detection, grasping, ToF, tags or broad parameter sweeps.
- Preserve map publication at three seconds unless a measured planning issue
  requires changing it. Local obstacle handling consumes live scans directly.
- Keep automatic reverse/recovery spins disabled. Planned turns remain allowed.

## First decision gate: failed pose estimation

Use the same input sequences and independent reference for both candidates.
Initial proposed targets: 30 cm endpoint translation error <=3 cm and heading
error <=3 degrees; report stationary drift and latency separately. Include a
plain/repetitive geometry case, not just one favourable room trajectory.
No scale fitting may erase underestimation. Quality checks must address weak
constraints and fresh-but-wrong outputs; timestamps alone are insufficient.

If neither candidate passes, the fast hardware MVP is blocked by odometry.
Finish an evidence report and a simulation demonstration clearly labelled
unqualified for hardware. Next work is a calibrated camera/range-constrained
estimator prototype, guided by the peer-reviewed audit. It is not justified
to promise that installing RTAB-Map or adding optical flow solves the failure.

## Navigation acceptance

Run short cases rather than long room tours:

1. Empty map, clear known-free goal: endpoint within 10 cm and 10 degrees of
   independent simulator truth, with no unintended reverse after arrival.
2. One obstacle with a reachable detour: no contact and clearance consistent
   with the stopping envelope.
3. Occupied/unreachable goal: reject or abort with a reason and zero output.
4. New obstruction, cancel, stale/lost scan or pose, and command/link failure:
   stop promptly; replan only while tracking and inputs remain valid.
5. Inject the observed underestimation and end-reversal regressions. Distinguish
   injected supervisor tests from identifying the cause of the real failure.

Keep the earlier proposed 250 ms software reaction target; independently grade
actual wheel-stop time/residual travel in the motor model. Three core goal
arrangements suffice for this small prototype, with failures preserved.
These are provisional engineering targets, not literature-certified guarantees.

## Separate robot session

Software completion does not resume the previously stopped automatic tests.
Before physical goals:

- Run real sensor inputs with motor output blocked; check accepted pose source,
  processing age, resource load and real-data replay evidence where available.
- Qualify bidirectional goal/action/cancel transport and Pi-local watchdog; the
  existing read-only viewer does not carry robot commands.
- Coordinate independent direction, speed-response and braking measurements.
  The current PWM scales do not establish metric speed; conversion must pass
  under the tested battery/load conditions before Nav2 commands can drive it.
- Then qualify one short free-space goal and one stationary obstacle detour,
  using an independent distance reference and immediate stop/teleop takeover.

REQUIRES_HARDWARE_TEST. This session is limited to a level bounded area,
laser-visible obstacles, stowed arm and the measured operating envelope.
A failed accuracy or stopping gate sends work back to its corresponding stage.

## Evidence and method discipline

Reuse the [peer-reviewed assumptions audit](slam-navigation-peer-reviewed-assumptions-2026-10-04.md)
rather than restart a broad review. Independent studies support feasibility;
Homer-specific performance requires measured evidence. Paper, repository and
arXiv copy of one study count once. Uncorroborated claims remain provisional.

The [full implementation plan](live-slam-rviz-goal-navigation-plan.md) supplies
detailed isolation, estimator, control and deployment requirements. This
schedule narrows the first implementation scope without waiving those gates.

Record exact revisions, simulator assumptions, calibration provenance, input
bags/worlds, true/estimated trajectories, commands, stop latency and failures.
First next action: **audit evidence and prepare the minimal moving-sensor
simulation**, with no real motor connection.
