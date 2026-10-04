# Unknown-environment object search and pickup

## Goal clarified on 2026-10-04

Given a request such as “pick up the red cup”, Homer starts in an unknown
bounded environment, builds a map while moving, searches for the requested
object, approaches a reachable manipulation position, grasps it, and verifies
that the pickup succeeded. No previously saved map, taught route, or known
object location is required. Map export is optional evidence, not a condition
for starting the task. Complete room coverage is not required before pickup.

Use the available RGB camera and temporary LDS-01 2D lidar. No tags or
additional sensors are prerequisites. Metric motion and object localisation
must be validated; RGB pixels alone are not metric 3D coordinates, and the
lidar scan plane may not intersect an object on a raised surface. The method
for metric object localisation with these sensors remains to be selected and
validated. Do not promise arbitrary-object pickup until this works.

Implement one section at a time. Test motion estimation and control in
simulation and recorded-data replay before hardware deployment. Preserve
teleoperation. Remote automatic movement tests remain stopped after the
50 cm actual / approximately 9 cm estimated failure.

## Staged deliverables

| Section | Deliverable | Acceptance | Status |
| --- | --- | --- | --- |
| 1. Reliable motion estimation and online SLAM | Camera/lidar streams, validated pose, live map in RViz; small unknown-room simulation | Known simulated movements agree with estimated motion; reproduce and reject the observed underestimation failure; replay real sensor data before deployment | Existing lidar software runs; physical movement accuracy failed; camera-assisted method unvalidated |
| 2. Safe local movement | Nearby goals on the map being built, obstacle monitoring, command watchdog and cancellation | Reach a nearby goal and stop correctly on takeover, cancellation or stale data | Not started; Nav2 work paused by user |
| 3. Object detection | Requested class/colour recognition from RGB | Detect an initially visible test object reliably; handle ambiguity and missing detections | Not qualified |
| 4. Search while mapping | Explore reachable unknown space and inspect camera views for the target | Find an initially unseen target without manual waypoints or a saved map; stop searching and transition to approach when found | Not started |
| 5. Metric target localisation and approach | Target position with uncertainty; collision-free base pose inside arm reach | Validate localisation against measured positions; reacquire after movement; refuse uncertain or unreachable targets | Not started |
| 6. Pickup and verification | Feasible pre-grasp/grasp/lift sequence, camera visibility recovery and outcome verification | Lift a reachable test object; distinguish verified success, failure and uncertainty | Existing arm work requires baseline qualification |
| 7. Integrated task | Request → online mapping/search → approach → grasp → verify | Start with an empty map and unknown target location; find and pick up one test object autonomously, or report an evidenced failure | Not started |

## First end-to-end MVP

Use one bounded level room and one supported small object on a reachable
raised surface. Vary the robot start and object location between trials.
Start each trial with no loaded map or object coordinates. Search and map
concurrently, approach once the target is found, and verify pickup by lifting.
Log the trajectory, detections, localisation confidence, selected grasp and
outcome. Expand object variety and environment complexity after this works.

The current arm cannot reach the floor. Initial targets must be on a raised
surface within its calibrated workspace. A navigation standoff such as 60 cm
may be an intermediate observation pose; final base placement must satisfy
arm reach and collision clearance rather than a fixed distance alone.

## Implementation constraints

- Retain working sensor drivers. RF2O and SLAM Toolbox are the existing
  baseline, not a validated final motion estimator. One owner per transform.
- Pi runs drivers and low-level control; laptop can run heavier processing.
- No automatic reverse/recovery spins. Validate drive direction, command
  scaling, stopping and estimator failure handling before autonomous motion.
- Explore within a level-room boundary excluding stairs and drop-offs;
  keep the arm stowed during travel.
- Use bounded search and recovery attempts. Report target-not-found,
  unreachable target, uncertain localisation or failed pickup truthfully.
- Map save/reload checks remain useful diagnostics and optional reuse;
  completing them does not establish unknown-environment task readiness.
- ToF may be evaluated later if available; it is not assumed in this plan.
- General grasping and fixed-versus-active-camera research retain the
  calibrated arm limits, reachability and verification requirements in the
  workspace engineering plan.

## Evidence and supporting plans

- [Section 1 checklist](slam-section-1-validation-plan.md): earlier lidar MVP;
  save/reload is a diagnostic rather than the final task workflow.
- [Physical accuracy results](slam-accuracy-2026-10-04.md).
- [Simulation-first validation plan](slam-simulation-validation-plan.md).
- [Camera–lidar literature review](slam-camera-fusion-literature-review-2026-10-04.md):
  earlier conditional RGB-D proposals do not supersede the current RGB/lidar
  constraint or establish a working camera-assisted estimator.
- [LDS-01 weekend handover](slam-lds01-weekend-2026-10-02.md): driver, override
  and rollback. Bench operation does not qualify autonomous movement.
