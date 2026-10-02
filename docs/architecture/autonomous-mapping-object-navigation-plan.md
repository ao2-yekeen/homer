# Autonomous mapping and language-directed object approach

Goal: Homer autonomously explores a bounded room, saves its map, and later
responds to requests such as “go to the red chair” by stopping approximately
60 cm away facing the object.

Implement one section at a time. Record evidence and failures before advancing.
Keep unfinished motion components disabled and preserve teleoperation.

| Section | Deliverable | Acceptance | Status |
| --- | --- | --- | --- |
| 1. Lidar SLAM MVP | Live scans, odometry, map, save/reload | [Section 1 checklist](slam-section-1-validation-plan.md) | Software implemented; physical checks pending |
| 2. Safe individual goals | Nav2, conservative speeds, collision monitoring, Pi command watchdog | Reach nearby goals; stop on cancellation, takeover, sensor/network loss | Not started |
| 3. Autonomous exploration | Reachable frontier selection and Nav2 goals | Explore without manually selected goals; skip unreachable frontiers | Not started |
| 4. Completion and reuse | Complete/partial/failure result; occupancy and pose-graph export | Three consistent autonomous mapping runs; restart localization | Not started |
| 5. RGB recognition | Common object classes and optional colours | Reliable detections, missing-object and ambiguity results | Not started |
| 6. Object positioning | Calibrated RGB/ToF association and map-frame position | Horizontal error ≤10 cm at five measured positions; reject unreliable depth | Not started |
| 7. Object approach | Reachable pose on robot-facing side of object | 8/10 trials at 60 ±15 cm front clearance, facing within 15°, no contact | Not started |
| 8. Natural language | Typed supported class/colour requests | Correct object selection; ambiguity handling; stop/cancel | Not started |

## Implementation defaults

- Retain LDS-01, RF2O, and SLAM Toolbox. One owner per estimator transform.
- Pi runs drivers and control; laptop runs heavier recognition/navigation.
- Section 2 starts at 0.10 m/s and 0.25 rad/s; no automatic reverse/recovery spins.
- Exploration stays within a configured level-room boundary, excluding stairs
  and drop-offs. The arm remains stowed.
- Section 3 selects the frontier with the shortest reachable path. Refresh at
  five seconds; stop a goal after 30 seconds without progress or 120 seconds
  total; skip a frontier after two failed attempts.
- Full qualification before exploration includes three movement loops,
  repeated reloads, ten-minute processing checks, bidirectional ROS transport,
  clocks, physical stopping distance, and fault tests.
- ToF's calibrated RGB overlap limits object positioning, not lidar exploration.
  Missing depth means refusal to approach; no guessed monocular distance.
- Initially approach stationary objects currently visible. Search for unseen
  objects, speech, semantic object-front estimation, grasping, and full 3D
  mapping are later work.

## Existing evidence

See the [LDS-01 weekend handover](slam-lds01-weekend-2026-10-02.md) for driver
location, override, rollback, and historical bench results. Bench operation
does not establish mounted-robot accuracy or autonomous navigation readiness.
