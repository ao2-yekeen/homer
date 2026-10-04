# Section 1 MVP implementation — 2026-10-02

Status: **software deployed; physical acceptance incomplete**.
The superseded MVP checklist and roadmap have been removed. This document
preserves implementation and physical-test evidence.

## Implemented

- Retained the temporary ROBOTIS LDS-01 driver and the existing RF2O/SLAM stack.
- Used `geometry:=urdf` at the operator's explicit request: scan origin
  `(0.090, 0.075, 0.093)` m and zero roll/pitch/yaw. No new mounting measurement
  is claimed, and the override template has not been marked physically verified.
- Added `ros2 run homer_slam mvp_check OUTPUT.json`: read-only live health,
  reference-distance, stationary-drift and movement/return checks. It refuses
  to overwrite evidence and treats unconfirmed physical trials as unknown.
- Stationary acceptance checks maximum excursion, so moving away and returning
  cannot masquerade as low drift. Return trials require observed movement.
- Enabled live scan points in the RViz configuration and reopened the existing
  read-only SSH viewer. A screenshot shows map, model, scan points and
  **Global Status: OK**.
- No drive publisher was added and no robot/arm motion was commanded.

## Live and reload evidence

The original live run found one scan, odometry and map publisher; approximately
4.93 scans/sec and at least 211 valid returns per scan. TF validation initially
needed correction for SLAM Toolbox's intentional 0.2-second future map stamp.
A later check passed every live gate and recorded fresh transforms.

Saved occupancy image/YAML, `.posegraph`, `.data`, actual parameters and version
information to Pi `~/homer-maps/section-1-mvp-20261002`. Mapping was stopped
before a single temporary localization stack loaded `map.posegraph`/`map.data`.

The first reload probe included lifecycle startup and correctly failed missing
publisher/TF gates. After activation, the 15-second reload check passed:

- 75 scans, 5.00 scans/sec, minimum 211 valid returns.
- 75 valid odometry poses; one publisher each for scan, odometry and map.
- Fresh timestamps/TF and uninterrupted measured streams.
- Map 71 × 83 cells, 0.05 m resolution, 767 known cells.

The checker now waits up to 30 seconds for startup readiness after discovery;
this does not excuse dropouts during the measured trial. Localization was
stopped and mapping restarted afterward; the current map has a new origin.
This establishes technical reload, not surveyed physical map alignment.

## Stationary and physical checks

The operator confirmed the robot was stationary on an external power supply
and authorized one 60-second trial. Recorded 296 scans/odometry poses at
approximately 4.93 Hz, minimum 208 valid returns. Maximum translation was
0.0111 m; endpoint yaw change was 3.45° and maximum yaw excursion was 3.71°.
**Stationary tracking failed the 3° rotational threshold.** Other check gates
are retained in `stationary-20261002.json`; this trial is not a pass.

The operator subsequently requested much shorter tests. Stop further physical
test rounds; quick live checks default to 10 seconds. Full 60-second drift
qualification remains pending correction of the measured rotation drift.
Movement must wait until the operator switches to battery power. No battery
movement is requested in this handoff and no movement command was issued.

Pending operator-dependent checks:

- Verify mounting matches the selected URDF and front/left/right scan alignment.
- Supply a physically measured obstacle distance for the ≤10 cm reference test.
- Perform the short movement and physical return to a marked start, on battery.
- Verify saved-map wall alignment physically.

## Verification and operational state

- Twelve hardware-independent geometry/network/tracking tests passed.
- Python compilation and whitespace checks passed.
- Pi package rebuilt successfully with the new `mvp_check` executable.
- Driver, serial bridge and teleoperation remain active. Mapping is running
  but remains disabled for boot. The temporary localization unit is stopped.
- The live viewer is `homer-lds01-live-view`; stop only that viewer with
  `docker stop homer-lds01-live-view`. Relaunch commands are in the checklist.
- Existing driver/range rollback remains in the weekend handover.

Pi evidence: `~/homer-evidence/section-1-mvp/`.
Workstation copy: `~/.local/state/homer-slam/section-1-mvp-20261002/`, containing
JSON checks, the map export, and `rviz.png`. Intermediate failed probes are
retained; they are not acceptance evidence.

Ten-minute qualification is deferred. A localization-startup temperature spot
check was 81.8°C; this is not a thermal pass. Autonomous navigation/exploration
remain disabled until their performance, communication and stopping gates pass.
