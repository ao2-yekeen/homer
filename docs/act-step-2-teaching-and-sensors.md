# Step 2: USB wrist camera and hand-guided teaching

Status: planning only. Hardware acceptance is `REQUIRES_HARDWARE_TEST`.
No dates or deadlines are assigned to these steps.

## Outcome

Make Homer ready to collect synchronized USB wrist-camera images and measured
six-joint positions while the operator physically guides the arm. No leader
arm or gamepad is required. Train a fresh ACT policy with the official LeRobot
implementation and ImageNet-pretrained ResNet-18, using Homer-derived state
and action statistics. The previously tested community weights are an optional
comparison, not a dependency of data collection.

The camera is the user's USB wrist camera, **not the Logitech camera**. Its
model, stable Linux device path and negotiated modes remain to be discovered.
Use the established raised platform and stationary base. Do not change arm
limits, base control or the existing Logitech stream for this task.

## What exists and what needs changing

| Existing component | Finding | Planned change |
| --- | --- | --- |
| `pi/soarm_ros_bridge.py` | Owns `/dev/robot-soarm`; six sequential reads; state publication at 1 Hz with no timestamp | Add measured timestamped snapshots at a configurable, tested rate |
| Legacy delta command topic | Adds capped deltas to prior targets | Preserve ordinary jogging; reject it throughout teaching and recovery |
| Watchdog | Writes a hold command after command expiry | Make behavior mode-aware so it cannot write during hand guidance |
| Teaching interface | Absent | Add explicit entry, supported guidance, and supervised exit |
| Camera module | Existing module serves the fixed RGB camera | Add a separate USB wrist-camera publisher |
| Coordinates | Tick limits exist; URDF reference alignment remains unverified | Retain raw ticks and define a reversible policy-coordinate mapping |

Read the deployed SDK and controller configuration before changing hardware
behavior. A local SDK copy is not evidence of the installed version or servo
capabilities. No direct second process may own the servo serial port.

## Implementation sequence

### 1. Discover the USB camera and deployed servo interface

- On the Pi, enumerate V4L2 capture devices, USB identity and stable
  `/dev/v4l/by-id/` paths; distinguish capture nodes from metadata nodes.
- Identify the wrist camera by disconnect/reconnect or USB identity. Do not
  guess `/dev/video0` or substitute the Logitech feed.
- Inspect camera modes and negotiate an initial 640×480 stream. Request
  30 FPS only if supported, and record actual rate, encoding and exposure.
  Fresh ACT can use another resolution if chosen consistently before recording.
- Inspect the deployed SDK's position read, torque read/write and supported
  grouped-read methods. Confirm register meanings for the actual servo models;
  make no EEPROM or firmware changes.
- Identify all bridge command publishers and processes that could open the
  bus directly. The bridge must remain the sole bus owner.

Deliverable: discovery report with camera identity, negotiated mode, SDK version
or hash, servo models, and command-writer inventory. Missing facts stay unset.

### 2. Define the rig configuration and recording representation

Create `config/act/rig.example.yaml`; keep machine-specific settings in an
ignored local copy. Include camera path, resolution, requested FPS, actual
selected recording FPS, joint names and IDs, existing tick limits, timestamp
rules, teaching torque mask, measured gripper endpoints and validation status.
Do not invent unmeasured values or duplicate limits that can silently drift.

Use the existing order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll, gripper. Store raw ticks as the source of truth. A practical fresh
ACT representation is a reversible per-joint scaling of the documented tick
interval into [-1, 1], with separately measured gripper open/closed semantics.
This is a policy coordinate, not radians, physical aperture or a claim of
collision-free reach. Preserve exact tick limits and rounding rules in metadata.
Never wrap wrist-roll values automatically across the tick boundary.

Measured future positions will become action labels in step 3. Step 2 must
provide their timing and semantics; it does not label the same current state
as a command or pretend measured motion is a leader's commanded target.

Deliverable: example configuration and representation specification with
round-trip checks and explicit calibration revision.

### 3. Implement timestamped joint snapshots without motion

Keep `/soarm/state_ticks` for existing consumers. Add
`/soarm/state_snapshot` using `std_msgs/String` JSON initially, to avoid a new
ROS interface package. Include sequence number, joint names/IDs, raw ticks,
validity, mode, torque state, ROS read-start/read-end timestamps, monotonic
read duration and per-joint sample timing where sequential reads are used.
Do not put ticks into `sensor_msgs/JointState.position` as if they were radians.

Benchmark six-position reads first, then prototype 10, 20 and 30 Hz. Use a
supported grouped read only after verifying it on the installed hardware.
Keep load/current diagnostics at low rate so they do not starve position reads
or the stop path. Serialize all bus access and bound SDK read timeouts.
Incomplete reads must be invalid snapshots, never old positions republished
with new timestamps. Do not block the ROS event loop behind a stalled bus.

Deliverable: sustained-rate report including p50/p95/max read duration,
read failures, sampling skew, missed periods and stop-command responsiveness.

### 4. Add the USB wrist-camera publisher

Create `pi/wrist_camera.py` with configurable stable device path. Publish
`/wrist_camera/image_raw` with explicit encoding and frame ID, plus diagnostics
for actual frame rate, frame sequence, capture duration, read errors and reconnects.
Convert BGR to RGB exactly once in the learning input path. Keep preview and
recording colors consistent.

Capture on the same Pi as joint readback. Use ROS wall-clock timestamps for
association and monotonic time for durations. A timestamp after a blocking
OpenCV read is a host capture estimate, not a verified exposure timestamp;
document buffering and use a driver timestamp if available and validated.
Bound queues and discard stale frames. Disconnection marks the stream invalid;
reconnection starts a new segment, rather than silently joining an episode.

Inspect wrist view and USB cable clearance along the intended supported
pick/lift/place path. Confirm the object and gripper remain visible, exposure
is usable and the guiding hand does not dominate the view. Log limitations.

Deliverable: saved preview frames, stream diagnostic report and verified mount
identity, with no substitution of the fixed Logitech camera.

### 5. Implement a teaching state machine

Suggested states: `READ_ONLY`, `CONTROL`, `ENTERING_TEACH`, `TEACHING`,
`EXITING_TEACH`, and `FAULT`. Mode changes and all servo writes must share one
serialized authority. Publish mode and per-joint torque status; an unknown
torque state must never be presented as successful teaching entry.

Entry, with the arm physically supported:

1. Reject new motion commands and invalidate outstanding command sessions.
2. Cancel pending trajectories through the established stop path before
   changing torque; verify stationary feedback and fresh readback.
3. Release only the configured joints, verifying each operation. The vertical
   arm may sag when torque is disabled; support is necessary during this test.
4. Enter `TEACHING` only after confirmed torque state and valid joint feedback.
   The ordinary watchdog must not issue position holds in this mode.

During teaching, reject jog, policy and automatic hold writes. Continue joint
readback. Distinguish operator cancellation from bus failure. Guidance must
be practical and readback continuous; do not force motion against powered
servos. A fault blocks motion and requests operator support; it must not blindly
enable torque or release additional joints.

Exit, still supported:

1. Keep command acceptance blocked; get a fresh complete in-limit pose.
2. Verify how this SDK/hardware can set a goal equal to that measured pose
   while torque is disabled. Test this behavior before relying on it.
3. Stage those current-position targets and re-enable torque with conservative
   settings, verifying each joint. Never reuse pre-teaching targets.
4. Verify stable feedback and torque status. Return to `READ_ONLY`; starting
   ordinary control requires a fresh session. Reject queued pre-entry commands.

If goal staging while disabled cannot be verified, exit remains a supervised
hardware procedure; do not implement an assumed automatic re-enable sequence.
Legacy unstamped jog messages cannot identify old sessions, so use a separate
control-enable gate and drain/detach their subscription around transitions.

Deliverable: supported entry/exit test, command rejection evidence and
confirmation that no stale-target movement occurs. Hardware stop behavior
must be documented per mode: dropping torque is not a universal safe stop.

### 6. Resolve gripper teaching separately

First test whether the supported, torque-disabled gripper can be moved gently
and retains useful position readback. Measure its open and closed endpoints.
Do not assume tick direction from another robot's calibration.

If manual gripper guidance is impractical, use explicit open/close buttons
for the gripper alone while arm joints remain in teaching mode. This is a
proposed fallback, not a verified hardware capability. Require a separately
enabled gripper mode with bounded endpoints, slow motion, finger clearance
and logged commands plus measured feedback. Keep other motion writes blocked.
Do not use gamepad demonstrations or a scripted arm replay as substitutes.

Deliverable: one workable, documented gripper method for the pilot episodes.

### 7. Measure synchronization and select recording FPS

Build a command-free diagnostic subscriber for wrist frames and snapshots.
Collect a sustained five-minute sample with the intended camera settings,
readback, low-rate telemetry and preview running together. Save raw timestamps
and sequences, observed throughput, gaps, image/joint offset distributions,
read skew, CPU load and reconnects.

Start with a candidate recording grid of 20 Hz; select 10 or 30 Hz only from
measurements. This is dataset FPS, not the rate of fresh CPU ACT predictions.
The earlier 478 ms p95 model result does not prevent faster data capture, but
later execution must explicitly handle slower chunk updates and feedback delay.

Initial quality targets, to validate rather than claim as measured:

- Both streams sustain at least the selected grid rate under combined load.
- Pairing offset p95 is at most half one recording period; samples beyond
  one period are rejected, never filled with stale values.
- Sequential joint sampling skew p95 is at most half one period; lower the
  rate or use validated grouped reads if this is not achievable.
- Report every missing/invalid sample; target fewer than 1% rejected grid
  samples in the sustained test, with no unexplained long gaps.

A software pairing offset does not prove exposure synchronization. Inspect a
short visible supported motion to estimate buffering delay before pilot data.
Bracket frames with valid joint samples when interpolating and retain the raw
values. Do not interpolate across faults, reconnects or teaching-mode changes.

Deliverable: chosen FPS and freshness/association thresholds justified by the
saved report. Step 3 implements final episode alignment and future-action labels.

## Planned files and verification

| File | Scope |
| --- | --- |
| `config/act/rig.example.yaml` | Measured rig settings and unset hardware fields |
| `pi/soarm_ros_bridge.py` | Snapshot publication, serialized bus ownership and teaching transitions |
| `pi/act_learning/teaching.py` | Testable state machine independent of ROS/SDK |
| `pi/act_learning/representation.py` | Reversible policy coordinates and metadata |
| `pi/wrist_camera.py` | Independent USB wrist stream |
| `pi/act_learning/check_streams.py` | Command-free timing and stream report |
| `pi/tests/test_act_teaching.py` | Rejected writes, failed torque changes, stale targets and restart behavior |
| `pi/tests/test_act_representation.py` | Tick-coordinate round trips and gripper direction |
| `pi/tests/test_act_streams.py` | Missing joints, repeated/stale frames, clock changes and reconnects |

Every ROS entrypoint follows `pi/AGENTS.md`: initialize `ros_runtime` before
importing `rclpy`, use the deployed middleware configuration and upload the
runtime module with standalone tools. Mock tests cover bus failures and mode
transitions; physical tests cover actual torque behavior, supported guidance,
gripper operation, mount clearance and measured readback rate. Preserve and
verify the existing ordinary jog/watchdog behavior outside teaching.

## Gate to step 3

- [ ] USB wrist camera identified and usable through the intended motion.
- [ ] Complete timestamped measured joints available at the selected rate.
- [ ] Supported hand guidance works without conflicting motion writers.
- [ ] Gripper can be demonstrated with one tested method.
- [ ] Teaching exit does not move toward stale goals; faults block commands.
- [ ] Raw ticks, coordinate mapping, torque states and timing are documented.
- [ ] Combined stream timing report meets the selected acceptance thresholds.
- [ ] Physical evidence and software tests are recorded separately.

Step 2 ends at sensor/teaching readiness. Complete episode recording, dataset
export, training notebooks and policy execution belong to subsequent steps.
