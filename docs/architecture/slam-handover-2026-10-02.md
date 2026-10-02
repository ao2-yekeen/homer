# Encoder-free SLAM handover — 2026-10-02

## Start here

**Later update:** the temporary TurtleBot3 LDS-01 now supplies valid scans,
and real bench mapping/save/reload works. See
[the weekend continuation](slam-lds01-weekend-2026-10-02.md) for the current
Pi configuration and remaining acceptance gaps. The old RPLIDAR cable problem
below is historical and has not been diagnosed or repaired.

Continue in `/home/ao2-yekeen/github/esp32_robot_bt`, branch
`implement-hotspot-slam`. **Software is implemented, but real mapping is not
working or accepted. Immediate priority: investigate the operator's split
LiDAR power/data cable and recover valid distances.**

Latest operator message: “i splited data and power cable not sure which bit got
corupted”. The operator already confirmed the LiDAR is spinning and uncovered.
Which cable was split, its connections, and the faulty part remain unknown.
The last response asked whether it was the LiDAR USB cable and requested a
photo showing the split and connections. No photo or wiring details have
arrived. Do not ask the operator to repeat answers already provided.

The operator became frustrated with lengthy replay/container investigations
while the real scans were unusable. Focus on this physical/data problem first;
keep updates short and relevant. Do not diagnose a damaged component without
evidence, infer pinouts from wire colours, or recommend powered wiring changes.
Check manufacturer documentation and compatibility before electrical advice.

Read `/home/ao2-yekeen/AGENTS.md` and `/home/ao2-yekeen/PROJECT_PLAN.md` in full
before changing code. The current explicitly authorized task is this SLAM
implementation, rather than resuming unrelated grasping work from that plan.
Do not merge while physical acceptance remains outstanding.

## Authorized design and operator preferences

- Disable encoders with minimal firmware changes, retaining code and the flag.
- ESP32 keeps drive control and command timeout. Existing services own LiDAR
  and serial ports; new SLAM components never send drive commands.
- Pi 4 initially runs sensor drivers, RF2O, and asynchronous SLAM Toolbox.
  Laptop runs Jazzy Docker, lightweight RViz, recording, and replay.
- Use the existing laptop hotspot as the primary ROS network. Tailscale is a
  separately selected fallback; never switch automatically or mix peers.
- Use the robot's existing URDF in RViz. This explicit request supersedes the
  original plan's omission of a robot model. Canonical URDF stays unchanged.
- Deliver usable LiDAR mapping first. RGB and ToF integration follows verified
  calibration. Autonomous navigation is outside this milestone.
- Preserve teleoperation, motor timeout, and an immediate stop path. No drive
  commands were sent during the reported software checks.

Full acceptance plan: `docs/architecture/slam-without-encoders.md`.
Operations: `pi/homer_slam/README.md`.
Evidence summary: `docs/architecture/slam-validation-2026-10-01.md`.

## Git and recovery precautions

- Base: `origin/main` at `dc666cc` when work began.
- Encoder-disable source commit `d3d602d` was cherry-picked as `decd8a9`.
- Remote: `https://github.com/ao2-yekeen/homer.git`.
- The substantial SLAM implementation is **uncommitted in this checkout**.
  Many files are staged; several have newer unstaged modifications on top of
  the staged versions. `replay-container.sh` is untracked at handover creation.
  Preserve both index and working-tree changes. Do not reset, clean, overwrite
  them from the remote, or commit only the older staged versions accidentally.
- This handover is committed separately and pushed on the feature branch.
  **A fresh remote clone does not contain the uncommitted SLAM implementation.**
  Resume in this workspace or transfer its complete working tree first.
- No implementation PR or completed code review is recorded. After final
  proportionate checks, commit the complete intended implementation, push,
  verify remote contents, and open it for review. Hardware gaps must be stated.

## Implemented software

`pi/homer_slam` is an `ament_python` package with configuration, launch files,
ROS nodes, operational commands, scripts, tests, and a Dockerfile.

Entrypoints:

```text
ros2 launch homer_slam odometry.launch.py
ros2 launch homer_slam mapping.launch.py
ros2 launch homer_slam localization.launch.py map_file:=<posegraph-prefix>
ros2 launch homer_slam view.launch.py
```

Processing launches also require actual `min_range` and `max_range`; the
observed driver metadata was 0.15 and 12 metres. Recheck after hardware changes.

- RF2O pinned to `b38c68e46387b98845ecbfeb6660292f967a00d3` in
  `dependencies.repos`, with `patches/rf2o-startup.patch`. Fixes initialization,
  quaternion, scan state/size/timestamps, missing sensor TF, failed-solve
  publication, logging, and scans with fewer than 16 valid in-range returns.
- TF ownership: SLAM owns `map -> odom`; RF2O owns `odom -> base_link`;
  static geometry owns `base_link -> laser`. RF2O timer is 20 Hz.
- SLAM settings: asynchronous, 0.05 m cells, 3-second map updates, 0.2-second
  minimum processing interval, 0.10 m / 0.10 rad motion thresholds.
- Canonical model:
  `simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf`.
  Derived laser origin is (0.090, 0.075, 0.093) m; base outline is
  0.318 × 0.350 m. Optical scan origin/orientation still need physical checks.
  Measured override YAML requires `verified: true` and its measurement source.
- RViz: fixed frame `map`, TopDownOrtho, 10 FPS, occupancy map, robot arrow,
  outline, and primitive URDF. Model uses separate `display/` frames and fixed
  illustrative joint positions, not live arm/neck/wheel feedback.
- Markers run at 5 Hz with finite lifetimes. Health and viewer monitors report
  stale data, missing TF, invalid ranges, and estimator/authority issues.
  TF buffers now use each node's clock for simulated-time replay.
- Mapping/localization host lease and discovery preflight reject overlapping
  estimators. Estimator exit shuts down the processing launch.
- Save command requires a nonempty map with known cells, both occupancy and
  posegraph exports, and metadata; refuses overwrite. Parameter dumps use
  explicit discovery timeouts and bypass the ROS CLI daemon.
- `baseline.py` measures CPU, memory, temperature/throttling, scan rate,
  odometry age, and gaps. Passing thresholds alone do not establish tracking
  validity. Recordings include scans, odometry, map, TF, diagnostics, and drive
  status; cameras are opt-in.
- Serial bridge defaults `publish_encoder_odometry=false`. Firmware keeps its
  encoder configuration flag false. Drive/status interfaces remain intact.
- Reversible Pi network drop-ins, rollback, clock/firewall/SSH helpers,
  disabled-by-default mapping service, and WSLg Docker viewer are provided.
- New `scripts/replay-container.sh` uses an isolated, network-disabled
  container and bag clock for inspection. Shell syntax was checked; its new
  end-to-end wrapper has not been runtime-qualified.

## Confirmed checks and limitations

- Firmware built on the PC, flashed using `deploy.sh esp32dev`, with all four
  flashed segments verified by readback. Drive/link/status returned.
- Six geometry/network, three serial protocol, five teleop, and two ROS serial
  bridge tests passed earlier. Encoder `/odom` was absent by default.
- Updated RF2O and package build succeeded on the Pi after the invalid-range
  guard was added.
- **Latest updated smoke tests passed twice: wall time and simulated time.**
  Log `ros-validity-smoke.log` contains two PASS lines and both map-save results.
  They cover invalid scans, recovery to valid synthetic scans, no odometry
  refresh on invalid scans, map/TF/URDF markers, duplicate launch refusal,
  empty map/overwrite refusal, stale diagnostics, and localization reload.
  These synthetic tests do not establish physical accuracy.
- Prior Docker RViz started through WSLg under UID 1000 with OpenGL 4.5.
  Latest image build completed as `91841f43ab48`
  (`sha256:91841f43ab486205c10df636f25b568483d97df86fcba32ff7bda01df0ea5727`).
  Its build context predates some final Python/save/replay-wrapper changes;
  rebuild before claiming the container matches the working tree.
- At handover inspection, `docker ps` listed **no running containers**.
  Earlier `homer-rviz-review` was a looping recorded-data viewer, not live SLAM.
  Do not assume RViz is currently open or that a visible robot proves mapping.
- Earlier replay best-effort `/scan` subscribers failed to receive messages in
  one container setup. Cause was not established; avoid presenting guesses as
  facts or pursuing this before recovering the real sensor data.

The validation document predates the final two updated guard-test PASS results;
update its pending-guard wording when finalizing software documentation.

## Actual LiDAR/map evidence — blocking

- A real 21.1-second recording had 137 scans, **every one with zero valid
  distances**. All seven map messages had width 0 and no occupancy cells.
- Ten fresh live scans also had zero valid distances. Restarting only the
  LiDAR service produced nine empty scans and one scan with one return at
  0.465 m. This is still unusable for the estimator.
- Driver reported RPLIDAR A1, firmware 1.29, hardware revision 7, health OK.
  Spinning and health OK do not prove sensor power/data integrity.
- Pi export directory `~/homer-maps/live-1790933702` contains a 1×1 placeholder
  occupancy image. **Not a usable map; do not use it for localization.**
- Earlier claims implying a usable map preview were corrected. The screenshot
  showed the URDF and stale-transform warning; it did not prove SLAM worked.
- User now reports splitting power/data cable. That is a possible cause,
  **not a confirmed diagnosis**. Identify wiring before proposing repair.

## Networking, clocks, and Pi state

- Pi hostname alias `pi`: explicit hotspot address `192.168.137.250`, via a
  Windows TCP SSH proxy. `pi-tailscale`: explicit fallback
  `robot-b@100.66.192.19`. User authorized this alias arrangement; generic
  AGENTS text describing the old alias is stale for this session.
- Windows hotspot is `192.168.137.1`. WSL uses mirrored networking but the
  hotspot adapter is absent. Observed WSL addresses were `10.142.22.6` and
  `172.16.16.173`; traffic to the Pi routed through `10.142.22.1` and failed.
  Docker host networking does not bypass this boundary. The network generator
  correctly rejects the unreachable hotspot source. Do not invent interface
  addresses or treat a successful TCP SSH proxy as proof of DDS UDP access.
- Clock check over hotspot SSH measured about -196 ms offset, uncertainty
  2.6 ms; required absolute offset plus uncertainty is below 50 ms. Changing
  WSL's clock alone was transient. Windows admin synchronization is pending.
- Pi production ROS services use Cyclone DDS domain 0, hotspot `wlan0`,
  explicit hotspot/self peers. Network drop-in backup: `20261001T221526Z`.
- Separately selected Tailscale transport passed bidirectional DDS ping/ack
  in isolated domain 72. This does not qualify full live fallback SLAM.
- Existing LiDAR, serial bridge, teleop, RGB and ToF services retained ownership.
  Production RF2O/SLAM were stopped after measurement; `homer-slam.service`
  was installed inactive and disabled. Recheck live state before restarting.
- Repository on Pi: `~/mobile-robot`; ROS workspace: `~/homer_slam_ws`.
  Latest smoke tests used isolated domain 71 and `/tmp/homer-guard-dds.xml`.
  Do not reuse that temporary profile for production services.
- Pi package updates fixed mixed-age Jazzy TF serialization dependencies.
  The install script includes the needed packages.

## Compute and camera limitations

- A 600.01-second run measured 6.42 scans/s (baseline 6.51), average total CPU
  59.67%, and p95 odometry age 205.3 ms. Age and thermal gates failed.
  Because real scans were invalid and the map empty, **this is not a qualified
  representative SLAM baseline**. Another earlier baseline overlapped builds.
- If a valid representative run fails gates, stop onboard estimators before
  moving the same processing stack to the laptop. Never run duplicates.
- ToF stream: `/tof_camera/depth/image_raw`, 240×180 `32FC1`, millimetres,
  processing-time timestamps, frame `tof_camera`. No calibration topics found.
  RGB CameraInfo/extrinsics and camera calibration remain later work.

## Evidence locations and useful commands

Local evidence root (outside Git):
`/home/ao2-yekeen/.local/state/homer-slam/validation-20261002`.

Important entries: `replay-1790933702/`, `inspect-bag.py`,
`pi-scan-validity.log`, `pi-scan-restart.log`, `scan-validity.py`, `live.json`,
`pi-validity-build.log`, `container-validity-build.log`,
`ros-validity-smoke.log`, and earlier RViz/replay logs.
Read-only live scan probe was copied to `/tmp/homer-scan-validity.py` on Pi.
Do not start a second driver or open the serial port while its service owns it.

```bash
git status --short
git diff
git diff --cached
tail -40 ~/.local/state/homer-slam/validation-20261002/ros-validity-smoke.log
PYTHONPATH=pi/homer_slam python3 -m unittest discover -s pi/homer_slam/test -p 'test_core.py'
```

For ROS smoke tests, source Jazzy and the built workspace on the Pi; consult
the operations guide/test code before running. Use an isolated DDS domain,
never the production domain, and avoid measuring compute during compilation.
No need to rerun passing tests merely to fill time.

## Next steps and completion gates

1. Resolve the unanswered cable/photo question. Establish the actual cable,
   wiring, power source and interface; use verified documentation before
   suggesting electrical changes. Record findings without assuming damage.
2. After operator correction, sample real `/scan` validity/ranges using the
   existing service. Require substantial valid returns before SLAM testing.
3. Verify physical LiDAR scan orientation/origin and footprint; then launch a
   single estimator stack and require nonempty, plausible occupancy cells.
4. Restore a real RViz demonstration with the URDF; solve hotspot WSL routing
   and clock gates. Fallback selection must be explicit and documented.
5. Finalize outstanding software/doc verification, rebuild current Docker
   context, and commit/push/review the full working-tree implementation.
6. Complete `REQUIRES_HARDWARE_TEST`: hotspot-only ROS; <50 ms clock offset;
   stationary drift <0.05 m / 3° over 60 s; three slow room loops; dimensions
   within 0.10 m and return pose within 0.20 m / 5°; save/restart/reload and
   three localization traversals; representative 10-minute compute gates;
   viewer/network/scan/estimator/save failure checks; explicit fallback check.

Do not mark the task complete, merge, claim usable map save/reload, or claim
hardware accuracy until those results are recorded. No autonomous navigation
or automatic robot movement is authorized by this handover.
