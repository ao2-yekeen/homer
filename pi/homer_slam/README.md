# Homer LiDAR SLAM and RViz

ROS 2 Jazzy, RF2O laser odometry, and SLAM Toolbox. The Pi runs processing;
RViz and recording run on the laptop. **Hotspot is primary; Tailscale is an
explicit fallback.** No component here publishes motor commands.

For the temporary LDS-01 Section 1 MVP, use the [step-by-step checklist](../../docs/architecture/slam-section-1-validation-plan.md).
`ros2 run homer_slam mvp_check OUTPUT.json` captures live health; `--mode
stationary` and `--mode return` capture tracking evidence. Physical trials
require operator confirmation and visual checks before acceptance.

## Install and geometry

Keep the repository at `~/mobile-robot` on the Pi. From that checkout:

```bash
./pi/homer_slam/scripts/install-pi-dependencies.sh
./pi/homer_slam/scripts/build-workspace.sh
source /opt/ros/jazzy/setup.bash
source ~/homer_slam_ws/install/setup.bash
```

RF2O is pinned in `dependencies.repos`. A checked-in patch fixes initialization
without a sensor transform, invalid initial quaternion, uninitialized scan
state, changing scan size/backward timestamps, and excessive waiting logs.
Failed motion solves do not refresh odometry or TF. The estimator's internal
helper node keeps a separate name to avoid duplicate node/parameter services.

The default `geometry:=urdf` reads the canonical robot model directly from
`simulation/urdf/home_mobile_manipulator_v6_real_base_layout.urdf`. The model
places the wheel midpoint at its `base_footprint` origin. The package derives
`base_link -> laser` from the fixed chain to `rplidar_a1` (0.090, 0.075,
0.093 m) and a base bounding outline including wheels/casters (0.318 × 0.350 m).
This uses the model's LiDAR link origin; enclosure and optical scan-plane
origins must still be checked physically. It is not a new measured scan-plane
calibration. The outline represents the base, not the arm's swept volume.

For measured overrides, copy `config/geometry.yaml`, fill all values and the
source, set `verified: true`, and pass `geometry:=/absolute/path/geometry.yaml`.
Do not set that flag without measurement. Sensor orientation, footprint, map
accuracy, and drift remain `REQUIRES_HARDWARE_TEST`.

## Primary hotspot network

`config/network.json` has separate hotspot/Tailscale peers. Update both copies
if DHCP addresses change. The Pi's current hotspot IP is `192.168.137.250`;
the Windows hotspot host is expected at `192.168.137.1`. Validate the source
address advertised by the actual WSL interface; do not advertise an address
that exists only on Windows but is absent inside WSL.

The laptop currently runs WSL2. Prepare `%UserProfile%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
```

**Save running WSL work, run `wsl --shutdown` from Windows, then reopen WSL.**
This cannot be applied to a running WSL network. Confirm the Pi route and
source address with `ip route get 192.168.137.250`. If the source differs from
`192.168.137.1`, update `network.json` on both hosts to that reachable local
address. The network generator refuses NAT addresses outside the selected
subnet and never silently switches to Tailscale.

Mirrored mode alone does not guarantee that Windows' Wi-Fi hotspot adapter
appears in WSL. On the inspected workstation Windows had `192.168.137.1`,
while WSL exposed only the ordinary Wi-Fi adapters and sent Pi-bound packets
to their default gateway. This matches an open [Microsoft WSL hotspot issue](https://github.com/microsoft/WSL/issues/11741).
Check `ip -br addr`, `ip route get 192.168.137.250`, and Windows
`Get-NetIPAddress -AddressFamily IPv4` before changing DDS peers. A TCP SSH
proxy working on Windows does not prove that WSL can send DDS UDP directly.
Do not invent a Linux interface address or use a TCP port proxy as a DDS fix.

Install Docker Engine inside WSL (`sudo apt install docker.io`), start its
service, and add your account to the docker group. Reopen the session for the
group to take effect. The container uses WSLg, host networking, and your user
ID. Do not run RViz over SSH X11 forwarding.

On the Pi, while stationary:

```bash
./pi/homer_slam/scripts/apply-pi-network.sh hotspot
source ~/.config/homer-slam/network/ros.sh
```

This writes reversible service drop-ins and restarts only already-active ROS
services. It preserves disabled cameras/arm services. There is a brief control
link interruption during the restart; retain the existing stop/timeout path.
The old scan watchdog remains evidence-only, not a motor safety gate.

On the laptop:

```bash
./pi/homer_slam/scripts/view-container.sh ros2 topic list
./pi/homer_slam/scripts/view-container.sh ros2 topic echo /scan --once --field header
```

Use domain `0` and Cyclone DDS on both hosts. Explicit self/peer unicast
addresses replace multicast discovery. If data is missing, inspect hotspot
routing, WSL mirrored networking, Windows/Hyper-V firewall rules scoped to the
hotspot subnet, and the generated Cyclone profile before trying a fallback.
For an inbound firewall block, run `scripts/configure-hotspot-firewall.ps1`
from Administrator Windows PowerShell. It adds only subnet-scoped UDP
7400–7800 rules for Windows/WSL, sufficient for domain 0 and the configured
participant limit. `-Remove` removes those named rules. It does not disable
firewalls or change their global defaults.

SSH tests TCP administration access; ROS additionally needs bidirectional UDP.
Check clock synchronization on both hosts before interpreting timestamps:

```bash
python3 pi/homer_slam/scripts/check-clock.py
```

If Windows time is offset, run `scripts/sync-windows-clock.ps1` from an
Administrator Windows PowerShell, allow synchronization to settle, and repeat
the check. Changing only the WSL guest clock is temporary because Windows can
reset it. The check requires absolute offset plus uncertainty below 50 ms.

`ssh pi` uses the hotspot. `ssh pi-tailscale` uses the separate Tailscale path;
neither should silently change networks. The Windows TCP proxy helper in
`scripts/ssh-network-proxy.ps1` accepts exactly one peer address.

## Map and view

Discover `/scan` range metadata, then use those values; do not assume example
values are your sensor's actual limits:

```bash
ros2 topic echo /scan --once --field range_min
ros2 topic echo /scan --once --field range_max
ros2 launch homer_slam mapping.launch.py min_range:=ACTUAL_MIN max_range:=ACTUAL_MAX
```

RF2O publishes `/odom` and `odom -> base_link`; SLAM publishes `/map` and
`map -> odom`. Launch preflight rejects existing odometry/SLAM publishers,
and a host lock prevents overlapping Homer sessions. No sensor driver is
started by these launches. Stop the previous launch before starting another.
Estimator exit stops the complete processing launch; explicitly start a new
session rather than stitching a reset odometry origin into an existing map.

On the laptop:

```bash
./pi/homer_slam/scripts/view-container.sh
```

The view defaults to map + robot URDF + arrow + URDF-derived base outline at 10 FPS, with
markers updated at 5 Hz and map updates at 3 seconds. Optional laser and TF
can be enabled in RViz. Robot markers expire when local odometry goes stale.
RViz map/model data can remain visible after disconnection. A 1 Hz monitor
running on the laptop displays a red odometry/TF warning locally when data
goes stale, even if the Pi's diagnostics cannot reach the viewer. The warning
appears beside the last robot pose; detailed diagnostics remain optional.

The existing primitive-geometry robot URDF is displayed by default. For the
smallest rendering workload, disable its display and its publisher:

```bash
./pi/homer_slam/scripts/view-container.sh ros2 launch homer_slam view.launch.py show_model:=false
```

Uncheck **URDF model (illustrative joint pose)** in RViz for this minimal view. Its `display/` frames
are isolated from estimation frames; movable joints are shown at zero in an
explicitly illustrative model, not as measured arm/neck/wheel feedback. The
canonical URDF is unchanged. Model publishing/rendering run on the laptop.

## Save, reload, and record

Run the save command **on the processing host**, where SLAM writes files:

```bash
ros2 run homer_slam save_map ~/homer-maps/room-001
```

Choose a new directory every time. Success requires occupancy YAML/image and
both `.posegraph`/`.data` exports, plus configuration, actual launch parameters,
URDF and dependency/version evidence. A failed save retains partial output for
inspection and returns failure. Occupancy image paths are made portable.

Stop mapping, then reload the pose graph:

```bash
ros2 launch homer_slam localization.launch.py map_file:=$HOME/homer-maps/room-001/map min_range:=ACTUAL_MIN max_range:=ACTUAL_MAX
```

Use RViz's **2D Pose Estimate** to supply `/initialpose` if starting elsewhere.
Map YAML alone is insufficient for this localization launch.

Record on the laptop:

```bash
./pi/homer_slam/scripts/view-container.sh ros2 run homer_slam record /data/bags/room-001
```

Camera recording is opt-in through `--camera-topics`, using discovered image
and calibration topic names. Default bags include `/map` and the lightweight
markers as well as scans, odometry, TF, diagnostics, and drive status so a
full replay can display the recorded map and robot. Keep an eye on disk usage. A scan arriving on time can still contain no valid distances; confirm actual returns before mapping. RF2O refuses fewer than 16 valid returns, diagnostics report them, and saving refuses an empty or entirely unknown occupancy map.
The live external Arducam driver currently publishes 240×180 `32FC1` depth in **millimetres**, not metres; convert units and calibrate before feeding it to standard RGB-D algorithms. No `CameraInfo` topic was found in the inspected graph. To replay, stop all
live processing on that ROS domain, use `ros2 bag play BAG --clock`, and launch
processing/view with `use_sim_time:=true`. If a bag contains `/odom` or TF from
live processing, replay only `/scan` when recomputing odometry; otherwise you
would create duplicate authorities. To view recorded results, replay the full
bag without launching RF2O/SLAM.

## Compute and fault evidence

```bash
ros2 run homer_slam baseline ~/homer-evidence/scans.json --seconds 600
# Repeat with SLAM running and the measured baseline scan rate:
ros2 run homer_slam baseline ~/homer-evidence/slam.json --seconds 600 --baseline-scan-hz MEASURED_RATE
ros2 topic echo /diagnostics
```

Initial gates: odometry p95 age <200 ms, average system CPU <75%, scan rate
within 10% of baseline, no current throttling, and continuous streams. Unknown
measurements are not passes. Record processing age and gaps; they are backlog
proxies, not direct queue-depth measurements. Diagnostics detect freshness,
nonfinite poses, missing/stale TF, duplicates, and authority/time changes.
They do not certify tracking accuracy or stop motors.

If the Pi fails the gates, stop its processing launch/service explicitly,
then run `mapping.launch.py` through the laptop container. Keep drivers and
motor control on the Pi. Save maps on that processing host/container, using
`/data`. Never switch hosts automatically.

## Optional service and explicit fallback

Install `pi/homer-slam.service` as a user unit only after manual trials pass.
Set `~/.config/homer-slam/session.env` with `HOMER_MIN_RANGE` and
`HOMER_MAX_RANGE`. The unit is disabled by default, has no restart policy,
and never starts the viewer or overwrites maps.

For a deliberate fallback, stop processing, apply the Pi Tailscale profile,
and select the same profile on the laptop:

```bash
./pi/homer_slam/scripts/apply-pi-network.sh tailscale
HOMER_NETWORK_PROFILE=tailscale ./pi/homer_slam/scripts/view-container.sh
```

Return to `hotspot` on both hosts to restore primary operation.
`rollback-pi-network.sh` removes only the managed network drop-ins and
restarts active affected units. Backups are kept under
`~/.local/state/homer-slam/network-backups/`.

To roll back SLAM, stop manual launches and `systemctl --user stop homer-slam`,
then remove the optional mapping unit and run the network rollback helper.
Sensor and drive services remain independent of this package. Restoring wheel
odometry additionally requires setting firmware `kEnableEncoderOdometry=true`,
building/flashing through `deploy.sh`, and explicitly setting bridge
`publish_encoder_odometry:=true` in its launch/service. Stop RF2O first so that
encoder and LiDAR odometry never publish `/odom` together. The encoder-disable
flag remains the default until deliberately changed.

## Tests and physical acceptance

```bash
PYTHONPATH=pi/homer_slam python3 -m unittest discover -s pi/homer_slam/test -p 'test_*.py' -v
# On a Jazzy host with the Homer workspace sourced:
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=71 python3 pi/homer_slam/test/ros_smoke.py
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=71 python3 pi/homer_slam/test/ros_smoke.py --sim-time
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=71 python3 -m unittest discover -s pi/tests -p test_encoder_odometry.py -v
```

The isolated synthetic test covers scans -> odometry/TF/map/markers, refusal
of overlapping launch, save/refuse overwrite, stale data, and localization
reload. It publishes no drive commands and is not physical validation.

`REQUIRES_HARDWARE_TEST`: hotspot-only ROS with no Tailscale route; sensor
orientation; stationary drift (<0.05 m/3 degrees over 60 s); three slow room
loops; map dimensions within 0.10 m; return pose within 0.20 m/5 degrees;
save/restart/localize three times; viewer/hotspot loss, scan dropout, estimator
restart, and failed save; separate fallback-profile verification. Keep manual
control during trials. Navigation and camera fusion are later milestones.

### Offline RViz inspection

On the laptop, show recorded results without requiring hotspot discovery:

```bash
./pi/homer_slam/scripts/replay-container.sh /path/to/bag-directory
```

This starts the URDF viewer with bag clock in a container with networking
disabled. It plays the full recording at half speed and keeps the RViz window
open afterward. The final frame is paused recorded data, not live telemetry.
It never launches an estimator. A stationary or invalid recording cannot
demonstrate robot motion or mapping accuracy.
