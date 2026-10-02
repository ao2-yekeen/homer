# Temporary LDS-01 SLAM continuation — 2026-10-02

The operator connected a TurtleBot3 lidar temporarily for the weekend. It is
sitting separately, rather than mounted on Homer. Real bench mapping, occupancy
export, pose-graph export, and localization reload now work. Robot geometry and
moving-room accuracy remain **REQUIRES_HARDWARE_TEST**. No drive commands were
sent during these checks; serial bridge and teleoperation stayed active.

## Driver and Pi configuration

Passive serial capture at 230400 baud received 50,232 bytes in four seconds,
with 1,195 consecutive LDS-01 headers spaced 42 bytes apart. The old RPLIDAR
protocol/115200-baud driver timed out on this sensor. Use the ROBOTIS
`hls_lfcd_lds_driver` for this temporary device.

The official Jazzy arm64 Debian package was downloaded on the workstation,
verified against the Pi apt metadata, and extracted without root into
`~/homer-lds01-test` on the Pi. It is not registered as a system-installed apt
package. Package SHA256:
`9f60f02400fde425a3488f1e5656e91216fff3df18e00d727c5c5b92d527bf16`.

Persistent user-service override:
`~/.config/systemd/user/rplidar.service.d/90-weekend-lds01.conf`.
It replaces only the driver's ExecStart, preserving existing device discovery,
network environment, dependency names, and enabled boot state. The driver
uses `%t/mobile-robot-devices/lidar` and publishes `/scan` in frame `laser`.
Runtime restart was checked; an actual reboot test has not been performed.

`~/.config/homer-slam/session.env` now contains:

```bash
HOMER_MIN_RANGE=0.12
HOMER_MAX_RANGE=3.5
```

Its previous contents are backed up in
`~/.local/state/homer-slam/lds01-weekend/session.env.before`.
`homer-slam.service` is running in mapping mode and remains disabled for boot.
There is one publisher each for `/scan`, `/odom`, and `/map`.

```bash
ssh pi
systemctl --user status homer-slam.service
systemctl --user stop homer-slam.service
systemctl --user start homer-slam.service
```

Stop the service before running mapping/odometry/localization launches manually.
Source Jazzy, `~/homer_slam_ws/install/setup.bash`, and
`~/.config/homer-slam/network/ros.sh` in manual ROS shells. Supply
`min_range:=0.12 max_range:=3.5` to processing launches.

## Real scan and map evidence

- 65-second mapping probe: 321 scans, 4.94 scans/sec, at least 209 valid
  returns in every 360-bin scan; 320 odometry messages.
- Map: 68 × 47 cells at 0.05 m resolution; 344 known cells, including
  290 free and 54 occupied cells. This is a stationary local bench map,
  not a surveyed room map.
- Scan, odometry and TF diagnostics reported level zero at probe completion.
- Received odometry age p95 was 53.2 ms. This short probe is not the required
  representative ten-minute compute run.
- First-to-last observed odometry change: 0.0059 m and 3.20 degrees over
  approximately 64.8 seconds. Rotation exceeds the 3-degree stationary target;
  the lidar's physical immobility was not independently monitored.
- Temperature spot checks were approximately 75–79°C; one mapping check
  returned `get_throttled=0x0`. No thermal/compute acceptance is claimed.

Saved on Pi:
`~/homer-maps/lds01-weekend-bench-20261002` with nonempty occupancy image/YAML,
`.posegraph`, `.data`, and actual parameter/configuration evidence.
Thirty seconds of real topics were recorded to
`~/homer-evidence/lds01-weekend-bench-20261002`.

Mapping was explicitly stopped before a single localization stack was launched
against that pose graph. The 30-second reload probe received 150 scans and 121
odometry messages, and a 68 × 49 map with 545 known cells. Diagnostics reported
fresh scan/odometry and available TF. This verifies bench reload, not moving
localization accuracy. Localization was stopped and mapping restarted afterward;
the running mapping session has a new origin and does not resume the saved graph.

Workstation evidence root:
`~/.local/state/homer-slam/validation-20261002/lds01/weekend/`.
It includes JSON probe results, exported map files, recording metadata/MCAP, and
`bench-map.png`. No evidence files or machine-specific service overrides are
tracked in Git.

## Remaining work and rollback

The estimator still uses the original URDF-derived lidar transform. Do not mark
that transform measured for this separate temporary sensor. Verify mounting and
orientation before testing robot motion accuracy. Three slow room loops,
localization traversals, stationary rotation, ten-minute compute gates, and
failure checks remain outstanding. Navigation is outside this milestone.

WSL still routes `192.168.137.250` through `10.142.22.1`; hotspot DDS and clock
acceptance remain unresolved. SSH access works, but does not prove laptop ROS
connectivity. No network profile was switched.

For immediate weekend use, a read-only live SSH relay is running in Docker
container `homer-lds01-live-view`. It subscribes to Pi scan, odometry, map, TF,
markers and diagnostics, transfers serialized messages over SSH, and republishes
them in isolated container domain 94. The Pi clock is relayed as `/clock`, and
the viewer uses simulated time to share that clock. This is **live telemetry**,
not bag playback. Only the existing Pi stack performs estimation; the container
runs the viewer. RViz was visually checked: Global Status OK, nonempty map and
the existing URDF visible. This temporary transport does not qualify the planned
direct hotspot DDS or independent host clock gates.

Workstation helper files are outside Git at
`~/.local/state/homer-slam/lds01-live-view/`. To reopen after closing RViz:

```bash
bash ~/.local/state/homer-slam/lds01-live-view/start.sh
```

Do not start a second copy while the named container is running. Stop the
temporary viewer/relay with `docker stop homer-lds01-live-view`; Pi mapping and
teleoperation remain independent. These helpers are temporary bench tools,
not a qualified transport implementation.

When reconnecting the original RPLIDAR, stop mapping and restore both driver
and range settings:

```bash
systemctl --user stop homer-slam.service
rm ~/.config/systemd/user/rplidar.service.d/90-weekend-lds01.conf
cp ~/.local/state/homer-slam/lds01-weekend/session.env.before ~/.config/homer-slam/session.env
systemctl --user daemon-reload
systemctl --user restart rplidar.service
```

Recheck valid ranges before restarting SLAM; the original cable/sensor issue
has not been repaired by this temporary substitution.
