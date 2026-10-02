# Encoder-free SLAM validation — 2026-10-01–02

This record distinguishes software checks from physical acceptance. No motor
commands were sent by these checks. The implementation is not yet accepted
for navigation or unattended operation.

## Installed and checked

- Encoder-disable change `d3d602d` integrated as `decd8a9`.
- ESP32 firmware built locally and flashed through `deploy.sh esp32dev`;
  bootloader, partition table, boot app, and application all passed readback
  verification. Drive link and status returned after flashing.
- Serial bridge defaults to no encoder `/odom` publisher. Live graph after
  deployment had zero `/odom` publishers before starting RF2O.
- Pi workspace builds with RF2O at
  `b38c68e46387b98845ecbfeb6660292f967a00d3` plus the checked-in patch.
- Six geometry/network tests, three serial protocol tests, and five teleop
  control-model tests passed. URDF offsets and footprint are model-derived;
  physical scan-plane calibration remains pending.
- Both ROS serial-bridge tests passed with Cyclone DDS: encoder publication
  disabled by default, explicit re-enable retaining legacy behavior.
- A synthetic mapping run passed odometry/TF/markers, overlapping-launch
  refusal, occupancy/pose-graph save, overwrite refusal, stale-data detection,
  and localization reload. Final-patch and simulated-clock runs are checked
  separately below; a synthetic result does not establish physical accuracy.
- Final-patch wall-time and simulated-time synthetic runs both passed,
  including healthy diagnostics, correct URDF transform/outline, missing
  geometry/maps, unavailable save services, and local viewer stale warnings.
- Container RViz started through WSLg under the host user with OpenGL 4.5.
  The packaged viewer also starts and publishes the URDF display. A
  recorded-data review exposed invalid LiDAR data, described below.
- Explicit Tailscale Cyclone profiles passed bidirectional DDS ping/ack
  between the Pi and laptop container in isolated domain 72. No estimators
  ran in that domain; existing services retained the hotspot domain-0 profile.
  This transport check does not qualify a complete live fallback SLAM run.
- Pi services use Cyclone DDS, domain 0, `wlan0`, self/peer hotspot addresses
  only. `/scan` and retained `/drive/link=True` were received locally.
  Network drop-in backup: `20261001T221526Z`.
- Existing ToF publisher uses 240×180 `32FC1` **millimetres**, processing-time
  timestamps, and frame `tof_camera`. No camera calibration topics were found.

The Pi initially mixed 2024 Jazzy core/interface packages with newly installed
2026 packages. TF serialization failures required
updating affected middleware, TF, Python runtime, and message packages. The
dependency installation script includes those packages for subsequent setups.
Runtime testing also fixed the diagnostics severity byte representation, TF
metadata compatibility, lifecycle activation, and failed-solve publication.

## Network and clock blockers

On October 1 WSL used its NAT interface (`172.30.186.74`). After the October 2
restart it uses mirrored networking, but still does not expose Windows'
hotspot interface. Windows has `192.168.137.1`; WSL has `10.142.22.6` and
`172.16.16.173`, and routes Pi-bound traffic to `10.142.22.1`. The generator
correctly refuses that route for the hotspot profile. This matches the
reported [WSL hotspot-adapter issue](https://github.com/microsoft/WSL/issues/11741).
Docker host networking does not resolve the missing interface.

Windows/WSL time was approximately 213 ms behind the synchronized Pi.
Adjusting only the guest clock briefly passed the 50 ms check, then Windows
reset the offset. On October 2 the primary hotspot SSH check measured
approximately -196 ms offset with 2.6 ms uncertainty, also failing the gate.
Administrator Windows clock synchronization is required;
the checked-in PowerShell helper and repeatable offset check are provided.

`ssh pi` selects the hotspot explicitly. `ssh pi-tailscale` preserves a
separate fallback. The Windows hotspot SSH proxy intermittently failed under
WSL load, so Pi administration tests used the explicitly selected fallback.
Hotspot SSH passed again on October 2. This is not evidence of hotspot-only
laptop ROS visualization.

## Compute evidence

The ten-minute pre-SLAM scan baseline was saved on the Pi at
`~/homer-evidence/scans-hotspot-20261001.json`: 600.04 s, 6.51 scans/s,
60.55% average system CPU. Thermal-limit flags occurred, with a spot-check
temperature around 85°C. This run overlapped installation/build/synthetic
testing and is unsuitable for accepting representative SLAM performance.
No odometry existed in that baseline, so odometry/continuity gates are
expected to be unfulfilled. On October 2, before rebuilding, temperature
was about 63°C and `get_throttled` returned `0x0`.

A separate real-LiDAR mapping run after compilation lasted 600.01 s:
6.42 scans/s, 59.67% average system CPU, and 205.3 ms p95 odometry age.
Scan-rate and stream-continuity gates passed; odometry age and thermal flags
failed. Both services wrote occupancy and pose graph exports at
`~/homer-maps/live-1790933702`, then onboard processing was explicitly stopped.
The optional persistent mapping service is installed, inactive, and disabled.
Inspecting the recorded data during RViz review found **all 137 scans had
zero valid distances**, every map message had width 0 and no cells, and the
exported image was a 1×1 placeholder. These files are **not usable maps**;
service success alone was insufficient verification. The measured CPU and
odometry age therefore do not qualify representative mapping performance.
Ten fresh live scans also had zero valid distances. Restarting only the LiDAR
service yielded nine empty scans and one scan with one return at 0.465 m.
The operator confirmed the sensor was spinning and uncovered. The driver
reports health OK, but that does not prove usable measurements. Hardware/data
recovery is still required; do not use these exports for localization.

The viewer was left open with the recording for inspection. Its stale-transform
warning and empty map must not be mistaken for a working SLAM demonstration.
Recorded-map/URDF reception and duplicate-estimator refusal passed; a scan-only
recomputation check failed to produce a map and is not recorded as a pass.
Code now rejects scans with fewer than 16 valid returns, reports invalid ranges,
and refuses empty/all-unknown maps before exporting. Updated guard smoke tests
passed with wall time and simulated time on October 2; both PASS results are
recorded in `ros-validity-smoke.log`. Three moving room loops remain outstanding.

The later temporary LDS-01 recovered real ranges and passed bench mapping,
save and localization reload. See the
[weekend continuation](slam-lds01-weekend-2026-10-02.md) for measured results
and the remaining geometry, stationary rotation, and network limitations.

## Physical acceptance — REQUIRES_HARDWARE_TEST

- [ ] Hotspot-only bidirectional ROS, with Tailscale unavailable; packet/routes
      verify selected addresses, and `/scan`, `/map`, and TF reach RViz.
- [ ] Clock offset plus measurement uncertainty <50 ms after Windows sync.
- [ ] Confirm URDF LiDAR scan origin/orientation and base footprint.
- [ ] Stationary drift <0.05 m and 3° over 60 seconds.
- [ ] Three slow room loops; dimensions within 0.10 m; loop-closure return
      pose within 0.20 m and 5°.
- [ ] Save, restart, reload, and localize on three physical traversals.
- [ ] Ten-minute representative processing compute gates pass; synthetic
      scenes and measurements during package compilation are insufficient.
- [ ] Viewer disconnect, hotspot loss, scan dropout, estimator restart, and
      failed save; mapping continues through viewer/network loss.
- [ ] Explicit Tailscale profile verified separately, without duplicates.
- [ ] Code review and physical results recorded before merging.

See the [operations guide](../../pi/homer_slam/README.md) for exact commands,
service opt-in, replay restrictions, and rollback.
