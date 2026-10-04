# SLAM accuracy checks — 2026-10-04

Nav2 was paused at the user's request. No navigation stack was deployed.
RViz remains a read-only live SSH viewer with scans, odometry, TF, and map.
Its local view is zoomed in and includes an odometry trail. These viewer
settings live outside Git in `~/.local/state/homer-slam/lds01-live-view/`.

## Short stability snapshot

A read-only 10-second recording collected 50 odometry samples. Assuming the
robot stayed stationary, maximum odometry excursion was **0.634 cm and
1.121°**; endpoint change was 0.455 cm and 0.857°. The map-frame robot pose
showed maximum excursion of 0.541 cm and 0.787°. Maximum odometry reception
gap was 0.245 seconds. This run did not obtain a fresh operator confirmation
of immobility and does not qualify the original 60-second stationary gate.
The earlier 60-second result remains failed on yaw (3.71° maximum).

These are stability measurements, not externally measured accuracy.
The directly forward scan median was 2.0865 m; no tape reference has yet
been supplied for comparison. Next: compare a tape-measured wall distance,
then a marked, measured 30 cm operator movement and return. Do not certify
accuracy from a motion command or odometry alone.

## Battery motion attempts

The user confirmed battery power and requested a short movement test.

- First attempt: stopped before sending nonzero commands because a front-sector
  return was 0.408 m, inside the 0.55 m clearance limit. Motor outputs zero.
- Second attempt: forward command 0.35, zero steering, bounded to three seconds
  or 0.30 m odometry displacement. Stopped early when front clearance became
  0.545 m. Estimated endpoint displacement was 0.0130 m, heading change 0.121°.
  Motor outputs confirmed zero afterward. Physical movement was not confirmed;
  the user could not clearly see visualization movement. That small estimate
  can include lidar drift, so the 30 cm test has not passed.

Motor commands currently map to open-loop PWM, not calibrated metres/second.
No autonomous navigation or automatic return motion was run. Existing gamepad
teleoperation remains active. Mapping had been inactive after the power change;
its service was started. A stale lidar feed recovered after restarting the
existing driver. A settled three-second live check then passed sensing,
transform freshness, unique publisher, and nonempty-map checks at 4.96 Hz.

## Evidence

Pi: `~/homer-evidence/section-1-mvp/`:

- `short-motion-20261004-090935.json`
- `short-motion-20261004-091311.json`
- `live-20261004-rviz.json`
- `stability10-20261004-091752.json`

Motion and stability JSON copies are also in workstation
`~/.local/state/homer-slam/section-1-mvp-20261004/`.
Temporary test scripts are not installed services. The movement scripts monitor
fresh scans/odometry/drive status, front clearance, and operator mode, send zero
commands on exit, and leave the existing drive and teleoperation services intact.
