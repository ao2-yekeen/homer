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

## Marked 30 cm attempt

The operator placed tape straight and requested 30 cm forward so actual travel
could be measured. The bounded command began at 0.35 and permitted at most 0.45,
with a twelve-second ceiling and an early stop for unsafe direction, excessive
lateral/heading change, inadequate clearance, stale inputs, or mode takeover.
It stopped on the tracking-direction limit. Estimated endpoint was **5.45 cm
backward and 5.88 cm left**, with heading change -4.70°. Motor outputs confirmed
zero. The 30 cm target was not reached; actual tape travel and physical direction
are awaiting the operator's measurement. This discrepancy needs resolution
before another movement or any claim of accurate SLAM. Do not interpret the
estimated backward direction as proof the hardware moved backward.

Evidence: Pi `~/homer-evidence/section-1-mvp/measured30-20261004-092031.json`,
with a workstation copy alongside the preceding JSON files.

The operator said the first movement was approximately the reported value and
requested a repeat. The second attempt also stopped on direction: estimated
3.53 cm backward, 0.66 cm left, heading +1.18°. Motor outputs zero. Evidence is
`measured30-20261004-092242.json` in the same Pi and workstation directories.
The operator's earlier approximate-value reply did not specify physical travel
direction; forward/backward confirmation was requested after this repeat.
Do not reverse motor commands or rotate the lidar transform on that ambiguous
reply alone. Distinguish physical motor direction from sensor orientation before
attempting the full 30 cm again.

## Forward-command retry: failed physical distance bound

After the operator confirmed physical backward travel, the next temporary test
used negative drive commands to request physical forward travel. RF2O reported
8.99 cm forward and 9.59° heading change; the script stopped on its heading limit.
Motor outputs were zero afterward. Evidence:
`measured30-20261004-092340.json` in the same Pi/workstation directories.

The operator measured approximately **50 cm**, and reported backward motion near
the end. This exceeds the intended 30 cm by about 20 cm. Estimated forward travel
was about 82% below the approximate physical measurement. The test failed to
bound physical travel; trusting unvalidated lidar odometry as its sole distance
cutoff was inappropriate. Telemetry records negative commands (-0.35, -0.45)
then zero, with about 5.8 s between first and last nonzero status and duty up to
99/255. It does not explain the observed end reversal.

Remote automatic motion tests are stopped. Do not resume them using previous
authorization. The user requested camera–lidar research and simulation before
deployment; see [the review](slam-camera-fusion-literature-review-2026-10-04.md).
The superseded simulation-first plan has been removed. No firmware,
TF polarity, estimator tuning or map-refresh changes were deployed as a result
of this failure. Motor zero was reconfirmed through `/drive/status`.
