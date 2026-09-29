# Encoder observation

Date: 2026-09-27

## Straight-line movement test

The robot was moved straight from approximately 4 cm to 30 cm on the physical
reference, for an expected displacement of about 26 cm. The robot was not
intentionally turned.

Odometry readings from `/odom`:

| Measurement | x (m) | y (m) |
| --- | ---: | ---: |
| Before movement | 0.1698 | -0.0233 |
| After movement | 0.1439 | 0.0127 |

The reported displacement was approximately 4.4 cm, or about 17% of the
expected physical displacement. The reported heading also changed by roughly
106 degrees despite the robot being moved straight.

## Observation

The heading change during a straight-line test indicates that the issue is
probably not only wheel-diameter calibration. One encoder is likely counting
with the opposite polarity to the other encoder, causing the odometry to
interpret straight travel as rotation.

## Follow-up

### 2026-09-28 hand-moved run

The operator moved the robot from the 4 cm mark to the 44 cm mark (40 cm
forward). The motor-disabled calibration image recorded left `+906` counts and
right `-699` counts during that movement. The complete capture is in
[`encoder-hand-2026-09-28.csv`](encoder-hand-2026-09-28.csv). Both wheels moved
forward, so the opposite count signs confirm the right encoder polarity was
reversed in software. The right ISR sign has been inverted in production and
calibration sources. Physical retesting of the new images is still required.

With the current 65 mm wheel diameter and 377 counts per wheel turn, the raw
counts imply 49.1 cm left and 37.9 cm right. After correcting the right sign,
their mean is 43.5 cm, versus 40 cm measured. The unequal magnitudes could
reflect wheel slip, an uneven hand push, or calibration error. One run does not
identify which, so wheel diameter and counts per turn have not been changed.

### 2026-09-29 lifted-wheel turn

With the motor-disabled calibration image, the operator turned each lifted
wheel forward one marked revolution and confirmed each returned to its starting
mark. The capture in
[`encoder-wheel-turn-2026-09-29.csv`](encoder-wheel-turn-2026-09-29.csv) started at
zero and settled at left `+506`, right `+379` counts. Neither count decreased
during the capture. A second marked revolution of each wheel in
[`encoder-wheel-turn-repeat-2026-09-29.csv`](encoder-wheel-turn-repeat-2026-09-29.csv)
settled at left `+497`, right `+379`, again without any decreasing counts.

The right result is close to the former shared calibration of 377 counts per
wheel turn; the left consistently produced about 500. The two left/right
ratios are 1.335 and 1.311, close to the 1.296 magnitude ratio from the
earlier straight-line run. Production now uses separate 502 left and 379 right
counts per wheel turn, rounded from the two measurements. At the current 65 mm
wheel diameter, the old 40 cm run would imply approximately 36.9 cm left and
37.7 cm right after this correction. This is a retrospective calculation, not
a validated straight-line odometry test. The remaining distance error and the
cause of the different counts still need investigation. Inspect the left
encoder's mechanical ratio and signal if the count changes with wheel speed.

The production image was restored, flash-verified, and its ROS bridge smoke
checks passed after each raw-count capture. The new per-wheel calibration was
then built and flashed; all regions passed flash verification, and the bridge
smoke checks for `/drive/link`, `/odom`, and `/drive/status` passed. A physical
straight-line odometry run is still required before this branch is merged.


Use the standalone `encoder_calibration` image for raw measurements. It keeps
all motor control pins low and does not run the ROS bridge or accept motion
commands. Build it on the workstation with `./build.sh encoder_calibration`.
Only flash it during a separate calibration session; restore and verify the
known working production image before driving again. With the diagnostic
image running, capture its 115200 baud USB serial CSV while turning each
marked wheel forward by exactly one revolution by hand, then move the robot
straight along a measured floor mark with motor power disabled. No ROS topic
or extra traffic in the driving firmware is needed.

The CSV records elapsed milliseconds, signed cumulative counts, signed count
changes, per-wheel travel, and the linear/angular velocities calculated from
that same count interval. One forward wheel revolution should produce roughly
502 left or 379 right counts of the correct sign if the current calibration is
stable. In a straight-line run, compare the calibrated wheel travel rather
than the raw counts; both wheel travel estimates should be positive and similar.
Compare their mean with the measured floor distance. The
velocity columns use `v=(left_m+right_m)/(2*dt)` and
`omega=(right_m-left_m)/(track_width*dt)`, with `dt` in seconds. This separates
encoder polarity/counts-per-turn errors from wheel diameter and track-width
calibration.

Check the direction/sign handling in:

```text
firmware/src/wheel_encoders/wheel_encoders.cpp
```

Specifically, verify the signs in `countLeft()` and `countRight()` while the
robot is driven forward. Correct encoder polarity first, then repeat the
straight-line test before tuning `kWheelDiameterMeters` or other odometry
constants.
