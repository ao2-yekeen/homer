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
377 counts of the correct sign if the current configuration is right. The
straight-line run should produce positive, similar left and right counts;
compare their total wheel travel with the measured floor distance. The
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
