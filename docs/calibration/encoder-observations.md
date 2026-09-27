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

Check the direction/sign handling in:

```text
firmware/src/wheel_encoders/wheel_encoders.cpp
```

Specifically, verify the signs in `countLeft()` and `countRight()` while the
robot is driven forward. Correct encoder polarity first, then repeat the
straight-line test before tuning `kWheelDiameterMeters` or other odometry
constants.
