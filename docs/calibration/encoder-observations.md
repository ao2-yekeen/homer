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
not only wheel-diameter calibration. The right encoder's direction sensing
requires checking in both directions; the later reverse test below showed
that its count sign did not reverse.

## Follow-up

### 2026-09-28 hand-moved run

The operator moved the robot from the 4 cm mark to the 44 cm mark (40 cm
forward). The motor-disabled calibration image recorded left `+906` counts and
right `-699` counts during that movement. The complete capture is in
[`encoder-hand-2026-09-28.csv`](encoder-hand-2026-09-28.csv). Both wheels moved
forward, so the opposite count signs initially appeared to indicate reversed
right encoder polarity. The right ISR sign was inverted in production and
calibration sources. That made forward readings positive, but the later
reverse test below showed that the right count still increased when its wheel
was turned backward. The original one-direction test did not establish a
working direction signal.

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
smoke checks for `/drive/link`, `/odom`, and `/drive/status` passed.

### 2026-09-29 straight-line odometry validation

With the corrected production image running, the operator moved the robot
straight from the 1 cm mark to the 40 cm mark, a measured 39 cm displacement.
The live `/odom` capture in
[`encoder-straight-2026-09-29.csv`](encoder-straight-2026-09-29.csv) contains 384
messages. It started at `(x=0.4494, y=0.0318)` m and ended at
`(x=0.7628, y=0.2568)` m. The endpoint displacement was 38.58 cm, 0.42 cm
below the marked distance (1.1%). Heading changed by about 1.25 degrees.
This validates the per-wheel count conversion for this hand-moved straight
run. Wheel diameter, track width, and motor-driven odometry still have not
been independently calibrated.

### 2026-09-29 reverse observation

The operator commanded reverse with the joystick, reported that both wheels
rolled backward while the robot stayed pointed straight, and observed travel
from approximately the 30 cm mark to the 2 cm mark (roughly 28 cm). In the saved
[`encoder-reverse-2026-09-29.csv`](encoder-reverse-2026-09-29.csv) capture,
`/odom` moved only 3.72 cm between endpoints while its heading changed by
82.8 degrees. This is a large mismatch; the approximate floor marks do not
explain it. The reported physical motion conflicts with the odometry, but the
capture does not expose raw left/right encoder counts and therefore does not
identify which signal or powered-drive condition caused it. Do not use the
successful forward run to claim reverse odometry is calibrated. Before changing
scale factors, check raw counts for one marked reverse wheel turn with motors
disabled, then capture raw counts during a controlled powered reverse run.

Reconstructing wheel travel from successive odometry positions and headings
with the configured 0.33 m track width gives about `-0.197 m` left and
`+0.280 m` right during this reverse run. The right magnitude agrees with the
reported roughly 28 cm move, but its sign is forward; the left sign is reverse
but its magnitude is short. These are inferred from integrated odometry, not
raw count frames. A single shared sign flip or wheel-scale adjustment would
not correct both discrepancies. The cause cannot be isolated from this capture
alone because the forward validation was hand-moved and this reverse run was
joystick-powered.

### 2026-09-29 motor-disabled reverse wheel turns

The operator turned the wheels backward by hand with the motor-disabled
calibration image running. The raw capture in
[`encoder-reverse-hand-2026-09-29.csv`](encoder-reverse-hand-2026-09-29.csv)
started at zero and ended at left `-488`, right `+369` counts. Every nonzero
left delta was negative and every nonzero right delta was positive. Earlier
forward hand turns gave positive counts on both sides. Thus the right count
sign stays positive for both wheel directions, even with the motors off. This
confirms a right direction-sensing fault; motor electrical noise is not needed
to reproduce it. In the current firmware, GPIO 22 (right encoder B) is sampled
when GPIO 23 (right encoder A) rises to choose count direction. The capture
cannot distinguish a disconnected or incorrect B wire, a failed B channel,
or an input-level problem. The previous single sign inversion cannot repair
this, and reverse odometry must be treated as unreliable until the direction
signal is corrected and retested. Production firmware was restored, verified,
and the ROS bridge smoke checks passed after the capture.

### 2026-09-29 repeated forward wheel turns

The operator requested another raw capture for one marked forward revolution
of each wheel. In
[`encoder-forward-one-turn-2026-09-29.csv`](encoder-forward-one-turn-2026-09-29.csv),
the final counts were left `+479` and right `+368`. Two left deltas were negative
during the turn, for 481 positive and 2 negative counts; every right delta was
positive. Compared with the preceding hand-moved reverse capture (`-488`
left, `+369` right), the magnitudes are close but the right sign is positive
in both directions. Earlier forward one-turn captures gave left `+506`/`+497`
and right `+379`/`+379`. These variations do not justify changing the current
502/379 counts-per-turn constants based on this single repeat. The direction
fault remains the priority.

The diagnostic flash initially lost serial contact twice and the first
production restore also stopped partway through. Both images were recovered
at lower flash rates. The final production image passed verification in all
four regions at 38400 baud, and `/drive/link`, `/odom`, and `/drive/status`
were observed after the serial bridge restarted.


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
changes, cumulative transitions on each encoder B pin, per-wheel travel, and
the linear/angular velocities calculated from that same count interval. The B
transition counters help distinguish a quiet/disconnected direction channel
from a channel that changes but has the wrong phase relative to A. One forward
wheel revolution should produce roughly
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
