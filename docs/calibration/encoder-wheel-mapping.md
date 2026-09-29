# Physical wheel to encoder mapping

On 2026-09-29, the motor-disabled diagnostic recorded isolated hand turns of
each physical drive wheel in
[`encoder-b-direction-2026-09-29.csv`](encoder-b-direction-2026-09-29.csv).
Left and right refer to the robot's sides when facing forward.

| Wheel turned | Encoder pins that responded before correction | Observation |
| --- | --- | --- |
| Physical right | GPIO 4 (A), GPIO 19 (B), formerly labelled left | Count ranged from -136 to +346; B had 2,694 transitions. |
| Physical left | GPIO 23 (A), GPIO 22 (B), formerly labelled right | Count rose to +724; B had zero transitions. |

The production pin mapping now labels GPIO 23/22 as the physical left encoder
and GPIO 4/19 as the physical right encoder. The right count direction rule was
aligned with the observed forward and backward signs of the physical right
wheel. Counts per wheel turn follow the same physical sides: approximately 379
left and 502 right, based on marked hand turns.

**Unresolved hardware fault:** GPIO 22 registered no B transitions while the
physical left wheel's A input counted. Its count increased in both directions
in the earlier reverse capture. The mapping correction cannot make that
direction measurement valid. Reverse odometry and turn estimates involving
that wheel remain unreliable until the left B signal is repaired and checked
in both directions. This branch does not change the ESP32 image currently
running on the robot.
