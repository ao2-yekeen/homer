# Motor diagnostic summary

## Confirmed working

The standalone `motor_diagnostic` firmware drives both motors directly from
the ESP32, without ROS, micro-ROS, gamepad, encoders, or controller code.

At full PWM (`255`), both motors start and stop together. This confirms the
motor pins, driver outputs, and power path at full duty. The standalone
diagnostic powers both motors continuously from PWM `154` to `255`, one duty
count at a time. Each value runs for 2 seconds. Both wheel encoders record the
first PWM that produces at least three pulses; both motors stop only after the
full-duty step. PWM `154` matches the tested ROS command (`linear.x = 0.7` ×
drive scale `220`).

## Shared PWM ramp result

The non-ROS diagnostic applied the same continuously increasing PWM to both
motor outputs, from `154` through `255`, for two seconds per value. The right
encoder produced 1,684 pulses at PWM `154`, establishing its start threshold
at `154`. The left encoder produced zero pulses at every tested value,
including PWM `255`. Both motor outputs were stopped after the final sample.

## Remaining issue

With the normal micro-ROS firmware, terminal drive tests were inconsistent:
the motors could pulse, or only one appeared to move. During a ROS probe, the
ESP32 `drive/status` topic reported mode `stopped` and PWM `0` for both motor
channels after commands were published.

With the four Pi command services stopped, the ESP32 still reported mode 2.
Publishing `/mode_override` with `data: -1` once and then repeatedly did not
change that reading. The mode callback did not act on those commands, although
ROS showed an ESP32 subscription on that topic. The serial receive path or
running firmware therefore still needs investigation.

## Next action

The current `esp32dev` source is a temporary motor-only micro-ROS diagnostic:
one `/cmd_vel` subscription, no mode gates, telemetry, odometry, neck control,
or timeout. It has built locally but has not been flashed or physically tested.
Keep the wheels lifted and motor power disconnect accessible for that test.
