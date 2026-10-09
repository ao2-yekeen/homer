# Collection test results

Executed the documented Pi commands:

```bash
cd ~/mobile-robot
bash act/scripts/check-setup.sh
bash act/scripts/start-demos.sh
```

Camera: passed, 640×480 MJPEG; short read check measured 15 FPS, device reported
30 FPS. This is capture throughput, not a completed 10 Hz demonstration test.

Arm: both commands correctly refused recording because the measured positions
were `[2378, 1752, 2112, 1580, 3522, 2054]`. Joint 4 is outside the saved
2438–3233 range. No limits were changed and no motion/torque writes were issued.
`guidance_verified` remains false in the Pi configuration.

Software: four automated tests passed, including the actual recorder loop with
simulated camera/servo inputs. It saved one successful episode, discarded another,
discarded an unfinished episode on quit, and generated the review video and joint
plot. Simulated episodes used temporary directories and were deleted; they are
not demonstration data. Failure-read and joint-limit rejection also passed.

Live collection remains `REQUIRES_HARDWARE_TEST`: resolve joint 4's pose or
calibration discrepancy, verify supported manual gripping/guidance and the wrist
view, then record and review one real complete pick-and-place. ACT training and
policy inference are not part of this collection test.
