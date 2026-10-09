# Collection test results

Live Pi test after removing the old task-specific recording limits:

- Camera check: 640×480 MJPEG, short throughput check about 17 FPS.
- All six servo reads passed at `[2378, 1752, 2112, 1580, 3522, 2054]`.
- Torque read zero for every servo. Recorder issued no servo writes.
- The documented `bash act/scripts/start-demos.sh` ran through an interactive SSH terminal.
- Two episodes were started and discarded continuously in one session: 41 and
  49 samples at 9.97 and 9.98 samples/s. A third test saved 72 samples at 9.99 samples/s.
- Every saved image decoded at 640×480, had six associated joint readings and
  correctly ordered capture/read interval timestamps.
- Video and joint-plot review generated successfully on the Pi.
- Fixed buffered terminal input so rapidly typed `dq` discards and exits reliably;
  the third live test verified this.

All live tests are labelled `discard`: they verify capture, not a successful
hand-guided grasp. They must not be included in training. Five software tests
also pass, including save-success/discard/unfinished-exit with simulated inputs,
review generation, failed-read rejection, invalid-tick rejection and acceptance
of joint 4 at 1580 despite obsolete task limits. Simulated data is temporary.

Recording accepts valid raw ticks (0..4095); the user confirmed earlier ranges
were specific to another task. Existing shared motion-controller limits were
not changed. There is no configuration approval flag; torque-off is checked
before starting. Manual guidance, wrist visibility and successful physical
pick-and-place remain operator checks before collecting training demos.
