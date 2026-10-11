# Manual placement corrections

Use the current ACT checkpoint to reach the late placement state, then record
only successful human recoveries. This extends the existing imitation-learning
dataset; it does not run reinforcement learning or automatically retrain ACT.

## Run the demo and automatically start corrections

From the PC repository terminal:

```bash
bash act/scripts/demo-and-correct.sh
```

This uses `act/outputs/kaggle-trained/policy`, returns to the saved initial pose
through the existing policy runner, executes **342 motor steps** (50 fewer than
the previous 392), then opens the correction recorder in the same terminal.
The arm holds until you support it and press **h**. The policy run name is
automatically saved in the correction metadata. Corrections stay on the Pi in
`act/data`; policy diagnostics are saved on the PC in `act/outputs/correction-demo-*`.
The existing standalone `run_policy.py` retains its own default; this shortened
default applies to `demo-and-correct.sh`.

Pass `--steps N` to choose an earlier handover, or `--checkpoint PATH`,
`--output PATH`, and `--host HOST` to override the defaults. A failed or
interrupted policy process does not automatically launch correction recording.
The transition is based on step count, not automatic detection of task failure.

## Start corrections after a separately run demo

After a supervised policy attempt has stopped and is holding, leave the object
and arm where they are. From the PC repository terminal:

```bash
ssh -t pi 'cd ~/mobile-robot && bash act/scripts/start-corrections.sh'
```

1. Support the arm. Press **h** to disable torque and start recording.
2. Guide the held object into the container, open the gripper by hand, then
   withdraw it. Avoid prolonged pauses while recording.
3. Press **f** only after successful placement and release. Press **d** for an
   unsuccessful correction. Press **q** to exit.

Both cameras and measured joints are recorded at the configured 10 Hz using
JPEG images and gzip joint samples in `act/data/episode-*`. Successful episodes
use the existing training format and are marked `assistance.mode=manual-correction`.
Use `--source-run observed-demo-...` to associate them with an autonomous run.
The autonomous approach, failed actions and the wait before **h** are excluded.
There is no joystick, saved-start-pose return or replay in this script.

Before **h**, the recorder leaves torque unchanged. Afterwards torque remains
off, including on exit/Ctrl+C. Keep supporting the arm as needed. Controllers
remain stopped after exit, as with the existing demo recorder.

Start with a small batch of about ten successful recoveries across the actual
failure states. That is a collection target, not a guaranteed sufficient number.
Include corrections beginning before a bad release; if the object was already
dropped, record recovery from its actual location rather than moving it without
recording. Keep the same object initially and vary reachable placement offsets.

## Train and evaluate afterwards

Mix successful corrections with the original successful full demonstrations.
Keep corrections from the same original policy attempt together in a train or
validation split; never split frames randomly. The current trainer splits by
episode, so related attempts require an explicit grouped split before training.
Do not use discarded/invalid corrections or autonomous failures as expert targets.
Inspect saved camera/joint alignment before packaging for Kaggle.

Retrain and compare physical pickup, placement and release success on held-out
trials. Recording alone does not update the deployed model. Corrective data
cannot compensate for an unreachable container or severe actuation errors.

RL remains an option if corrections plateau, but requires a suitable reward,
reset procedure and online robot training. See [HIL-SERL](https://hil-serl.github.io/)
and [interactive imitation learning](https://arxiv.org/abs/2402.17768).

Status: software checks only; **REQUIRES_HARDWARE_TEST** for hand-guided collection.

If a servo reports a fault during the ACT run, the combined command stops before
opening corrections. The policy endpoint reports its servo ID, joint name and
SDK error description, saves the original error in `movement-report.json`, and
closes cameras/serial without further motor commands or torque changes during
cleanup. A faulted motor's actual torque state is not assumed. Status 32 means
overload, not serial communication loss; the motor fault must be resolved before
another autonomous attempt.

2026-10-11 fault investigation: the shortened attempt executed 250 steps before
snapshot feedback failed. A subsequent read-only inspection found status 32 on
servo 6 (`gripper`), status 0 on servos 1–5. No motion or torque commands were sent
for this inspection. Fault reporting and cleanup passed mocked tests; recovery
and the full handover remain **REQUIRES_HARDWARE_TEST**.
