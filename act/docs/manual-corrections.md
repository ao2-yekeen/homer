# Manual placement corrections

Use the current ACT checkpoint to reach the late placement state, then record
only successful human recoveries. This extends the existing imitation-learning
dataset; it does not run reinforcement learning or automatically retrain ACT.

## Fresh predictions for every demo

Run from the PC repository terminal:

```bash
bash act/scripts/demo-and-correct.sh
```

Each attempt returns to the saved initial pose and runs ACT using that attempt's
camera observations. It executes **322 actions** by default: the configured
342-action duration minus `--steps-before-end 20`. It holds at that attempt's
own endpoint, then prompts for **h**. It does not reuse another demo's pose.
This is a fixed action-count cutoff, not automatic detection of task completion.

Support the arm, press **h** to release torque and record the manual finish,
then **f** to save successful placement and release. After saving, reset the
object/container and press **Enter** at the PC prompt for the next fresh attempt.
The arm stays torque-off after your manual finish until the next attempt starts.
**q** quits. Failed/interrupted ACT runs do not open the correction recorder.
Only the manual segment is saved as expert correction data; each correction's
metadata names its own autonomous source run. Corrections remain on the Pi in
`act/data`; autonomous reports/images remain under the PC output directory.

`--steps` sets the full duration before the offset is subtracted; `--checkpoint`,
`--output` and `--host` select the policy, output prefix and Pi. `--run-act` is
accepted explicitly but fresh ACT is already the default.

### Optional saved-run replay

To reuse a specific saved trajectory instead of generating fresh predictions:

```bash
bash act/scripts/demo-and-correct.sh --source-run act/outputs/correction-demo-20261011-120002-062436
```

This reaches that run's initial pose and replays its saved commands through
step 322, holds at its saved measured endpoint, then prompts for **h**. After
**f**, this explicit saved-run mode returns directly to the same endpoint for
another correction. **q/x** cancels positioning or replay. Saved replay cannot
adapt to a changed object position. Change the cutoff with `--steps-before-end`.

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
Without `--start-pose`, this direct command records from the current pose and
does not return automatically. There is no joystick.

In current-pose mode, torque stays unchanged until **h**, then stays off. In
saved-pose mode, successful returns hold with torque on until **h**. Exit while
waiting preserves the current torque state; stopping during a return uses the
existing return routine to release torque. Keep supporting the arm as needed. Controllers
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

2026-10-11 repeated-correction update: saved final-chunk pose selection,
startup positioning and return after saving passed software tests. No physical
motion was executed for this update; **REQUIRES_HARDWARE_TEST**.

2026-10-11 near-end selection: default offset is 20 motor steps, using saved
per-step measured feedback rather than 50-step chunk snapshots. Software tests
passed; new positioning remains **REQUIRES_HARDWARE_TEST**.

2026-10-11 startup replay fix: startup now reaches the saved initial pose and
replays the saved commands through the selected handover step before prompting
for h. Successful saves still return directly to the handover. Twelve software
tests passed; **REQUIRES_HARDWARE_TEST**.

2026-10-11 per-attempt handover: fresh ACT predictions are now the default for
each demo. The recorder exits after each successful correction; the PC starts
a fresh attempt after scene reset. Fourteen software tests passed. Physical
validation remains **REQUIRES_HARDWARE_TEST**.
