# Manual placement corrections

Use the current ACT checkpoint to reach the late placement state, then record
only successful human recoveries. This extends the existing imitation-learning
dataset; it does not run reinforcement learning or automatically retrain ACT.

## Repeat corrections without rerunning ACT

From the PC repository terminal:

```bash
bash act/scripts/demo-and-correct.sh
```

The command selects the most recently saved **completed** policy run. On startup,
it reaches that run's initial pose, then replays its saved motor commands up to
**20 motor steps before the end** (about two seconds at 10 Hz). It holds at the
saved measured handover pose and only then prompts for **h**. No ACT inference
is needed. Place the object and container as they were at the beginning of the
source run before starting.

Support the arm, then **h** releases torque and starts recording your manual
finish. **f** saves a successful correction and immediately returns directly to
the same handover pose, powered and holding. Reset the object and container to
match that pose before pressing **h** again. The approach is replayed once on
startup, not after each save. **q/x** cancels automatic motion; **q** exits the
recorder. A discarded correction (**d**) leaves torque off at the current pose.

Only manual motion between **h** and **f** is recorded; approach, return motion
and waiting are excluded. Saved metadata includes the source run, selected step,
initial pose, replay commands and correction pose. Corrections remain on the Pi
in `act/data`. Saved replay does not adapt to changed object positions.

Choose a particular existing run:

```bash
bash act/scripts/demo-and-correct.sh --source-run act/outputs/correction-demo-20261011-120002-062436
```

That completed 342-step attempt supplies its measured **step-322** pose from
`movement-report.json`, leaving 20 actions for manual completion. Change the
offset with `--steps-before-end 50`, for example. Faulted runs are excluded from
automatic selection. The selected pose may already be after release or during
withdrawal; the offset does not detect task phases.

To collect from a fresh policy attempt instead, use **--run-act**. This runs
342 steps by default (50 fewer than 392), then returns to its initial pose and replays to the selected handover for corrections. `--steps`, `--checkpoint` and `--output` apply to
this fresh-run mode; `--host` applies to both modes. Failed/interrupted fresh
runs do not open the correction recorder. No model is loaded in the default mode.

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
