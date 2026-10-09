# Simple ACT training plan

Train a fresh ACT policy with official LeRobot code and an ImageNet-pretrained
ResNet-18 visual backbone. Use the USB wrist camera and hand-guided demonstrations
from Homer's vertically mounted SO-101. Start with one easy foam block, a fixed
tray and the established raised platform, with the base stationary.

## Steps

1. **Set up ACT.** Use the tested LeRobot version and CPU environment. The
   existing checkpoint smoke test passed (387 ms median, 478 ms p95), but tested
   community weights are not the training baseline. Initialize a fresh policy
   with the pretrained visual backbone and Homer-derived normalization.
2. **Get recording working.** Check the USB wrist view, test supported hand
   guidance and gripper operation, then use one Pi recorder to save images,
   six measured joint positions in raw ticks and timestamps. Start at 10 FPS.
   Inspect one complete pick-and-place before proceeding. See the
   [step 2 instructions](step-2.md).
3. **Collect demonstrations.** Record five successful episodes and inspect them.
   Then aim for about 50 clean demonstrations across five nearby object positions.
   Keep the camera, tray and start pose consistent. Discard failed or incomplete
   episodes. Keep a few whole episodes aside for validation; never randomly
   split neighboring frames.
4. **Prepare and train.** Export to the pinned LeRobot format. State is measured
   ticks; actions are future measured positions, with an explicit tested offset
   and no crossing of episode boundaries. Compute mean/std statistics from the
   training split. Use a Kaggle GPU if available, with Colab as backup. Start
   with batch size 8 and a 200-step smoke run; reload its checkpoint, then train
   within available compute. Save weights and resumable training state.
5. **Try the trained policy.** Preview predicted targets without motion first
   and benchmark the trained configuration. Then test slowly under supervision,
   using existing joint limits, fresh feedback and an immediate stop. ACT predicts
   absolute target ticks; the existing accumulated-delta topic cannot be used
   as an absolute interface. Add a small bounded target adapter when implementing
   this step. Use short chunks and replan; measure timing before selecting the
   execution rate or chunk length.
6. **Improve and evaluate.** Record successful corrections for repeatable failures,
   retrain, and run 20 supervised trials at withheld positions. Record success,
   misses and slips. Freeze the camera setup and checkpoint after evaluation.
   Expand to more objects only once the first task works consistently.

## Minimum files to implement

| File | Purpose |
| --- | --- |
| `act/runtime/record.py` | USB camera, joint readback, start/stop/discard and explicit teaching controls in one program |
| `act/config/rig.example.yaml` | Camera device, 10 FPS, joint order, existing limits and tested gripper settings |
| `act/runtime/export_dataset.py` | Check recordings, create future-position labels and export LeRobot episodes |
| `act/training/kaggle_train.ipynb` | Pinned installation, fresh ACT setup, training and checkpoint download |
| `act/runtime/run_policy.py` | Preview, CPU timing and bounded supervised execution |

Reuse the existing SDK and controller limits. Add only checks that catch real
failures: incomplete readback, frame/state alignment, episode-boundary labels,
invalid targets and stopping. Ignore local recordings, configuration and weights
in Git. Keep credentials in notebook secrets or environment variables.

## Essential conditions

Only one process owns the servo bus. Other controllers and automatic watchdog
writes must be inactive while physically guiding the supported arm. Restore
torque only through a verified procedure that cannot move toward stale targets.
Keep the stop path during policy execution. A disconnected camera or failed
joint read invalidates a recording; old values must not be reused as new data.

Raw ticks avoid a new coordinate-calibration layer. Preserve the joint order:
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper. Keep
this order and representation identical between training and execution.

No custom ROS topics, standalone camera publisher, generalized teaching state
machine, diagnostic dashboard or multi-rate benchmark is required initially.
The recorder can access the SDK directly after the existing bridge is stopped.
If an implementation uses ROS, follow `pi/AGENTS.md`.

The current hardware work is `REQUIRES_HARDWARE_TEST`. Step 1's saved report is
software evidence, not proof of grasp performance. Free cloud allocation depends
on account availability. Recording at 10 FPS does not imply fresh CPU predictions
at 10 FPS; chunk execution and feedback delay must be tested in step 5.

References: [official ACT code and example](https://github.com/huggingface/lerobot/tree/v0.4.4/src/lerobot/policies/act),
[existing software check](../training/README.md).
