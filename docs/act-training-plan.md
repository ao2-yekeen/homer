# Wrist-camera ACT adaptation plan

## Objective

Train a fresh ACT policy using the official LeRobot implementation and an
ImageNet-pretrained ResNet-18 visual backbone on hand-guided demonstrations
from Homer's vertically mounted SO-101. Demonstrate a learned
pick-and-place policy with the base stationary, one foam block, a small raised
platform region, and a fixed destination tray. No leader arm or gamepad
demonstrations. Train on Kaggle's free GPU, with Colab as backup.

Use the USB wrist camera, separate from the Logitech camera. Initialize the
action policy freshly and compute normalization statistics from Homer data.
The community checkpoint is retained only as an optional comparison.

This is an implementation plan, not a validated hardware capability. Freeze
the setup after evaluation and corrective training.

## Optional comparison checkpoint (not the training baseline)

The published checkpoint has one `observation.images.wrist` RGB input with
shape `[3, 480, 640]`, `observation.state` shape `[6]`, and `action` shape `[6]`.
It uses ACT with ResNet-18 and 100-action chunks. Its processors use mean/std
normalisation. Its config includes an author-local `pretrained_path`; replace
that through the supported loading configuration rather than following it.

Download and pin the complete repository revision:

- `model.safetensors`
- `config.json`
- `policy_preprocessor.json`
- `policy_preprocessor_step_3_normalizer_processor.safetensors`
- `policy_postprocessor.json`
- `policy_postprocessor_step_0_unnormalizer_processor.safetensors`
- `train_config.json` and `README.md` for provenance

The published training config is not an optimizer-state checkpoint. Start a
new fine-tuning run, not a resume of the author's run. Verify the source dataset
joint order, position representation, gripper scale, and FPS before deciding
which processors to reuse. Do not assume six values imply compatible units.

## Proposed files

| File | Responsibility |
| --- | --- |
| `config/act/rig.example.yaml` | Wrist camera identity, FPS, joint order, measured calibration, teaching limits, task and local host settings; unmeasured values remain unset |
| `config/act/train.yaml` | Pinned software and visual backbone weights, fresh ACT configuration, Homer dataset, batch size, learning rates, checkpoints and training budget |
| `pi/wrist_camera.py` | Stable camera-device selection; timestamped RGB publication and stream diagnostics |
| `pi/soarm_ros_bridge.py` (modify) | Timestamped higher-rate measured positions; explicit supported teaching lifecycle; bounded absolute-target path; preserve existing jog control and watchdog |
| `pi/act_learning/representation.py` | Joint ordering and invertible tick/policy conversions; no guessed zeros or directions |
| `pi/act_learning/record.py` | Start, stop, discard and label episodes; synchronise measured joints, images, gripper events and teaching status |
| `pi/act_learning/export_dataset.py` | Convert raw demonstrations to the pinned LeRobot dataset format; construct future-position action labels and split whole episodes |
| `pi/act_learning/check_dataset.py` | Detect missing joints, stale frames, timestamp gaps, invalid limits, inconsistent rates and unsuccessful episodes; produce inspection report |
| `training/act/requirements.lock` | Reproducible tested training dependency set |
| `training/act/kaggle_train.ipynb` | GPU check, installation, data staging, model adaptation, short benchmark, training and persistent checkpoint export |
| `training/act/train.py` | Initialize fresh ACT with pretrained visual backbone; configure Homer schema and normalisation; start a fresh optimizer and save complete resumable checkpoints |
| `pi/act_learning/run_policy.py` | CPU/laptop inference benchmark, default command-free preview, chunk handling and explicit guarded execution |
| `pi/act_learning/evaluate.py` | Record initial position, outcome, failure reason, timing and model revision for each trial |
| `pi/tests/test_act_representation.py` | Conversion round trips, gripper direction and joint ordering |
| `pi/tests/test_act_dataset.py` | Action alignment, episode boundaries and rejection of stale/incomplete recordings |
| `pi/tests/test_act_execution.py` | Absolute-target semantics, feedback freshness, stop conditions and chunk cancellation |

Update `.gitignore` for local rig configuration, recordings, datasets,
downloaded weights, outputs and notebook execution artifacts. Secrets belong
in notebook secrets or environment variables, never committed files.

## Work order and acceptance gates

1. **Compatibility and inference smoke test.** The optional comparison
   checkpoint has been tested; fresh ACT is now the training baseline. Pin a tested LeRobot
   version and model revision. Inspect source action semantics. Load the full
   checkpoint and perform command-free inference on sample RGB/state input
   on the available laptop. Measure latency before choosing control FPS.
2. **Teaching and sensors.** Verify camera mount and cables over the
   intended motion. With the arm supported, verify torque management and
   readback while guiding it. Teaching mode excludes all motion writers and
   has an explicit supervised exit; it must not snap to a stale target on
   re-enable. Prototype timestamped joint readback at 20–30 Hz, measure actual
   throughput, and select a sustainable rate shared by recording and execution.
   Follow `pi/AGENTS.md` for every ROS entrypoint. See the
   [detailed step 2 plan](act-step-2-teaching-and-sensors.md).
3. **Pilot data.** Record five complete successful episodes. Inspect
   video/state alignment, future-position labels and gripper transitions.
   Keep the guiding hand out of the wrist camera's view as far as practical;
   assess contamination before collecting the full dataset. Manual measured
   positions are not leader-command labels: document the future-offset label
   construction and avoid zero-action same-frame targets.
4. **Dataset.** Aim for 50 clean episodes distributed across five
   nearby object positions. Reserve whole episodes and additional object
   positions for validation and physical evaluation. Do not split neighbouring
   frames randomly. Keep camera, tray, object orientation and start pose fixed.
5. **Cloud training.** Use one Kaggle GPU, initial batch size 8,
   reducing if necessary. Start with a 200-step smoke run and reload its saved
   checkpoint. Measure throughput and checkpoint size; choose the subsequent
   step budget from available quota. Use local runtime disk for training data
   and persistent storage for weights plus optimizer/RNG/step state. Rebuild
   Homer state/action statistics explicitly; verify preprocessing numerically.
6. **Guarded evaluation.** Convert predicted absolute joint targets
   correctly; the existing delta topic accumulates relative to prior targets,
   so it cannot be treated as a direct absolute-position interface. Recheck
   every action against measured feedback and limits, reject stale inputs,
   cancel queued actions on stop, and retain the hardware watchdog. Joint
   bounds do not prove collision-free paths. Report publication separately
   from observed movement and successful grasping.
7. **Corrections and freeze.** Score at least 20 supervised trials at
   withheld positions. Add successful corrective demonstrations for observed
   failures, retrain and re-evaluate, then freeze camera, rig and checkpoint.
   Report the measured success rate; 18/20 is a target, not an established
   result or a statistical reliability guarantee.

## Step 1 implementation status

The command-free checkpoint loader and CPU benchmark are implemented in
`training/act/smoke_test.py`, with pinned model revision and tested CPU
dependencies. See `training/act/README.md` for commands and the saved report.
Strict loading of the complete model and saved processors passed; output was
finite with shape `[1, 100, 6]`. Median latency was 387 ms and p95 478 ms on
the local laptop with four CPU threads. Control FPS remains unset pending
sensor throughput and guarded-execution measurements.

The public datasets named in the card confirm joint order and 30 FPS. The
training config names a different `_v2` dataset whose metadata returns HTTP
401 without authentication. Exact source units and gripper calibration remain
unverified; transfer compatibility is not established by the software test.

## Blocking facts to resolve first

- Safe supported hand guidance, gripper teaching and servo readback have not
  been physically verified.
- Wrist-camera identity, mounting, visibility and cable clearance are unknown.
- Homer policy coordinates and gripper calibration need a documented mapping.
  Source-to-Homer calibration matters only for the optional comparison.
- Sustainable recording FPS is unmeasured. The comparison checkpoint CPU
  latency is measured; the final trained configuration must be benchmarked.
- Free GPU allocation and quota depend on the user's account and availability.

No new boot service, firmware change, base navigation, multi-object selection,
or wrist-camera purchase is required by this plan. Start with one object;
expand only after measured success.

## Sources

- [Source model](https://huggingface.co/TakuyaHiraoka/act_so101_pick_diverse_objects)
- [Source configuration](https://huggingface.co/TakuyaHiraoka/act_so101_pick_diverse_objects/blob/main/config.json)
- [ACT documentation](https://huggingface.co/docs/lerobot/act)
- [Kaggle GPU guidance](https://www.kaggle.com/docs/efficient-gpu-usage)
- [Colab limitations](https://research.google.com/colaboratory/faq.html)
