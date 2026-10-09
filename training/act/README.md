# ACT checkpoint compatibility check

This standalone tool has no ROS, serial, camera capture or motion interface.
It downloads the eight pinned checkpoint files, checks the schema, strictly
loads all model weights and both saved processors on CPU, and measures fresh
100-action chunks, including preprocessing and decoding. Cached action dequeue
time is not reported as inference speed.

## Run

Use Python 3.10 on Linux x86-64 for the tested CPU environment. Install `uv`
using its official installer or an existing Python environment, then:

```bash
uv venv --python python3.10 .venv-act
uv pip sync --python .venv-act/bin/python training/act/requirements-cpu.lock --index-strategy unsafe-best-match
.venv-act/bin/python training/act/smoke_test.py
```

First run requires network access and downloads the model to the Hugging Face
cache. Model weights and local reports are ignored by Git. Run from the repository
root. Optional saved inputs:

```bash
.venv-act/bin/python training/act/smoke_test.py --image wrist.jpg \
  --state 0 0 0 0 0 50 --iterations 20 --threads 4
```

Those state values are illustrative source-policy coordinates, **not servo
ticks**. Default input is constant RGB and the checkpoint's mean state; it checks
software execution, not grasp quality. Report location defaults to
`outputs/act/smoke.json`. Invalid shapes, nonfinite outputs and incompatible
weights fail the run. Timing is local to the host and configured thread count.

## Source representation audit

Model revision and expected LeRobot version are in `source.json`. Public source
dataset metadata and its immutable revisions are saved in `source-audit/`.
Both datasets named by the model card confirm 30 FPS and this order for state
and action: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll,
gripper, each named `.pos`. That supports position targets rather than tick
deltas, but metadata alone does not establish calibration, units, signs or
physical gripper opening.

The model card names expert and HIL datasets, whereas `train_config.json` names
`TakuyaHiraoka/so101_pick_diverse_objects_v2`. The latter's metadata returned
HTTP 401 without authentication. This may mean private/unavailable access;
the exact training dataset cannot be independently audited yet. Do not silently
treat the public datasets as identical to the training dataset. Do not reuse
source joint/action statistics for Homer without measured conversion and new
training statistics.

The source config's author-local pretrained path and CUDA processor are
overridden in memory. No source files are rewritten. Backbone initialization
does not download ImageNet weights because strict loading supplies the complete
backbone. This starts no optimizer and performs no training.

## Acceptance

Software gate: full weights and processors load, decoded output is finite with
shape `[1, 100, 6]`, and repeated fresh-chunk latency is measured. The measured
p95 gives a fresh-chunk frequency estimate; it does not determine command FPS.
Control FPS remains unset until recording throughput, transport delay and
guarded execution are measured. The vertical rig and source calibration still
require compatibility validation before execution.

Sources: [model and card](https://huggingface.co/TakuyaHiraoka/act_so101_pick_diverse_objects),
[ACT implementation](https://github.com/huggingface/lerobot/tree/v0.4.4/src/lerobot/policies/act),
[expert dataset](https://huggingface.co/datasets/TakuyaHiraoka/so101_pick_diverse_objects),
[HIL dataset](https://huggingface.co/datasets/TakuyaHiraoka/so101_pick_diverse_objects_hil_round1).

## Measured result

The saved `source-audit/cpu-smoke-result.json` records a successful run on
this WSL Linux laptop with four CPU threads and ten measured iterations after
two warmups. Median fresh-chunk latency was 387 ms, p95 478 ms (about 2.09
fresh chunks/s). Full strict weight loading, source mean-state normalization
and finite decoded `[1, 100, 6]` output passed. This does not support fresh
prediction at 20–30 Hz on this host. Buffered chunk execution may use a higher
command rate, but its feedback delay and safe chunk handling must be measured
in the later execution step. No arm or camera hardware was accessed.
