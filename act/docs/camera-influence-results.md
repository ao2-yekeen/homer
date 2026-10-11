# Offline camera influence results — 2026-10-11

Step 1 of [the reliability plan](policy-reliability-plan.md) is complete.
Both cameras affect this checkpoint's action predictions. This does **not**
establish that it locates the object or container correctly, or that it can
complete placement. No robot commands were sent during this test.

Checkpoint: `act/outputs/kaggle-trained/policy`, weights SHA-256
`e2543e574cb236c9edc3982bf1afa8c4c075e53ad2dc4d9d98da611711491d09`.
Inputs: four snapshots from `act/outputs/observed-demo-20261011-111819`.
Each comparison holds all six joint inputs fixed and replaces only the indicated
images with images from the nearest validation joint state (Euclidean distance
in raw ticks). Each donor pair comes from the same validation frame.

Numbers below are mean absolute prediction changes across six joints, in servo
ticks: **first five actions / full 50-action chunk**. These are changes relative
to the original images, not errors against expert actions.

| Snapshot (commands already executed) | Donor joint RMS gap | Replace wrist | Replace Logitech overview | Replace both |
| --- | ---: | ---: | ---: | ---: |
| 000 (0) | 0.4 | 19.63 / 33.34 | 0.99 / 2.68 | 18.65 / 31.19 |
| 003 (150) | 34.5 | 26.64 / 35.96 | 28.68 / 67.27 | 39.72 / 63.07 |
| 006 (300) | 86.3 | 31.09 / 162.06 | 43.02 / 53.58 | 56.94 / 199.34 |
| 008 (392, finished) | 79.4 | 27.21 / 126.05 | 87.40 / 82.04 | 82.90 / 147.27 |

Identical-input repetitions changed predictions by **0 ticks** at every anchor.
The baselines also exactly reproduced the saved live-run predictions at snapshots
000, 003 and 006. Snapshot 008 had no original prediction because execution had
finished. Grey-image ablations also changed predictions; blanking both cameras
changed the first five actions by 28.61, 149.10, 150.62 and 181.63 ticks respectively.

Donors (episode, zero-based row):

| Snapshot | Validation episode | Row |
| --- | --- | ---: |
| 000 | episode-1791653260822254088 | 0 |
| 003 | episode-1791653858931103352 | 166 |
| 006 | episode-1791654045434196604 | 215 |
| 008 | episode-1791655757324584249 | 216 |

The donor views differ in robot pose, scene appearance and task stage, not just
object position. Single-camera replacements can also create inconsistent image
pairs. Blanks are outside the normal input distribution. Therefore these results
show image influence, but cannot distinguish useful visual feedback from reliance
on appearance or task-stage cues. They do not identify the cause of failed placement.

The next planned step is successful human recovery demonstrations from the late
failure states. A controlled object-position experiment with unchanged arm pose
would provide stronger evidence about spatial visual feedback before claiming
that the policy can adapt to position changes.

Reproduce from the repository root using the existing ACT environment and local
checkpoint, run snapshots and validation episodes:

```bash
act/.venv/bin/python -u act/training/check_camera_influence.py \
  --checkpoint act/outputs/kaggle-trained/policy \
  --run act/outputs/observed-demo-20261011-111819 \
  --data act/data \
  --output act/outputs/camera-influence/report.json
```

The JSON report stores fixed joint states, source image paths, donor identities,
all decoded predictions, per-joint changes, first-action/first-five/chunk metrics
and checkpoint identity. Large local data and output files remain ignored by Git.
