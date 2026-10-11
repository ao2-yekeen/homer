# Fine-tuning ACT with hand corrections

Prepare compressed training copies from the PC's recordings:

```bash
act/.venv/bin/python act/training/prepare_corrections.py
```

Output: `act/outputs/correction-training/`. The output directory must be new;
use `--output` to create another version, or `--resume` to finish an interrupted
export. Original PC and Pi recordings remain
unchanged. This is an offline workflow; it does not access the robot.

The existing checkpoint's `dataset.json` supplies its original 50/6 episode
split. All those full demos are included without additional temporal cuts;
their previously applied replay-to-hand handover trimming is preserved.
The 18 successful manual corrections use a separate seeded 16/2 episode split.
Validation episodes never enter the training sampler.

Images use the previous training package's 320×240 JPEG quality-75 format.
Already sized JPEGs are copied byte-for-byte; larger images are resized once.
Joint rows use lossless gzip JSONL. Joint order, absolute tick units, both camera
inputs, next-frame targets, 10 Hz timing, chunk size 50, and padded end-of-episode
labels match the earlier model. Correction segments remain separate episodes;
no artificial trajectory is stitched between demonstrations.

## Reducing hesitation

For manual corrections only, quiet runs of at least one second are shortened
to five frames: three at the beginning and two at the end. Shorter pauses are
kept. The final frame is always kept. Moving sections are not resampled.

A run must stay within three ticks of its first joint pose and have little
image change in both cameras. Comparisons use both the first and previous
image, preventing cumulative slow movement from being treated as a pause.
Gripper changes greater than two ticks protect half a second on each side.
The camera heuristic compares 80×60 grayscale images: mean difference <=2.5
and no more than 2% of pixels changing by >20 intensity levels.

These are conservative experiment settings, not validated contact or object
settling detectors. Small object movement can be missed and hand motion can
cause extra frames to be retained. Acquisition timestamps and `source_frame`
remain available for auditing; the trainer uses retained frames as nominal
10 Hz steps, shortening only detected pauses. Every removed interval is
recorded in episode metadata and `preparation.json`.

Human corrective data and replay of earlier demonstrations follow the general
interactive imitation learning approach studied in
[DAgger](https://arxiv.org/abs/1011.0686) and
[Human-in-the-loop imitation learning](https://arxiv.org/abs/2012.06733).
The half-second cap and 50/50 sampling ratio are local starting settings;
those papers do not establish them as optimal for this robot. Hands visible
in correction images remain a deployment difference.

## Train on a GPU

With the same pinned environment as the previous run (`lerobot==0.4.4`,
`torch==2.7.1`, `torchvision==0.22.1`):

```bash
act/.venv/bin/python -u act/training/train.py \
  --data act/outputs/correction-training/data \
  --dataset-manifest act/outputs/correction-training/dataset.json \
  --init-from act/outputs/kaggle-trained/policy \
  --output act/outputs/act-corrections \
  --device cuda --steps 2000 --batch-size 8 --learning-rate 0.00001
```

This loads existing weights and saved pre/post processors, preserving their
normalization statistics. Both backbone and action model use the lower learning
rate. Each training example is sampled from corrections with probability 0.5,
otherwise from the original training demos, avoiding domination by the longer
original dataset. Checkpoints are saved every 200 steps. To resume, use the same
command with `--resume` and omit `--init-from`.

On Kaggle or a friend's machine, copy `data/` and `dataset.json` from the
prepared output, the entire original `policy/` checkpoint, and `train.py`.
Use that machine's Python and change the command paths, including `--init-from`.
The explicit checkpoint argument overrides the preparation machine's path.

Validation reports original-demo and correction errors separately and selects
`best/` using their equally weighted mean. `latest/` includes optimizer state
for resuming. These errors measure joint prediction, not placement success.
Before replacing the deployed model, compare original versus fine-tuned policies
on repeatable placement trials. Retaining the raw dataset also permits a
trimmed-versus-untrimmed comparison if hesitation or contact timing worsens.
