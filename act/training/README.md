# ACT training tools

Training ran on Kaggle using its free GPU allowance because no local GPU was
available. The experiment used 56 demonstrations followed by 18 hand-guided
corrections. Neither policy achieved reliable pickup and placement.

## Recorded-data training

`train.py` trains LeRobot ACT from both camera views and six joint positions.
It uses absolute future joint ticks, 50-action chunks at 10 Hz, ImageNet image
normalization, and whole-episode validation splits. Normalization statistics
come only from training episodes.

`package_training.py --episodes-file episodes.txt` exports selected successful
recordings, resized images and portable GPU launchers. It removes the recorded
stationary replay-to-hand handover pause without changing the original data.

`prepare_corrections.py` prepares the original demos and correction segments;
`package_corrections.py` bundles them with the previous policy. Fine-tuning
preserves the saved normalization, samples original demos and corrections
50/50, and uses 2,000 steps at a learning rate of 0.00001. Long quiet periods
in corrections are shortened; removal ranges remain in the exported metadata.
Validation keeps original-demo and correction errors separate. These errors
measure joint prediction, not successful grasping.

`train-corrections-kaggle.ipynb` contains the Kaggle workflow. It takes the
private correction-training ZIP and returns `act-trained-results.zip`.
The portable packages also support a separate NVIDIA GPU machine through
`start.sh` or `start.bat`; their included README describes the launcher.
Datasets, weights, environments and output archives are kept out of Git.

## Source checkpoint audit

`smoke_test.py` separately checks a public pretrained checkpoint on CPU;
`source.json`, `requirements-cpu.lock` and `source-audit/` preserve its revision,
environment and measured results. This was a compatibility check, not the
policy trained on Homer's recordings.

The saved laptop run loaded weights and processors and returned finite
`[1, 100, 6]` action chunks. Median latency was 387 ms and p95 was 478 ms
with four CPU threads. It did not test physical grasping. Source joint units
and calibration cannot be assumed to match Homer's raw servo ticks.
