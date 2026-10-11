# Train the corrected robot policy

Extract the ZIP, then open a terminal inside `act-correction-training`.

Windows:

```bat
.\start.bat
```

Linux:

```sh
bash start.sh
```

Requires an NVIDIA GPU with a CUDA 12.8 compatible driver, internet for initial
setup, and free disk space for Python, GPU dependencies and checkpoints
(allow 15 GB). No robot or repository checkout is needed.
Python 3.11, dependencies and caches are installed locally in this folder.

This **fine-tunes the included previous Kaggle policy**, preserving its saved
normalization. It does not train from scratch. The 56 original demonstrations
and 18 hand corrections are already compressed and prepared. Each training
sample has a 50% chance of coming from corrections; held-out demonstrations
remain excluded. It runs 2,000 optimizer steps at learning rate 0.00001.

Progress, loss and ETA appear in the terminal and `output/terminal.log`.
Checkpoints save every 200 steps. Keep the terminal open and disable sleep.
If interrupted, run the same command to resume. Ctrl+C requests a checkpoint;
allow saving to finish. Add `--batch-size 1` if GPU memory runs out.

When complete, send back **act-trained-results.zip** from this folder.
It includes the best validation checkpoint, processors, dataset split and logs.
The larger optimizer state remains in `output/latest` for resuming.
Offline prediction accuracy does not establish physical placement success.
