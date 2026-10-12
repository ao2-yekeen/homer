# Train the Homer ACT policy

Extract the package and run `bash start.sh` on Linux or `.\start.bat` on
Windows. Requires an NVIDIA GPU with a CUDA 12.8 compatible driver, internet
for initial downloads, and about 15 GB of free disk space. No robot is accessed.
Python, dependencies and caches are installed inside the extracted folder.

The original-demo package trains a fresh policy for 20,000 steps. The correction
package includes `dataset.json` and `initial-policy/`: it fine-tunes that policy
for 2,000 steps at learning rate 0.00001, preserving normalization and sampling
original demonstrations and corrections 50/50. Both use two cameras and six
joint positions, with whole episodes held aside for validation.

Progress is saved in `output/terminal.log`; checkpoints save every 200 steps.
Run the same command to resume an interrupted run. Ctrl+C requests a checkpoint;
allow saving to finish. Keep the machine awake. Add `--batch-size 1` to reduce
GPU memory use or `--steps 200` for a short trial.

The result is `act-trained-results.zip`, containing the best validation policy,
processors, dataset split and logs. Resumable optimizer state stays in
`output/latest`. Validation error does not establish physical grasp success;
Homer's experiment did not achieve reliable pickup and placement.
