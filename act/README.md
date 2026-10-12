# ACT grasping work

This directory contains Homer's ACT demonstration recorder, data-preparation and
training tools, and the experiments that may eventually connect a learned policy
to the arm. Collection runs directly on the Raspberry Pi using the Feetech SDK;
it does not need ROS or LeRobot. Policy training runs separately on a PC.

## Current collection setup

The recorder uses two 640×480 RGB views (wrist and Logitech overview), six raw
servo positions in this order—shoulder pan, shoulder lift, elbow, wrist flex,
wrist roll, gripper—and a nominal 10 samples/s. On a new Pi, install the
recorder dependencies (Python 3, OpenCV, PyYAML, NumPy, and the existing
`STservo_sdk`), copy `config/rig.example.yaml` to the ignored
`config/rig.local.yaml`, then set the camera, serial, and SDK paths for that
machine. Check that the wrist view sees the object and gripper and the overview
view sees the workspace.

Before opening the arm, support it and stop competing controllers. From a
laptop, connect to the Pi with a terminal that supports key input:

```bash
ssh -t pi
cd ~/mobile-robot
systemctl --user stop soarm-bridge robot-teleop rgb-object-detection
bash act/scripts/check-setup.sh
```

The check is read-only: it captures test frames and reports all six joint
positions and torque states. The recorder will not start unless torque is off.
Keep the arm supported whenever torque is released; do not restart the arm
bridge during collection because an old goal can move the servos.

## Manual hand-guiding attempt

The attempt to guide the arm by hand did not work, so no successful hand-guided
demonstration was collected. The scripts below describe the intended recording
flow; they do not make the arm mechanically back-drivable or prove that this
setup can be guided. The available notes do not identify why the attempt failed.
Resolve that hardware issue before trying to collect demos.

If hand guidance becomes workable, the intended flow is to teach an approach,
then record demos that replay it before a hand-guided final alignment and grasp:

1. Run `bash act/scripts/record-approach.sh`. Press `s` to return to the saved
   start pose and begin recording. Torque is on only during that return; with
   torque off, support and guide the arm toward the object, stopping before
   contact. Press `p` to save the six-joint approach trajectory. This source
   stores joint positions and timestamps only, with no camera images.
2. Run `bash act/scripts/start-demos.sh`. Press `s` for each attempt; it returns
   to the saved start pose, records both cameras and joint feedback, then replays
   the saved approach. At `APPROACH COMPLETE`, support the arm and press `h` to
   release torque. Hand-guide the final alignment, grip, lift, placement and
   release while recording. Press `f` to save a successful episode, `d` to
   discard it, and `q` to exit. Reset the scene before the next `s`.

Both scripts stop the arm bridge and teleop services before opening devices and
leave them stopped on exit. Keep the terminal open and the workspace clear.
For full hand-guided recording without an approach replay, see
[collection instructions](docs/collect-demos.md).

## What the setup attempt established

A live Pi check read all six servos with torque off and captured both 640×480
camera streams. Continuous recording saved samples at about 10 Hz, and video
and joint-plot review worked. Those test episodes were discarded; they establish
that sensing and logging work, not that hand guidance works. A return-to-start
smoke test began with the arm already at its saved pose, so travel from another
pose was not tested. Hand-guiding, manual gripper operation, and a complete
pick-and-place remain unresolved hardware problems; see
[collection test notes](docs/collection-test.md).

## Training and layout

Start with the [training setup](training/README.md) for the checkpoint smoke
test and portable training package. The [plan](docs/plan.md) describes the
remaining dataset, training and policy-execution work. Recordings, local rig
configuration, virtual environment and model outputs are ignored by Git.

```text
act/
  config/     Example and local rig settings
  data/       Local recordings and datasets (ignored)
  docs/       Setup, collection instructions and test evidence
  outputs/    Local checkpoints and reports (ignored)
  runtime/    Recorder, review and policy utilities
  scripts/    Setup checks and Pi launchers
  training/   Packaging, training and policy tools
  .venv/      Local Python environment (ignored)
```
