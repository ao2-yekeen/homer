# ACT demonstration collection

This directory contains Homer's recorder and the later dataset/training tools.
Recording runs on the Raspberry Pi from camera frames and measured positions of
all six Feetech joints. ROS and LeRobot are not needed to record; training runs
separately on a PC.

## Set up the Raspberry Pi

The Pi needs Python 3, OpenCV, PyYAML, NumPy, the Feetech `STservo_sdk`, two USB
cameras, and the arm's serial adapter. The recorder uses the Python environment
that can import `cv2`, `yaml`, and `STservo_sdk`; `sdk_path` must point to the
folder containing the SDK. The SDK is already installed on Homer's Pi.

On a fresh Pi, first check out this repository at `~/mobile-robot`. From your
laptop, open an interactive Pi terminal, then prepare the local config and list
the connected devices:

```bash
ssh -t pi
cd ~/mobile-robot
cp act/config/rig.example.yaml act/config/rig.local.yaml
ls -l /dev/v4l/by-id /dev/serial/by-id
```

Edit `act/config/rig.local.yaml` (for example, with `nano`) for this machine.
Set `camera` to the wrist
camera, `overview_camera.device` to the Logitech overview camera, `serial` to
the arm adapter, and `sdk_path` to the SDK folder. Keep the six servo IDs and
joint order as supplied; set `fps` to the desired recording rate (default 10).
The local file is ignored by Git.

Support the arm, then stop other arm and camera owners and run the read-only
checks:

```bash
systemctl --user stop soarm-bridge soarm-bridge.path robot-teleop rgb-object-detection
bash act/scripts/check-setup.sh
```

The check saves `act/data/camera-check.jpg` and
`act/data/camera-check-overview.jpg`; inspect both views. It also reads all six
joint positions and torque states without commanding motion. Fix camera paths,
permissions, serial settings, or SDK imports before recording. The recorder
requires torque off at startup. Keep supporting the arm whenever torque is off;
do not restart the arm bridge during a session because an old goal may move it.

## Record demonstrations

**Hardware blocker:** an attempt to guide the arm by hand with torque off did
not work. No successful hand-guided demo was collected. Resolve this physical
issue before collecting data; the steps below describe the software workflow,
not a working hand-guiding setup.

Once the arm can be guided safely, teach a reusable approach source:

```bash
bash act/scripts/record-approach.sh
```

In the interactive terminal, press `s` to return to the saved start pose and
begin recording. Torque is on only for the return. With torque off, guide the
arm toward the object, stopping before contact, then press `p` to save and exit.
The source contains six joint positions and timestamps, but no camera images.

Then record full episodes:

```bash
bash act/scripts/start-demos.sh
```

Press `s` for an attempt. The script returns to the saved start pose, records
both cameras and joint feedback, then replays the approach source. At
`APPROACH COMPLETE`, support the arm and press `h` to release torque; guide the
final alignment and grasp while recording. Press `f` to save a success, `d` to
discard, or `q` to exit. Reset the object before the next attempt. The launcher
stops competing controllers before opening devices and leaves them stopped on
exit. Connect over SSH with `ssh -t pi` so the keys work.

The recorder stores raw 0–4095 servo ticks in shoulder-pan, shoulder-lift,
elbow, wrist-flex, wrist-roll, gripper order, alongside two 640×480 RGB views at
about 10 samples/s. Review an episode with:

```bash
python3 act/runtime/record.py --review act/data/episode-NAME
```

Inspect `review.avi`, `joints.png`, and `metadata.json`. Keep only complete,
successful episodes for training. Live setup tests confirmed sensor capture and
logging near 10 Hz, but those test episodes were discarded; the hand-guiding
attempt and a complete pick-and-place remain unsuccessful/unverified. See
[collection test notes](docs/collection-test.md) and the more detailed
[collection guide](docs/collect-demos.md).

## Training

For the checkpoint compatibility check and portable training package, see the
[training guide](training/README.md). The remaining dataset and policy work is
in the [ACT plan](docs/plan.md). Recordings, local rig configuration, model
weights, and reports are ignored by Git.
