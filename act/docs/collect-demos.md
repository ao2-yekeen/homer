# Collect demonstrations

All collection code, settings and data live in `act/`. This is a standalone
Python recorder using the installed Feetech SDK; ROS and LeRobot are not needed
for collection. Run on the Pi, where the arm and USB wrist camera are connected.

## First setup

Dependencies: Python 3, OpenCV, PyYAML, NumPy and the existing `STservo_sdk`.
These are already available on the current Pi. Copy `config/rig.example.yaml`
to `config/rig.local.yaml` on a new installation and check the device paths and
SDK path. Recording accepts raw servo positions from 0 to 4095 ticks. Earlier task-specific
motion limits do not apply to hand-guided recording.

Stop competing controllers and check the devices:

```bash
systemctl --user stop soarm-bridge robot-teleop
cd ~/mobile-robot
bash act/scripts/check-setup.sh
```

The check saves `act/data/camera-check.jpg` and prints camera rate, positions
and torque. Inspect the image: the wrist view must show the object and gripper.
The recorder refuses invalid raw tick readings or powered joints.
It never changes torque or writes servo positions.

With the arm supported and torque already off, gently test guidance and manual
gripper closing: can you lift and place the object? Park the arm on a stable
support at the end, leaving torque off. This is the manual exit procedure for
this recorder. Do not restart the bridge or restore torque as part of collection;
its old goals may cause movement. If guidance or manual gripping does not work,
stop here; powered gripper controls need a separate tested setup.

After checking guidance and gripping, start collection. No configuration approval
flag is needed; the recorder checks torque is off on all six servos.

## Record continuously

From your laptop, open an interactive Pi session:

```bash
ssh -t pi
cd ~/mobile-robot
bash act/scripts/start-demos.sh
```

Keys work in the terminal:

- `s`: start a new demo; guide approach, grip, lift, place and release.
- `f`: save a successful demo. Reset the object, then press `s` again.
- `d`: discard the current demo after a miss or interrupted movement.
- `q`: finish the session. An unfinished demo is discarded.

Repeat without restarting the script. Each demo gets a unique folder under
`act/data/episode-*`; previous sessions are retained. Reset movements between
`f` and the next `s` are not recorded. Use one easy object first and review one
complete demo before collecting a batch. With a local graphical Pi desktop,
run `python3 act/runtime/record.py` instead for a live camera window and the
same keys in that window.

Support the arm throughout guidance. On errors or exit, it remains torque off;
park it on its support before letting go. The script does not provide powered
holding or an automatic return pose.

## Review a demo

Replace the example folder with the path printed when saving:

```bash
python3 act/runtime/record.py --review act/data/episode-EXAMPLE
```

Open `review.avi` and `joints.png`. Check the full task, gripper motion, wrist
visibility, guiding-hand obstruction and image/joint alignment. `metadata.json`
reports the actual sample rate and outcome. Discard unsuccessful demonstrations;
future export must use only episodes labelled `success`.

## Handover

- `scripts/start-demos.sh`: starts the terminal recorder from any directory.
- `scripts/check-setup.sh`: read-only camera and arm checks.
- `runtime/record.py`: device ownership, recording, failure handling and review.
- `config/rig.example.yaml`: six ordered joints and camera settings.
- `config/rig.local.yaml`: machine settings; ignored by Git.
- `data/episode-*`: JPEG images, timestamped `samples.jsonl`, and `metadata.json`.

Each sample contains all six measured positions, the host camera-read interval,
and the joint-read interval on the same monotonic clock. JPEGs are ordinary RGB
images; OpenCV supplies BGR input to its encoder. Sensor exposure timestamps
are unavailable, so camera buffering/alignment still needs visual inspection.
Communication errors invalidate the active episode; previous positions are never
reused. Training/export is the next block of work: these are raw demonstrations,
not yet a LeRobot training dataset or action labels.

Verified on the Pi: all six servos responded, torque was off on all six, and
the generic USB camera captured 640×480 MJPEG at about 19 FPS in a short read
check (device reports 30 FPS). Recording requests 10 samples/s; validate the
actual rate in the first episode. The saved camera image currently shows the
room rather than the grasp area; reposition/check the wrist view. Teleoperation
was stopped for sole arm access. No servo movement or torque writes were made.

Hardware status: `REQUIRES_HARDWARE_TEST` for supported guidance, manual gripping
and one complete pick-and-place. The user confirmed the old joint ranges belonged to another task; they no longer
restrict recording.
