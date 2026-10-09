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
systemctl --user stop soarm-bridge robot-teleop rgb-object-detection
cd ~/mobile-robot
bash act/scripts/check-setup.sh
```

The check saves `act/data/camera-check.jpg` and
`act/data/camera-check-overview.jpg`. Inspect both images: keep the wrist view
on the object and gripper, and the Logitech overview view on the whole work area.
The read-only setup check reports arm positions and torque without changing
either. The recording command requires all servos to be torque-off when it starts.

With the arm supported and torque off, gently test guidance and manual gripper
closing: can you lift and place the object? Park the arm on a stable support at
the end. Do not restart the bridge during collection; its old goals may cause
movement. If guidance or manual gripping does not work, stop here; powered
gripper controls need a separate tested setup.

After checking guidance and gripping, start collection. On first startup, the
recorder saves the current six-joint pose as the session reference; later runs
reuse that saved pose. Remove `data/session-start-pose.json` to capture a new
reference. The recorder requires torque to be off at startup. Before each demo,
`s` stages the live servo positions as hold targets,
enables torque, returns toward the saved pose in small steps, and switches torque
off immediately after reaching it. Recording starts only if the measured pose
remains within 30 ticks on every joint after torque is released. Then guide the
demo by hand. Keep the work area
clear during the powered return; `q` or `x` aborts the return and releases torque.

## Record continuously

From your laptop, open an interactive Pi session:

```bash
ssh -t pi
cd ~/mobile-robot
bash act/scripts/start-demos.sh
```

Keys work immediately in the terminal, without Enter. Typed letters are hidden.
Pressing `s` prints `RECORDING STARTED` and the folder path; while recording,
the terminal prints the saved sample count once per second.

- `s`: return to the session start pose, release torque, and start a demo; guide
  approach, grip, lift, place and release.
- `f`: save a successful demo. Reset the object, then press `s` again.
- `d`: discard the current demo after a miss or interrupted movement.
- `q`: finish the session. An unfinished demo is discarded.

Repeat without restarting the script. Each demo gets a unique folder under
`act/data/episode-*`; previous sessions are retained. Reset movements between
`f` and the next `s` are not recorded. Use one easy object first and review one
complete demo before collecting a batch. With a local graphical Pi desktop,
run `python3 act/runtime/record.py` instead for a live camera window and the
same keys in that window.

Support the arm throughout hand guidance. Torque is disabled after each return
and on return errors or exit; park it on its support before letting go.

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

Each sample contains all six measured positions, the wrist camera-read interval,
the Logitech overview camera-read interval, and the joint-read interval
on the same monotonic clock. JPEGs are ordinary RGB images; OpenCV supplies BGR
input to its encoder. The two camera reads happen sequentially, so their host
timestamps show the offset; sensor exposure timestamps are unavailable. Inspect
buffering and alignment in the first episode before collecting a batch.
`data/session-start-pose.json` stores the measured session reference pose.
Every episode's `metadata.json` stores that same
pose as `start_state_ticks`, plus the measured state at recording start and the
return completion timestamp. Each sample separately records the live pose.
Before each episode, the recorder returns the arm to the saved pose and releases
torque for hand guidance.
Communication errors invalidate the active episode; previous positions are never
reused. Training/export is the next block of work: these are raw demonstrations,
not yet a LeRobot training dataset or action labels.

Verified on the Pi: all six servos responded, torque was off on all six, and
the generic USB camera captured 640×480 MJPEG at about 19 FPS in a short read
check (device reports 30 FPS). Recording requests 10 samples/s; validate the
actual rate in the first episode. Teleoperation must be stopped so the recorder
is the sole servo-bus owner.

Hardware status: `REQUIRES_HARDWARE_TEST` for supported guidance, manual gripping
and one complete pick-and-place. The user confirmed the old joint ranges belonged to another task; they no longer
restrict recording.
