# Step 2: get hand-guided recording working

Use the generic USB wrist camera plus the fixed Logitech C930e overview camera.
Save raw joint ticks and both RGB views; train a fresh
ACT policy later using official LeRobot code and its pretrained visual backbone.
Hardware checks are `REQUIRES_HARDWARE_TEST`.

## 1. Check the USB wrist camera

Find the wrist camera's actual capture device and use a stable USB device path
when available. The recorder requires the Logitech C930e as the second,
overview view. Start with 640×480 and a recording rate of 10 FPS if supported.
Preview both views through the intended pick-and-place path: the wrist camera
should show the object and gripper, while the Logitech should show the whole
work area. Check that the wrist cable moves freely and your guiding hand does
not hide the task.

Done when: a clear, usable wrist view is available throughout the motion.

## 2. Test supported hand guidance and the gripper

Stop the existing arm bridge and other arm controllers before opening the
servo port in the recorder. Keep the recorder as the sole bus owner; no jogging,
policy or watchdog position writes may occur during guidance.

With the vertical arm physically supported, verify the installed SDK's torque
controls. Release the joints to be guided and confirm all six positions still
read correctly while you move them gently. Do not force powered joints.
Try opening and closing the gripper manually and record which tick values
mean open and closed.

If manual gripper guidance is impractical, add simple open/close keyboard
controls for the gripper alone, using tested endpoints and slow motion. Keep
arm-joint writes blocked and hands clear of the powered gripper.

Before collecting demonstrations, test the return from the current pose to the
saved session pose in a clear work area. Confirm that current-position goals are
staged before torque is enabled, the return uses small steps, torque is verified
off before guidance, and `q` or `x` aborts the return. On failure, keep the arm
supported and verify torque is off before resuming.

Done when: you can guide the arm, demonstrate gripper opening/closing, read
all joints and exit guidance without a stale-target movement.

## 3. Build one recorder

Implement `act/runtime/record.py`, with a small
`act/config/rig.example.yaml` for camera device, FPS, joint order and gripper
settings. Reuse SDK position reads from the arm bridge. Accept valid raw ticks (0..4095);
old task-specific motion limits do not constrain hand-guided recording.
No new ROS topics or separate camera process is needed.

Provide start, stop and discard keys, plus explicit teaching entry/exit controls
or instructions for the tested manual procedure. Start at 10 FPS. Each episode
saves:

- Wrist and Logitech overview RGB images with their capture timestamps.
- Six measured raw tick positions and read-start/read-end timestamps.
- Joint order, camera settings, actual sample rate and success/discard label.
- Gripper command events if keyboard gripper control is used.

Timestamp image capture and position reads on the same Pi using the same
monotonic clock. Store their actual timing rather than assuming the loop is
exactly 10 FPS. A host camera-read timestamp estimates capture time; inspect
buffering delay in the sample. Convert camera BGR to RGB once. A failed capture
or incomplete joint read stops and invalidates the episode; never reuse the
previous sample. Keep the camera queue short to avoid old frames.

Done when: one recording contains fresh images and all six measured joints
at roughly 10 FPS without unexplained gaps or growing capture delay. Lower
FPS if the hardware cannot sustain it; change it before the full dataset.

## 4. Inspect one complete pick-and-place

Manually demonstrate approach, grip, lift, place and release on the established
raised platform with the base stationary. Review the video alongside a simple
joint-position plot. Check that joint motion matches the images, gripper
transitions are captured, the object is visible and the guiding hand does not
dominate the view. Fix obvious delay or missing data before collecting more.

Done when: one successful episode is complete and aligned well enough to see
which joint/gripper movement accompanies each part of the task. Then proceed
to the five pilot demonstrations in step 3 of the general plan.

## Keep implementation small

Use raw ticks in recording and training; dataset mean/std normalization comes
later. No radians conversion, generalized state machine, custom camera publisher,
separate timing tool or 10/20/30 Hz study is needed for the first demo. Test bus
ownership, failed-read handling and the verified torque procedure. Recording
is not trajectory replay: the next step constructs future-position action
labels, and ACT learns predictions from images plus current joint positions.

Recorder and continuous collection commands are implemented. See
[Collect demonstrations](collect-demos.md) for the short run guide, handover
layout and remaining physical checks.
