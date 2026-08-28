# Characterising the reachable workspace

This is the HOM-8 procedure. It records what the *physical* arm can reach
around the raised test platform; it does not derive a workspace from the URDF
or servo EEPROM limits.

## Reference frame

Use `base_footprint`: its origin is the floor projection of the robot base
reference point; +x is forward, +y is robot left, and +z is upward. Measure in
metres. Mark the origin and +x direction on the floor before recording points.
The URDF's 0.400 m arm-mount height is only an approximate setup cross-check;
measure the physical robot and enter platform values in
[`config/reachable_workspace.yaml`](../../config/reachable_workspace.yaml).

## Safe measurement procedure

`REQUIRES_HARDWARE_TEST`. Keep the base position taped, use the raised rigid
platform, support the arm, set conservative speed, and keep an emergency power
disconnect accessible. Do not attempt floor points or maximum extension.

1. With motion inhibited, confirm the frame marks, platform height, and usable
   platform x/y bounds. Enter those measured values in the config.
2. Enable only the existing manual, dead-man-controlled arm jog path. Move one
   joint at a time and stop immediately if there is any mast, base, camera,
   platform, self-collision, or cable-clearance concern.
3. At representative front/left/right and near/far platform points, measure
   the gripper-centre XYZ. Record both safe reachable points and boundary or
   unreachable points. An unreachable sample must say why.
4. Capture the displayed `/soarm/state_ticks` values when available. It is
   evidence only, not a command to replay.
5. Return to a known safe position between samples. Do not automate movement
   from the resulting records.

The recorder only appends an observation; it sends no ROS message and opens no
servo device:

```bash
cd ~/mobile-robot
python3 pi/soarm_workspace_record.py \
  --sample-id front-centre-01 --x-m 0.20 --y-m 0.00 --z-m 0.32 \
  --result reachable --reason "Held safely with all clearances checked" \
  --joint-ticks 1000 1500 1800 2700 2048 3000 \
  --mast-clear --base-clear --camera-clear --platform-clear --cables-clear \
  --confirm-physical-observation
```

The example coordinates and ticks are format examples, not measurements.
Samples are appended to `data/reachability/workspace_samples.jsonl`, which is
intentionally not committed because it is robot-specific experimental data.

## Completion evidence

HOM-8 is complete only when the config has a physically aligned frame and
measured platform height/zone, and the sample log contains representative safe
reachable and unreachable XYZ observations with evidence. Review the samples
before using any region for grasp planning.
