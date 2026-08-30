# Characterising the reachable workspace

This is the physical-validation procedure for the current HOM-8 reachable-
workspace implementation. The primary workspace is generated from the URDF
geometry and joint limits, then limited to the front of the robot and filtered
against the modelled mast/base/arm-mount/neck collision volumes. Physical
observations are retained to validate a boundary, the servo-to-URDF alignment,
or an unmodelled clearance risk.

Generate and view the model-derived point cloud:

```bash
python3 simulation/pybullet_sim.py --workspace --gui --seconds 60
```

The display shows the workspace envelope around the robot: blue is lowest and
red is highest. The underlying gripper-centre point cloud is exported as
`data/reachability/urdf_workspace.ply` for a full-resolution view outside
PyBullet.

Use the right-hand PyBullet panel to navigate. **Camera yaw (turn around)**
rotates around the robot; pitch, zoom, and look-at x/y/z adjust the angle,
distance, and target. Standard PyBullet mouse controls remain available.

## Reference frame

Use `base_footprint`: its origin is the floor projection of the robot base
reference point; +x is forward, +y is robot left, and +z is upward. Measure in
metres. Mark the origin and +x direction on the floor before recording points.
Use the URDF frame and joint limits as the preliminary workspace source. The
current EEPROM tick limits are not yet aligned with URDF radians, so this is
not a replacement for a physical safety boundary. Enter
the physically measured platform position and usable target zone in
[`config/reachable_workspace.yaml`](../../config/reachable_workspace.yaml).

## Safe measurement procedure

`REQUIRES_HARDWARE_TEST`. Keep the base position taped, use the raised rigid
platform, support the arm, set conservative speed, and keep an emergency power
disconnect accessible. Do not attempt floor points or maximum extension.

1. Generate the URDF workspace cloud and inspect its overlap with the intended
   raised-platform zone. Do not use it as an automatic motion command.
2. With motion inhibited, confirm the frame marks, platform height, and usable
   platform x/y bounds. Enter those measured values in the config.
3. Enable only the existing manual, dead-man-controlled arm jog path. Move one
   joint at a time and stop immediately if there is any mast, base, camera,
   platform, self-collision, or cable-clearance concern.
4. Test only selected cloud-boundary points or points near the platform. Record
   disagreement or any unmodelled clearance restriction; an unreachable sample
   must say why.
5. Capture the displayed `/soarm/state_ticks` values when available. It is
   evidence only, not a command to replay.
6. Return to a known safe position between samples. Do not automate movement
   from the resulting records.

The recorder only appends an observation; it sends no ROS message and opens no
servo device:

```bash
cd /path/to/esp32_robot_bt
python3 pi/soarm_workspace_record.py \
  --sample-id front-centre-01 --x-m 0.20 --y-m 0.00 --z-m 0.32 \
  --result reachable --reason "Held safely with all clearances checked" \
  --joint-ticks 1000 1500 1800 2700 2048 3000 \
  --mast-clear --base-clear --camera-clear --platform-clear --cables-clear \
  --confirm-physical-observation
```

The example coordinates and ticks are format examples, not measurements. The
command only records an observation; it does not move the robot.
Samples are appended to `data/reachability/workspace_samples.jsonl`, which is
intentionally not committed because it is robot-specific experimental data.

## HOM-9 reachability lookup

Build the reusable lookup once the URDF workspace cloud has been generated:

```bash
python3 pi/soarm_reachability_map.py build \
  --points data/reachability/urdf_workspace.ply \
  --output data/reachability/reachability_map.json
```

It stores `reachable` cells directly represented by collision-filtered samples,
and `marginal` cells within a 3 cm boundary margin. Every other point is
`unreachable`. Query an XYZ gripper-centre position in `base_footprint`:

```bash
python3 pi/soarm_reachability_map.py query \
  --x-m 0.20 --y-m 0.00 --z-m 0.32
```

The lookup only filters candidate grasps. It does not approve a trajectory or
command the robot; `marginal` is always `REQUIRES_HARDWARE_TEST`.

After recording physical samples using the procedure above, compare them
without moving hardware:

```bash
python3 pi/soarm_reachability_map.py validate \
  --samples data/reachability/workspace_samples.jsonl
```

For HOM-9 acceptance, record several measurements whose non-marginal
predictions match the observed result. Do not record a match until it has been
observed on the physical robot.

## Completion evidence

HOM-8 is complete when the calibrated-URDF point cloud has been generated and
reviewed against the physically aligned platform height/zone. Record any
boundary validation observations and discrepancies before using a region for
grasp planning.
