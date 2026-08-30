# Characterising the reachable workspace

This is the HOM-8 procedure. The primary workspace is generated from the URDF
geometry and joint limits, then limited to the front of the robot and filtered
against the modelled mast/base/arm-mount/neck collision volumes. Physical
observations are retained to validate a boundary, the servo-to-URDF alignment,
or an unmodelled clearance risk.

Generate and view the model-derived point cloud:

```bash
python3 simulation/pybullet_sim.py --workspace --gui --seconds 60
```

The green cloud contains gripper-centre positions in `base_footprint`. It is
also exported as `data/reachability/urdf_workspace.ply` for a full-resolution
3D view outside PyBullet.

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

HOM-8 is complete when the calibrated-URDF point cloud has been generated and
reviewed against the physically aligned platform height/zone. Record any
boundary validation observations and discrepancies before using a region for
grasp planning.
