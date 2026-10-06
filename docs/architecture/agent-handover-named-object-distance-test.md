# Agent handover: minimal named-object distance test

Prepared: 4 October 2026. Status: **LIVE BOX SEGMENTATION VERIFIED; METRIC CLEARANCE UNQUALIFIED; PHYSICAL ACCEPTANCE INCOMPLETE**.

## Task and boundaries

Implement a small stationary test in which the operator enters an object name,
for example `bottle`, `cup` or `shoe`. Verify this chain:

**Requested name → YOLOE RGB segmentation → native ToF object mask → visible
floor-contact distance corrected for camera pose.**

The test must accept a different name without code edits. It must not be hardcoded
to bottles or require a reference photograph and manually selected reference box.
An arbitrary name is an input capability, not a guarantee that every object will
be recognized or produce usable depth.

Keep the base and arm stationary. Announce any head motion to the operator
before issuing it. Preserve camera capture processes and autonomous-motion gates.
Do not start the approach service, publish drive commands, flash firmware or add
SLAM, navigation, pickup or a new autonomous architecture for this task.

The operator explicitly requested this separate handover document. Record actual
implementation and measured results in the existing [architecture README](README.md).

## Current implementation and evidence

- Workstation inference is the selected first implementation; capture stays on
  the Pi. Use `ssh pi` for the established connection.
- `pi/approach_object/tools/capture_rgb_tof.py` captures the existing ROS image
  publishers, paired within 80 ms. The capture supports bursts and records optional declared servo pose
  and factory ToF metadata. Do not discard or silently commit unrelated work.
- `pi/approach_object/tools/yoloe_depth_preview.py` currently uses YOLOE visual
  reference matching, requires `--reference` and `--box`, and writes a
  bottle-specific mask filename. It is not the requested text-prompt interface.
- `pi/approach_object/approach_object/geometry.py` contains depth reconstruction
  and RGB projection, but its current distance estimator measures a nearest
  whole-object surface, not the floor-contact point. Reuse projection primitives;
  do not label that existing output as contact clearance.
- `pi/approach_object/config/calibration.yaml` has missing intrinsics/transforms
  and front-edge offset, unvalidated depth interpretation, and movement disabled.
- Uncommitted `floor_range.py`, `range_quality.py`, their tools and tests are
  diagnostics to inspect, not qualified perception components. Earlier depth
  repeatability checks were poor and a 20-frame burst yielded no accepted floor
  plane. Do not present floor fitting as an already-working fallback.
- Both cameras tilt together, but their fixed optical offsets may differ.
  ToF publisher values are millimetres; radial versus optical-axis range still
  needs validation. FOV overlap alone does not establish metric registration.
- Existing YOLOE-26n-seg checkpoint SHA-256:
  `1741c1f8da3cea47e2c01829c334a50dc0b9bbd05e685b90a3ce84fae32c8c1b`.
  Workstation trials used Ultralytics 8.4.173. Temporary workstation environments,
  models and captures may disappear; verify availability and package compatibility.

## Head-angle convention

Servo commands and physical viewing angles are different quantities. Relative to
the vertical pole, 90° means horizontal, below 90° means downward and above 90°
means upward. Downward optical pitch is `90° - pole_viewing_angle`.

| Servo command | Estimated pole viewing angle | Basis |
| --- | --- | --- |
| 80° | 54° | RGB scene rotation fit |
| 90° | 61° | RGB scene rotation fit |
| 100° | 71° | RGB scene rotation fit |
| 110° | 80° | RGB scene rotation fit |
| 120° | 90° | Approximate operator startup reference |
| 130° | 100° | Interpolation |
| 140° | 111° | Interpolation |
| 150° | 121° | Successful upward-view fit |

Use piecewise interpolation only within 80–150°. These are estimates, not measured
shaft feedback or qualified ToF optical poses. The first sweep showed no upper
movement; a subsequent 80° → 150° → 80° sweep did move upward. Retain that
inconsistency as unresolved. A requested command alone cannot confirm pose.

`tools/estimate_head_mapping.py` reproduces the image fit. Pi evidence is under
`~/homer-evidence/approach-object/{head-angle-mapping,head-full-range,head-map-complete}`.
The head last returned to command 80°. ESP32 startup commands 120°; restarting the
Pi alone does not establish the ESP32 head pose.

## Implementation contract

Add a workstation-only tool under `pi/approach_object/tools`, named
`named_object_distance_test.py`. Its interface is:

```bash
python pi/approach_object/tools/named_object_distance_test.py \
  --target "shoe" --model /path/to/yoloe-26n-seg.pt \
  --capture /path/to/capture --calibration /path/to/calibration.yaml \
  --servo-deg 100 --output /path/to/test-output
```

This interface is implemented. The tool reads saved
captures and never publishes ROS commands. Accept a nonempty text target; use
YOLOE text embeddings/class setup for that name, not visual-reference prompting.
Reuse the verified checkpoint; start with 640-pixel input and confidence 0.5.
Pin and record dependencies, checksum and prompt in the result. Report processing
latency; this stationary test does not qualify real-time autonomous operation.

1. Read the paired RGB, depth, amplitude/confidence when available, and timestamps.
   Reject RGB/ToF separation above 80 ms. Validate frame IDs, shapes and units.
2. Segment candidates matching the entered name. Select the candidate closest to
   image centre and highlight it. Across the three observations retain identity
   using the existing association logic; reject ambiguous association. A missing
   target invalidates any earlier distance.
3. Reconstruct valid ToF pixels with the appropriate intrinsics and depth
   semantics. Project them into RGB through the camera relationship and select
   points inside the chosen RGB mask. Preserve native ToF pixel indices to build
   and display the **native ToF object mask**. Do not independently recognize or
   match the object in the ToF amplitude image.
4. Select a contact band from the lowest 10% of the visible object mask, excluding
   its boundary, invalid depth and mixed background/floor returns. Start with the
   existing minimum of 12 retained samples; insufficient support returns no
   distance. A lower mask band is only a candidate contact region: hidden,
   clipped or geometrically ambiguous floor contact must be rejected.
5. Transform each accepted sample using camera pose at image acquisition,
   including its pixel-ray angle, fixed ToF optical offset and pivot translation.
   Take the median forward coordinate of reliable contact samples, then subtract
   the robot-front offset. Do not multiply a whole-object median by camera-centre
   pitch or infer a universal camera height from the earlier bottle observation.
6. If only nominal FOV alignment or estimated pose is available, label all mask
   overlays and numerical diagnostics **ESTIMATED / UNQUALIFIED**. Missing units,
   depth semantics or necessary transforms/front offset must yield null contact
   clearance with the specific blocker. Never invent missing calibration.
7. Save generic `object-mask.png`, `tof-object-mask.png`, contact-sample mask,
   annotated RGB, ToF amplitude/outline and masked-depth panels, plus `result.json`.
   Report target, score, selected identity, timestamps, servo command, estimated
   viewing angle, retained sample count, depth spread, clearance or null, quality
   reason, calibration provenance and `motion_authority: false`. Invalid regions
   must be visibly distinct from valid zero-valued measurements.

Do not generalize the autonomous start service as part of this minimal test.
Keep `enable_motion: false`, `qualified: false` and qualification flags unchanged.

## Implementation status — 5 October 2026

Implemented `named_object_distance_test.py` and the persistent setup/live-capture
runner `run_named_object_distance_test.py`. Text reaches `set_classes([target])`
unchanged. Model, MobileCLIP encoder, dependency versions and CLIP revision are
pinned and verified. Default input remains 640; `--imgsz 960` succeeded for the
operator's literal `box` prompt. The runner sources the Pi's established Cyclone
DDS environment, resolving the initial Fast DDS capture segfault.

At command 80°, ten fresh paired frames detected the upright red box in 10/10
frames (scores 0.648–0.698; timestamp separation 3.6–74.02 ms). Three-frame tests
at 100° and 90° detected it but clipped its lower contact region. Announced
head-only movements ended at 80° after seven-second settling waits. Base and arm
were not commanded. Literal `red box` was also tested successfully, but this is
not the required second distinct physical object.

The existing ToF device owner now writes factory calibration metadata; controls
are in hundredths of pixels. Captured K has fx=190.92, fy=191.25, cx=120, cy=90
at 240×180. A subsequent fresh three-frame test detected `box` in all frames
(scores 0.753–0.817); nominal contact-band depth spread 0.660–0.667 m was not
usable range evidence. Factory K does not qualify physical accuracy or RGB
registration. No confidence threshold or depth semantics was invented.

The operator reference is 0.38 m horizontal front-edge clearance and box height
0.35 m. The operator subsequently identified the v6 URDF as the geometry source.
Its camera origin at the estimated 80° pose is 0.583355 m high and 0.138369 m
behind the plate front edge; lens-centre/optical-axis assumptions and physical
angle still require validation. Pythagoras applies to the specific ray's
vertical separation, not universally to the object's full height. The numeric
output remains null with explicit calibration blockers. All flags remain false;
absence, second-object and two-view metric acceptance remain outstanding.
See the architecture README for the runnable command and local evidence paths.

## Minimal physical test

1. Enter an object name and place that object on a level floor, with the contact
   region visible. Record an independent reference clearance from the robot's
   front edge to its nearest floor-contact point.
2. Announce head movement. Capture three distinct fresh pairs at command 100°,
   then three at 80°. Use a conservative 7 s settling wait for this test and
   verify the camera view has stopped changing; never use images during tilt.
   Acquisition freshness must be within 500 ms when captured. Saved-image
   inference may occur later; do not mislabel it as live perception.
3. Run the same named-object tool on every pair. Visually inspect RGB segmentation,
   native ToF association and contact samples before accepting distance results.
4. Remove the target and recapture without further head movement. Confirm that
   target absence clears the previous valid distance.
5. Repeat with a different object and name, with no code changes. Include a simple
   opaque object; transparent or weak-return objects may correctly be rejected.

## Checks and acceptance

- Verify that two different target names reach text-prompt inference unchanged.
- Test native ToF indexing/projection, millimetre conversion, angle sign,
  per-pixel transformation and front-edge subtraction with meaningful synthetic
  geometry. Reject unknown calibration rather than substituting identity matrices.
- Test absent target, ambiguous identity, clipped contact, invalid/contaminated
  depth and missing geometry. Confirm all produce null clearance and a reason.
- Inspect the tool for absence of motion publication or camera-device access.
- Physical acceptance: correct masks for both objects; three fresh observations
  per view; paired timestamp separation at most 80 ms; valid clearance estimates
  within 5 cm of the independent reference and within 5 cm between the two views;
  removing the object produces no valid distance; no base or arm movement.
- If accurate clearance cannot yet be established, deliver the useful masks and
  explicit blocker, but mark the distance criterion failed or untested. Do not
  claim that attractive overlays establish accuracy.

Run proportionate checks, record actual outcomes and commands in the architecture
README, and push only checked task changes on the existing feature branch for
review. Do not merge while physical criteria remain outstanding. Avoid expanding
this test into drive qualification or redesigning the robot stack.
