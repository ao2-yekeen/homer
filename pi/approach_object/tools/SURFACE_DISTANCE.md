# Visible object distance

The saved-frame visual-prompt tool `named_object_distance_test.py` uses the
RGB-guided native ToF graph implemented in `approach_object/surface_distance.py`.
Run it through `run_named_object_distance_test.py` with a visual `--reference`
and `--box`, plus `--capture DIRECTORY`. No floor, robot pose, complete object
silhouette, object class, known dimensions or expected range is required.
This path is not yet integrated into the ROS approach controller.

## Membership and visibility

`depth_registration.point_footprints` projects each measured native pixel's
four corners at its measured depth, with distortion and calibrated extrinsics.
A per-RGB-pixel z buffer determines visible footprint ownership. It does not
require neighboring triangles to pass a depth-span threshold. Footprints over
32 RGB pixels in either dimension are excluded and recorded as uncovered;
footprints approximate sampling cells, not the sensor's optical response.
No missing depth is interpolated. Raw unreliable returns may occlude other
returns: the system does not invent visibility behind an unreliable sample.

RGB mask overlap supplies a soft prior for a four-connected native graph.
A regularized random-walker Laplacian combines this prior with local range
and mask boundaries. Separate faces and disconnected visible patches may
contribute; there is no largest-component selection or global range gate.
Solver failure is reported and uses the RGB prior, not a claimed graph result.

`association_perturbation_px` defaults to 1 and provides a configurable
translation sensitivity envelope, not measured calibration uncertainty.
Mask erosion supplies strong evidence only; it does not delete object
membership. Image borders use replicated mask values for partial objects.
Membership >= 0.5 is visualized as target. Strong distance evidence requires
both graph membership and the lower overlap envelope >= 0.75; background
requires graph membership and the upper envelope <= 0.25. The other samples
remain uncertain. These engineering thresholds are not calibrated probabilities.

## Independent range reliability

SDK confidence below `confidence_min` excludes a measurement from distance,
not from object membership. Local axial second differences flag suspect raw
measurements while allowing slopes. The least available directional residual
avoids automatically rejecting a fold with a coherent adjoining face. Local
support requires at least three returns in a 3x3 neighborhood, including self.
This can abstain on very thin targets; it does not remove them from membership.

`tof_noise_sigma_m`, when available from sensor characterization, supplies the
geometry scale. Without it, observed neighbor differences and robust local
residual statistics provide explicitly uncalibrated scales. These heuristics
cannot identify every flying pixel or remove systematic sensor bias.

Distance is median raw Euclidean range from the **ToF optical origin** across
reliable target measurements, each native reading counted once. No dominant
interval, total extent cap or global MAD range trim is applied. P5–P95 spread
describes the measured surface distribution, not an accuracy bound. Too few
reliable samples produce no distance. Ambiguous visual instance selection
still produces no distance; old depths are not reused.

`depth_statistic: median` explicitly identifies the reported per-frame depth.
The burst result is the median of frame medians, identified by
`aggregation_statistic: median_of_frame_medians`. Raw minima are diagnostic
extremes and are never substituted for the reported object distance.

Burst aggregation requires every frame to return a range and overlapping
P5–P95 intervals. This is a repeatability heuristic, not correspondence-based
3D fusion or proof of absolute accuracy. Existing visual tracking selects the
instance; the graph only refines the selected mask's depth association.

## Artifacts and audit

- `distance.png`: RGB outline and distance support points.
- `tof-depth.png`: complete depth scene with target outline.
- `isolated-depth.png`: all target-associated raw depths, including those
  excluded from distance. Black means absent association or invalid depth.
- `membership.png`: soft graph membership; unknown samples are black.
- `sample-states.png`: labelled native audit view.
- `data/`: raw isolated depths, RGB/native/support masks, membership/prior
  arrays, SDK/local reliability, local residuals, sample states and legend.

The audit separates invalid depth, outside RGB view, not visible/uncovered,
background, uncertain membership, target with unreliable depth, and distance
support. Occlusion and uncovered subpixel footprints are currently a combined
state. `result.json` records counts, solver status, scale sources and range.
Unknown pixels are never filled or shown as measured depth.

Existing background RGB/IR translation refinement remains optional. Failed
cross-modal fits provide no displacement evidence; they neither invalidate nor
validate the fitted geometry. Qualification flags remain false and motion has
no authority. Calibration and sensor bias must be validated independently.

## Remaining evaluation

The literature review is `RGB_TOF_SEGMENTATION_REVIEW.md`. This implementation
is an engineering synthesis, not a reproduced published method. Compare it
with projection-only and graph-cut baselines on ten held-out objects, partial
views, distractors and multiple distances. Measure membership precision/recall,
valid-depth coverage, ToF-origin range error, failure rate, Pi latency and memory.

## Experiment storage

Capture tools save timestamped directories under the repository's `captures/`
folder. These files and local `experiment-archives/` bundles are ignored by Git.
Experiment data is archived in the Drive Experiments folder.
No accuracy, generalisation or Pi performance claim follows from implementation.
