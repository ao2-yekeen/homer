# Literature review: segmenting a visually selected object in RGB and ToF

Reviewed 6 October 2026. This is a targeted engineering review, not an exhaustive
systematic review. The evidence base is restricted to peer-reviewed journal and
conference/workshop papers. Author-hosted manuscripts are linked where useful
for access, with the publication venue identified. Paper results are not
evidence of accuracy on this robot.

## Conclusion

The most defensible next implementation is calibrated RGB-guided segmentation
on the native ToF grid, with explicit uncertainty and separate depth validity
labels. Use the visual prompt to establish object identity; use depth geometry,
boundaries and temporal evidence to assess each measurement. An object can have
several faces, disconnected visible regions and substantial depth variation.

This recommendation is an engineering synthesis of the literature, not a
published algorithm already validated on our sensor. Neither preserving every
projected return nor keeping only the largest depth component establishes a
correct object segmentation.

## What the existing implementation does

Code inspection found several independent opportunities for losing measurements:

1. `named_object_distance_test.py::analyse_mask` removes returns below the
   configured confidence threshold before registration.
2. `depth_registration.py::rasterize` constructs native depth triangles and
   rejects triangles with a range span over the configured limit (default 5 cm).
   A valid sample with no surviving visible triangle can lose RGB association.
3. Association uses a default 4-pixel RGB mask erosion and requires at least
   60% of a sample's rendered visible footprint inside that eroded mask.
4. `surface_distance.py::select_surface` requires local neighbours, connects
   samples using a default 5 cm range difference, anchors selection to the
   largest group, and pools only sufficiently large groups whose P5–P95 range
   intervals overlap the anchor's interval.
5. MAD trimming and a default 15 cm total range-spread limit further restrict
   distance support. Separated groups can trigger an ambiguity result.

These are heuristics. Range separation alone cannot distinguish another face
of the target from background contamination. The current function receives no
RGB appearance, object probability or local surface-normal evidence.

The recent visualization change preserves all *associated* returns rather than
only distance-support returns. It does not recover returns lost during steps
1–3 or prove the retained returns are physically correct. The relative
contribution of each stage still needs a measured rejection breakdown.

## Relevant literature

### 1. Joint RGB–depth segmentation

Liu, Philipose and Sun, **Automatic objects segmentation with RGB-D cameras**
(Journal of Visual Communication and Image Representation, 2014), combine RGB,
depth and boundary evidence using a boundary-constrained
trilateral filter and graph cuts. Their reported evaluation covers 24 objects
and 1,500 images. They specifically discuss depth holes and noisy boundaries.
Simple colour-guided smoothing can fail when opposite sides of a boundary
have similar colours. [Publisher record](https://www.sciencedirect.com/science/article/abs/pii/S1047320313000527)

**Application:** object membership should combine modalities rather than be
determined by one depth cluster. Internal printed texture should not become a
mandatory object boundary. The paper's camera and dataset differ from ours;
its abstract and available introduction were reviewed, not an independently
reproduced implementation.

### 2. Seeded graph segmentation

Rother, Kolmogorov and Blake, **GrabCut** (ACM Transactions on Graphics /
SIGGRAPH, 2004), optimize foreground/background
appearance and boundary terms with iterative graph cuts.
[Author institution](https://www.microsoft.com/en-us/research/publication/grabcut-interactive-foreground-extraction-using-iterated-graph-cuts/)

Grady, **Random Walks for Image Segmentation** (IEEE TPAMI, 2006), propagates labels from
seeds over a weighted graph and computes a membership probability for each
unlabelled node. [Author-hosted paper](https://leogrady.net/wp-content/uploads/2017/01/grady2006random.pdf)

**Application:** a prompted RGB mask can supply foreground/background evidence
for a native-depth graph. Edges can incorporate geometry and boundary evidence.
Graph cuts are a candidate for discrete labels; random walker is an alternative
when soft membership is useful. Its probabilities are graph-model results,
not calibrated probabilities of physical correctness. OpenCV's RGB GrabCut
alone does not implement the proposed RGB–ToF fusion.

### 3. ToF uncertainty is sensor dependent

Belhedi et al., **Noise modelling in time-of-flight sensors…** (IET Computer
Vision, 2015), show on
their sensor that amplitude alone does not adequately describe depth noise;
pixel position, depth and integration time also matter. Their study focuses
on repeatability, which does not by itself establish absence of systematic bias.
[Paper](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/iet-cvi.2014.0135)

**Application:** characterize our sensor before assigning statistical meaning
to a confidence threshold or a depth difference. Use a measured noise model
to normalize local surface residuals. Do not import another camera's constants
or assume repeated readings prove absolute accuracy.

### 4. Mixed boundary returns can be confidently wrong

Vasudevan et al., **Color-Guided Flying Pixel Correction in Depth Images**
(MMSP 2024), describe mixed foreground/background ToF returns that yield
confident but incorrect depths. Their correction uses aligned RGB information
and a 3D neighbourhood reflecting the measurement field of view, with corrected
positions constrained to the measurement ray.
[Paper](https://arxiv.org/html/2410.08084v1)
[Published proceedings DOI](https://doi.org/10.1109/MMSP61759.2024.10743790).

**Application:** model suspect boundary returns explicitly; SDK confidence
alone is insufficient. Correction is a separate, inferred product and should
not overwrite the raw measurement used for reporting range. Their method
assumes aligned inputs and uses appearance assumptions that require checking
on textured or same-colour scenes. Flying pixels are a plausible failure
mechanism here, not a demonstrated explanation of the current capture.

### 5. General visual prompting and tracking

Wang et al., **YOLOE: Real-Time Seeing Anything** (ICCV 2025), support visual,
text and prompt-free detection/segmentation. Visual prompts encode the category
of interest; they do not inherently guarantee identification of one particular
instance among similar objects.
[Published paper](https://openaccess.thecvf.com/content/ICCV2025/papers/Wang_YOLOE_Real-Time_Seeing_Anything_ICCV_2025_paper.pdf)

Ravi et al., **SAM 2: Segment Anything in Images and Videos** (ICLR 2025), provide
promptable image/video segmentation using streaming memory.
[Accepted conference paper](https://openreview.net/notes/edits/attachment?id=DwBxgXiqXM&name=pdf)

**Application:** keep visual selection, and compare RGB masks against a SAM 2
reference on the workstation if needed. These models supply appearance masks,
not calibrated metric depth. Instance selection and tracking still need
evaluation when similar objects coexist or the target is partly visible.
Published speed claims are not Raspberry Pi CPU benchmarks for this setup.

### 6. Physical limits of ToF boundary measurements

Chugunov et al., **Mask-ToF: Learning Microlens Masks for Flying Pixel Correction
in Time-of-Flight Imaging** (CVPR 2021), model ToF image formation and use learned
microlens amplitude masks to mitigate mixed returns at acquisition.
[Published paper](https://openaccess.thecvf.com/content/CVPR2021/papers/Chugunov_Mask-ToF_Learning_Microlens_Masks_for_Flying_Pixel_Correction_in_Time-of-Flight_CVPR_2021_paper.pdf)

**Application:** supports treating mixed boundary measurements as a physical
sensor issue. Its hardware modification is not a software fix for our camera;
the useful lesson is that segmentation alone cannot guarantee correct depth.

SAD / Segment Any RGBD is excluded from the evidence base because the reviewed
source identifies it as a technical report, without a verified peer-reviewed
publication for that work.

## Proposed architecture

The following is our proposed design, rather than a claim made by one paper.

1. **Audit registration and data loss.** Record per-native-sample reasons:
   invalid return, SDK reliability exclusion, outside RGB view, occluded,
   missing mesh coverage, uncertain RGB boundary association, background,
   suspect depth, or accepted target. Retain the raw arrays and show these
   states separately. Validate projection with independent captures; a
   translation refinement cannot repair incorrect intrinsics or extrinsics.

2. **Associate measurements without depending exclusively on triangles.**
   Evaluate direct point projection with bounded sample footprints and depth
   visibility checks as a baseline beside the current mesh. A triangle failing
   continuity should not automatically make all its vertices unavailable.
   Boundary footprints can contain mixed returns, so direct projection is not
   an automatic guarantee of correct membership either.

3. **Represent association uncertainty.** Sample the RGB object mask or logits
   over each projected footprint and plausible projection perturbations.
   Preserve uncertain boundary samples as uncertain, rather than turning a
   universal erosion margin into definitive background. Account for different
   camera viewpoints, visibility and capture timing.

4. **Refine membership with a seeded RGB–depth graph.** Use native ToF samples
   as nodes. Foreground/background costs come from RGB membership and reliable
   seeds; neighbour costs use local geometry residuals normalized by measured
   noise, reliable depth discontinuities and RGB object-boundary evidence.
   Allow folds, slopes and disconnected visible patches. Avoid enforcing one
   global depth interval or one object colour. Missing depth remains unknown.
   Compare graph cuts and random walker before selecting the solver.

5. **Keep object identity separate from measurement reliability.** A sample
   can be likely to belong to the target while its distance is unreliable.
   Store both assessments. Preserve the full visible object mask and label
   unknown depth explicitly. Never present completed or corrected depth as a
   raw sensor observation.

6. **Estimate range from reliable target measurements.** Preserve the current
   median Euclidean range definition for comparability, and report its support
   and distribution. A multi-face object need not have one narrow depth peak.
   Evaluate a separate nearest-supported-surface statistic if approach control
   needs it; raw minimum is vulnerable to a single bad return. Membership
   uncertainty and genuine physical extent need different result explanations.

7. **Use temporal evidence with correct correspondence.** Reproject or track
   observations before combining them. Motion, occlusion changes and viewpoint
   changes must not cause stale depths to be assigned to the current mask.
   Temporal agreement supports repeatability, not proof of metric correctness.

For Pi deployment, start by measuring the cost of native-resolution graph
refinement with the existing visual detector. Use workstation models as
comparison references. This is a deployment hypothesis; no Pi latency or
memory budget has been demonstrated for the proposed system.

## Evaluation needed to establish generalisation

Use the requested ten distinct objects, with multiple distances, orientations,
backgrounds and complete/partially clipped/occluded views. Include texture,
thin features, multiple visible faces and similar distractors. Report:

- RGB and native-depth object membership precision/recall and boundary error;
- coverage relative to actually valid target measurements, separately from
  missing sensor returns and portions outside either camera's view;
- distance error against a measured visible surface with a consistent origin;
- failure/abstention rate, temporal stability, Pi latency and peak memory.

Fix parameters on a development subset and evaluate held-out objects. Compare
the current pipeline, projection plus RGB mask, graph refinement, and temporal
refinement separately. Measure calibration residuals and raw ToF bias apart
from segmentation error. Do not tune to the known 52 cm box distance.

No implementation or benchmark was run as part of this literature review.

## Subsequent implementation

The saved-frame visual-prompt tool now uses bounded point footprints and a
soft-seeded random-walker native graph, separate membership/reliability maps,
and a per-sample audit. See `SURFACE_DISTANCE.md` for implemented rules and
limitations. Graph-cut comparison, correspondence-based temporal depth fusion,
independent calibration validation and the ten-object benchmark remain pending.
The code change is not evidence that segmentation or range accuracy improved.
