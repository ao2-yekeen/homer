# ACT placement reliability plan

Saved 2026-10-11 following the supervised run in
`act/outputs/observed-demo-20261011-111819`.

The run completed 392 commands but did not place the object inside the plastic
container. Camera snapshots showed the object beside it near the end. Completion
of motor commands is not task success. The arm was left holding with torque on.

1. **Test camera influence offline (completed 2026-10-11).** Hold the six joint inputs
   fixed and replace wrist, overview, then both images. Repeat identical inputs
   to measure numerical variation. Compare decoded predictions in servo ticks,
   including the first five actions and the full 50-action chunk. Use real images
   from validation demonstrations with nearby joint states, plus blank-image
   ablations. Save inputs, donor identities, predictions and metrics. Sensitivity
   establishes image influence, not correct object localisation or successful
   control; mismatched image/state pairs and blanks are diagnostic inputs only.
   [Results and reproduction command](camera-influence-results.md): both cameras
   affect predictions, identical repeats have zero change, and original live-run
   predictions are reproduced exactly. Correct spatial feedback remains unproven.
2. **Collect recovery demonstrations.** At repeatable policy failure points,
   take over manually and record successful placement and release. Keep human
   corrections identifiable; do not label failed autonomous motion as expert
   actions. Add corrections to training and retain whole-episode validation.
3. **Test more frequent visual updates.** Compare the current 50 executed actions
   per prediction (five seconds at 10 Hz) with shorter execution intervals.
   Consider ACT temporal ensembling for smooth overlapping predictions. Measure
   inference, camera and network delays before selecting the interval. This is
   an experiment, not a guaranteed fix; avoid introducing discontinuous motion.
4. **Measure task success.** Run ten supervised trials with a consistent setup,
   then nearby object-position variations. Score pickup, placement inside the
   container and release separately. Select changes using physical outcomes,
   rather than validation joint error alone.

If the container and grasp geometry are repeatable, a separately validated
placement-and-release trajectory after ACT pickup is an alternative. It depends
on a consistent grasp and fixed container pose; it is not yet implemented.

Only step 1 is currently authorised for implementation. Further physical changes
require testing; no new policy motion is part of the offline camera test.

References: [ACT](https://arxiv.org/abs/2304.13705),
[DART discussion of DAgger and corrective feedback](https://proceedings.mlr.press/v78/laskey17a/laskey17a.pdf).
