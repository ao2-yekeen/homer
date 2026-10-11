# ACT failure investigation — 11 October 2026

This is an inspection of the learned ACT policy, not the archived classical
controller. No motor commands, training, dataset deletion, or controller changes
were performed for this investigation.

## Conclusion

The latest attempt with five actions per observation stalled because it repeatedly
executed predictions near the starting pose. That experiment has already been
reverted to the previous default of 50 actions. The preceding full-chunk attempts
have a different problem: they move towards the carton, but the saved observations
do not demonstrate a sustained pickup and transfer. Motor feedback follows the
commands sufficiently closely that a missing command stream does not explain the
latest full-chunk failure.

There is direct evidence of ambiguous pause targets in the training data and
substantial prediction error even on successful demonstration observations.
There is also a fixed execution cutoff unrelated to task completion. These are
concrete problems; their individual contributions to physical grasp failure have
not been isolated experimentally. The evidence does not establish one mechanical
cause, prove that images are ignored, or establish that a different algorithm
would succeed.

## Evidence examined

- Local movement reports for 11 October and matching Pi directory listing.
  The Pi has no newer `demo-*` run than `demo-20261011-155137`.
- Complete image sequence, predictions, and 342 command/feedback records from
  `demo-20261011-154244`, the latest attempt with 50-action chunks.
- All 74 prepared episodes: 56 original full demonstrations and 18 hand
  corrections, plus their metadata and source-run reports.
- Recorder, dataset preparation, training, inference preprocessing, motor
  endpoint, checkpoint configuration, and launcher.
- Offline inference at seven observations each from one original training
  episode, one correction training episode, and one correction validation episode.
  This is 21 observations, not an exhaustive checkpoint evaluation.

Detailed local artifacts are in `act/outputs/act-investigation-20261011/`:
`run-contact-sheet.jpg`, `command-vs-measurement.png`, original/correction
contact sheets and start sheets, `teacher-forced-evaluation.json`,
`preprocessing-parity.json`, `dataset-audit.json`, `episodes-summary.json`, and
`correction-timing.json`. These artifacts are ignored by Git; the findings below
are retained in the repository.

## 1. Separate the two failures

In `155137`, all 342 elbow predictions were clipped at the upper limit. The
gripper stayed at 2028–2031 ticks and the final measured joints were
`[2405,190,3058,1513,3910,2028]`. The beginning of each newly predicted chunk
repeated the near-start portion; later actions containing approach motion were
discarded by the five-action execution window. Restoring 50 actions restores
approach motion; it does not fix grasping.

In `154244`, final measured joints were
`[2553,1891,1616,1304,3535,2350]`. The carton remains visible on the red surface
in the later wrist observations; the overview observations do not show a
successful transfer into the container. Only one image pair per 50 actions is
saved, so a brief contact, slip, or lift between observations cannot be excluded.
The exact contact failure remains unclassified.

## 2. Commands are reaching the arm

For `154244`, mean absolute commanded-versus-measured error, in joint order,
was `[2.3,8.8,10.5,2.3,14.0,4.8]` ticks. Maximum error was
`[7,41,51,17,51,76]`. These are samples taken after each command interval, not
settled accuracy measurements or proof of correct fingertip placement.

During steps 150–199, the gripper followed its target with a mean error of
2 ticks. During 200–249, its mean error was also 2 ticks. The policy commanded
this part of the sequence; the arm was not simply ignoring it. Compliance,
backlash, geometry, and grip contact are not measured by this report.

The earlier `152923` gripper overload is a separate recorded failure. There is
no reported communication or overload error in `154244`.

## 3. Training and deployment preprocessing agree

For correction episode `episode-1791718207653404369`, frame 174, the training
dataset/preprocessor and deployment/preprocessor produced exactly equal state,
wrist-image, and overview-image tensors: maximum absolute difference 0 for each.
The image shapes were both `[1,3,240,320]`; state shapes were `[1,6]`.

Decoding the entire predicted action tensor versus decoding each action as the
runner does also gave maximum difference 0. Camera names and joint order agree
in the inspected code/configuration. This rules out those specific processing
differences, not physical changes in camera mounting or actual frame latency.

JPEG compression is not established as a cause. The data preserves raw joint
ticks losslessly and uses the same intended image dimensions at inference.

## 4. Demonstration pauses provide conflicting targets

Across all 56 original episodes, the first joint departure greater than 30 ticks
occurs at frame 50 or 51: approximately five seconds at 10 Hz.
`prepare_corrections.py` applies its pause removal only to corrections, leaving
these original waits intact. Across corrections it removed only 44 frames in
total. This does not mean every remaining pause should be deleted; contact and
settling intervals can be useful.

Concrete ambiguity in original training episode
`episode-1791655805468765753`:

- Frames 10 and 40 have exactly equal six-joint states.
- Their mean absolute grayscale image differences at 80×60 are only 0.85/255
  for wrist and 0.92/255 for overview.
- Their targets 50 frames ahead differ by 697 ticks at the shoulder and
  531 ticks at the elbow.

The policy sees one current observation (`n_obs_steps=1`) and no elapsed-time or
task-phase input. Near-identical observations therefore ask it to predict very
different futures. This supports a learned hesitation/averaging explanation,
particularly for repeatedly using the beginning of a predicted chunk. It does
not prove that all late grasp failures arise from pauses.

## 5. Prediction errors exist before physical execution

Using actual successful-demo images and measured states as inputs, mean absolute
error over all six joints and all 50 predicted actions was:

| Inspected episode group | Seven-observation mean error |
| --- | ---: |
| Original training | 40.86 ticks |
| Correction training | 38.63 ticks |
| Correction validation | 72.39 ticks |

These are diagnostic samples, not replacement validation metrics. Errors vary
greatly by phase. In the correction training episode, frame 232, the predicted
50th base target was 2361 versus a demonstrated target of 2188: a 173-tick
difference during transfer. In the validation episode, frame 142, shoulder and
elbow chunk MAEs were 294 and 350 ticks. Training episodes themselves also have
gripper chunk errors near 90–97 ticks at some inspected observations.

Thus accurate imitation of critical motion is not established by the reported
global validation score. The existing validation samples only four evenly
spaced observations per episode and does not measure grasp or placement success.

## 6. The runtime stops by count, not task completion

`demo.sh` requests 342 motor steps. `policy_session.py` sets `completed=True`
when it sends that many steps, then holds the arm. It has no learned end signal
or visual success assessment. “Finished” does not mean “picked and placed.”

Seventeen corrections started after 150 autonomous steps. Their hand portions
contain 249–405 retained frames, giving combined recorded lengths of 399–555
frames. One older correction started after 342 steps and has 427 hand frames.
The approximate start of transfer, measured only as the first base angle below
2200 ticks, occurs after combined step 342 in 11 of those 17 cases. This is a
trajectory marker, not a detector of lift or release.

Human recording duration does not dictate the required autonomous duration:
pauses differ, and the policy can progress faster or slower. Nevertheless,
342 is not justified as an end-of-task criterion for this corrected dataset.
Simply increasing it cannot repair a missed grasp or a policy stuck at pickup.

## 7. Further mismatches and limitations

- Original demonstrations include different carton orientations and camera
  framing from the inspected correction examples. The current overview mainly
  shows the container; the wrist supplies most visible pickup detail. Image
  variation is real, but this inspection does not establish camera movement as
  the cause or prove that the training coverage is sufficient.
- Some recorded wrist-flex positions are below the live 1070-tick lower limit.
  The original training minimum is 911; correction training minimum is 907.
  There are 541 original-training frames and 562 correction-training frames
  below that wrist limit. Deployment clips these targets. This prevents exact
  execution of some taught paths, especially later transitions. In `154244`
  wrist-flex is not clipped, so this does not explain that run's pickup failure.
  Do not widen limits to fit the demonstrations without physical calibration.
- Fifty actions at 10 Hz means approximately five seconds between visual
  replans, plus capture/inference pauses. There is no image feedback inside a
  chunk. The five-action experiment exposed a separate progress problem, so
  blindly shortening chunks again is not a verified remedy.
- Image sensitivity tests show that changing images changes predictions. This
  is not evidence that predictions follow the object in the correct direction.
- Acquisition timestamps are host read times, not sensor exposure times. The
  saved reports cannot certify camera/state synchronization or secure grip.

## Recommended next work, in order

1. Prepare a new training copy that removes the known automatic start wait and
   compresses demonstrated hesitation in both original and correction groups,
   while retaining grasp/settling context. Preserve recordings and the existing
   checkpoint. Treat camera-only changes and actual object movement separately
   from idle pauses; do not simply delete every repeated joint position.
2. Identify and review taught transitions outside executable wrist limits; do
   not silently clamp training labels and call those corrected demonstrations.
3. Evaluate predicted actions specifically before closure, during closure,
   at lift, and during transfer on held-out episodes. Retain per-joint errors
   and image examples. A single global MAE is inadequate for model selection.
4. Fine-tune only after the above audit, then test a bounded attempt with images
   covering closure and lift. Use an execution budget compatible with corrected
   trajectories and distinguish budget exhaustion from successful completion.
5. Measure object-following with a controlled change in object position and the
   same initial robot/camera setup. This has not been demonstrated yet.

Do not switch algorithms, collect another large batch, claim success from a
completed command count, or promise that more training alone will fix contact.
