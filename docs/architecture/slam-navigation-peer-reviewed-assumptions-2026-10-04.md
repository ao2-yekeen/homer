# Peer-reviewed audit of the live SLAM/navigation plan

Historical audit for the earlier SLAM/navigation task. Its superseded execution
plans have been removed. Evidence and physical qualification requirements
still apply; current task context is in the [architecture guide](README.md).

Date: 2026-10-04. Scope: RViz-selected goals on a growing map, using Homer’s
RGB camera and LDS-01 2D lidar. This is a targeted literature audit, not a
systematic review or hardware qualification. Published methods establish
feasibility under their tested conditions; they do not validate Homer’s build.

## Decision

The SLAM + global planner + local obstacle controller architecture is supported.
The proposed promise of fixing odometry by changing to ICP is not supported.
Camera/2D-lidar metric odometry is supported as a research method, conditional
on calibration, correspondence, observability and computation. A readily
reproducible ROS 2 implementation for Homer has not been established.

Treat the reported 50 cm actual / approximately 9 cm estimated trial as a failed
system test. It neither identifies the root cause nor proves all lidar odometry
is inadequate. Do not reuse that estimator as the sole movement-distance limit.

## Corroboration standard added after user review

Use multiple independent studies where available. Count the published paper,
its arXiv copy, author repository and tutorial about it as one underlying study.
A survey provides corroborating synthesis, not an independent replication of
the cited experiment. Related authorship, reused datasets and different sensor
hardware must be reported. Source count does not outweigh applicability.

For each implementation decision, record:

1. The precise claim and at least two relevant peer-reviewed sources when
   available, including limitations or contradictory findings.
2. Whether support is direct on comparable hardware, a general principle,
   or an engineering inference. Do not turn partial support into full approval.
3. Compatibility with Homer's actual sensors, geometry, timing, actuator
   interface and processing resources.
4. A reproducible experiment with an independent reference and evidence of
   failures as well as successes. Agreement between two estimators is not a
   ground-truth measurement.

If direct corroboration is missing, mark the claim provisional and experimentally
resolve it. Literature-supported feasibility and validated-on-Homer operation
are separate statuses. No source establishes the cause of the 50 cm/9 cm trial.

| Decision | Corroborating sources | Strength and remaining gap |
| --- | --- | --- |
| Compare lidar estimators rather than assume ICP wins | P1 RF2O experiments; P2 planar ICP; P11 independent ICP benchmark | Multiple experiments demonstrate conditional registration performance; P11 is 3D and does not rank the LDS-01 candidates |
| Add metric constraints before using monocular displacement | P10 independent VO tutorial; P4 range-aided VO experiment | Scale ambiguity is independently supported; P4 is the direct planar-lidar fusion study reviewed, not two independent reproductions on Homer-like hardware |
| Treat scene ambiguity and wrong correspondences as real failure modes | P9 broad SLAM survey; P11 registration experiments; P3 corridor discussion | General mechanisms corroborated; Homer's actual root cause remains unknown |
| Separate global planning from local obstacle control | P5 Nav2 system; P6 independent local-control method | Components have independent support; P5/P8 share an author and are not independent replications of the integrated stack |
| Measure actuation and stopping rather than rely on PWM labels | P6 physically admissible velocities; P12 actuation/low-level-control survey | Requirement corroborated; conversion and braking numbers remain robot-specific measurements |
| Combine simulation with measured real behaviour | P7 system-identification/transfer experiment; P12 independent multi-platform survey | Transfer limits corroborated; neither establishes the real-to-sim gap of this configuration |

## Assumption register

| Assumption | Verdict | Evidence | Consequence for Homer |
| --- | --- | --- | --- |
| Planar lidar can estimate metric base movement without wheel odometry | Supported under suitable conditions | [P1], [P3] | Inspect frames, inputs, scene constraints and implementation; current physical failure remains unresolved |
| Replacing RF2O with ICP will improve accuracy | Not established | [P1], [P2], [P3] | Run a controlled comparison; retain no replacement merely because it installs successfully |
| LDS-01’s roughly 5 Hz rate alone explains the failure | Not established | [P1] includes 5 Hz tests | Test acquisition timing, frame-to-frame motion and geometry rather than prescribe a frequency fix |
| Monocular RGB plus planar lidar can recover metric scale | Supported as a method, conditional for this robot | [P4] | Require valid range/visual association and ongoing scale constraints; evaluate an implementation before selecting it |
| Camera optical flow or appearance loop closure supplies independent metric distance | Not established | [P3], [P4] | Distinguish visual tracking, loop correction and metric local odometry |
| SLAM and goal navigation can run together | Architecture supported | [P5], [P8]; exact Jazzy workflow additionally documented in [D1] | Retain SLAM Toolbox mapping; add Nav2 without requiring saved-map localization |
| Local obstacle handling must wait for every SLAM map publication | Unnecessary architectural coupling | [P5], [P6] | Feed scans into the local obstacle model; independently measure local response latency |
| Existing arbitrary PWM scaling is sufficient for a velocity-based local controller | Unsupported for Homer | [P6] assumes realizable velocities/dynamics | Qualify actual speed response and braking; a command-number cap is not a measured speed limit |
| Two agreeing estimators or fresh timestamps prove accurate motion | Not established | [P3] discusses missing geometric constraints; [P4] requires valid association | Correlated failures and weak observability remain possible; quality checks need their own qualification |
| A simulated pass qualifies physical navigation | Not established | [P7] relies on identification and actuator/latency modelling | Keep real replay, shadow operation and physical command/stopping qualification |
| 3 cm, 10 cm and 250 ms are literature-validated limits for Homer | No | These were plan proposals | Derive operating limits from measured error, footprint, sensor coverage, dynamics and stopping envelope |

## Papers and their actual relevance

### P1. Jaimez et al., ICRA 2016 — RF2O

*Planar Odometry from a Radial Laser Scanner: A Range Flow-based Approach.*
[Full paper from an author’s university](https://cvg.cit.tum.de/_media/spezial/bib/jaimez2016icra.pdf).

RF2O estimates planar motion from consecutive scans using robust range-flow
alignment. The model assumes a static scene and uses coarse-to-fine processing
to address linearization limits. Section VII compares RF2O, PL-ICP and PSM in
simulation and on a robot. It includes 5 Hz cases and reports RF2O outperforming
the alternatives in its simulated scenarios; PL-ICP performs poorly in a
corridor. Its sensor geometry/range differs from LDS-01. This contradicts a
blanket claim that ICP is better or 5 Hz necessarily prevents usable odometry.

### P2. Censi, ICRA 2008 — point-to-line ICP

*An ICP Variant Using a Point-to-Line Metric.* DOI: 10.1109/ROBOT.2008.4543181.
[Author’s paper](https://censi.science/pub/research/2008-icra-plicp.pdf).

The paper demonstrates convergence/precision benefits of its point-to-line
variant while noting reduced robustness to very large initial displacement
errors. It supports scan-matching comparison, not universal ICP superiority.
PLICP is not identical to RTAB-Map’s implementation; keep that distinction in
benchmark reports. The author-paper text was available through search indexing;
a direct open failed during this audit.

### P3. Labbé and Michaud, Journal of Field Robotics 2019 — RTAB-Map

*RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and
Long-Term Online Operation.* DOI: 10.1002/rob.21831.
[Published record](https://onlinelibrary.wiley.com/doi/10.1002/rob.21831),
[author manuscript uploaded in 2024](https://arxiv.org/html/2403.06341v1).

The paper separates odometry from the graph backend. Section 3.1.2 explains
that ICP can miss corridor-direction velocity changes where geometry provides
no constraint; external odometry can supply that missing information. Visual
configurations evaluated use stereo/RGB-D, and synchronization/extrinsics are
explicit requirements. This supports ICP as a candidate and calibration work,
not a proven monocular-RGB/LDS-01 replacement. Constant-velocity prediction can
continue despite weak translation observability; successful registration alone
must not be treated as independent proof of metric movement.

### P4. Chiodini et al., IEEE Transactions on Instrumentation and Measurement 2020

*Retrieving Scale on Monocular Visual Odometry Using Low-Resolution Range
Sensors.* 69(8), 5875–5889. DOI: 10.1109/TIM.2020.2964066.
[Authors’ university publication record](https://research.unipd.it/handle/11577/3342438),
[paper text on ResearchGate](https://www.researchgate.net/publication/338444124_Retrieving_Scale_on_Monocular_Visual_Odometry_Using_Low-Resolution_Range_Sensors).

The paper combines monocular landmark tracking with range constraints and
bundle adjustment to maintain metric scale. It tests a planar SICK lidar as
well as low-resolution ToF, with independent references. Range points are
associated to visual observations using calibrated transforms. It supports
RGB/2D-lidar feasibility, but not assigning a scan depth to every image pixel.
Its computation discussion does not establish navigation-rate operation on
Homer. Valid overlap/association, texture and calibration must be checked.
The university lists its PDF as restricted; sections III–VI were inspected in
the accessible paper transcription. Public reusable ROS 2 code was not verified.

### P5. Macenski et al., IROS 2020 — Navigation2

*The Marathon 2: A Navigation System.*
[Peer-reviewed venue record from the research organization](https://research.samsung.com/research-papers/The-Marathon-2-A-Navigation-System),
[full manuscript](https://arxiv.org/pdf/2003.00368).

Navigation2 separates planner, controller and recovery servers, orchestrated
through behavior trees and actions, and integrates perception/environmental
models. It demonstrates navigation on a different robot. This supports using
a navigation stack around SLAM rather than expecting SLAM to command obstacle
avoidance. It does not verify Homer’s motor scaling, endpoint tolerance or
three-second publication interval. Exact present-day package wiring requires
version-matched documentation and tests.

### P6. Fox, Burgard and Thrun, IEEE Robotics & Automation Magazine 1997

*The Dynamic Window Approach to Collision Avoidance.* 4(1), 23–33.
DOI: 10.1109/100.580977.
[Author manuscript hosted by CMU](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf).

The method searches velocities reachable under robot dynamics and excludes
motions unable to stop before obstacles. This supports explicit velocity,
acceleration and braking calibration. Applying that requirement to Homer’s
PWM interface is an engineering inference; the original robot differs from
Homer. Nav2 DWB is related to this approach, not an exact reproduction or a
collision guarantee. Neither sensor blindness nor wrong dynamics is repaired
by configuring nominally conservative velocity values.

### P7. Tan et al., Robotics: Science and Systems 2018 — simulation transfer

*Sim-to-Real: Learning Agile Locomotion for Quadruped Robots.*
DOI: 10.15607/RSS.2018.XIV.010.
[Conference proceedings](https://roboticsproceedings.org/rss14/p10.html),
[full paper](https://roboticsproceedings.org/rss14/p10.pdf).

This work uses system identification, actuator modelling, latency simulation
and randomized conditions for quadruped-policy transfer. It supports those
transfer principles, not guaranteed Gazebo-to-Homer navigation equivalence.
Using them here is an inference across platform and algorithm differences.
Simulator fidelity still depends on measured parameters; randomized unknowns
cannot replace real direction, stopping and sensor validation.

### P8. Macenski and Jambrecic, Journal of Open Source Software 2021

*SLAM Toolbox: SLAM for the Dynamic World.* 6(61), 2783.
DOI: 10.21105/joss.02783.
[Peer-reviewed publication and review record](https://joss.theoj.org/papers/10.21105/joss.02783),
[full paper](https://joss.theoj.org/papers/10.21105/joss.02783.pdf).

This software paper supports online mapping and the choice to retain SLAM
Toolbox. It is not experimental validation of Homer’s odometry or motor control.
Map serialization is a capability, not a prerequisite for the selected live-map
goal task.

### P9. Cadena et al., IEEE Transactions on Robotics 2016

*Past, Present, and Future of Simultaneous Localization and Mapping: Toward the
Robust-Perception Age.* 32(6), 1309–1332. DOI: 10.1109/TRO.2016.2624754.
[Full published paper](https://robotics.caltech.edu/wiki/images/1/1a/PastPresentFutureSLAM.pdf).

The survey discusses wrong data associations, perceptual aliasing, dynamic
scenes and the dependence of maturity on robot/environment/performance
requirements. It corroborates considering failure mechanisms and deployment
conditions. It does not diagnose Homer or replicate P4's fusion experiment.

### P10. Scaramuzza and Fraundorfer, IEEE Robotics & Automation Magazine 2011

*Visual Odometry: Part I — The First 30 Years and Fundamentals.* 18(4).
[Author's full tutorial paper](https://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf),
[publication listing](https://rpg.ifi.uzh.ch/visual_odometry_tutorial.html).

The peer-reviewed tutorial explains that ordinary monocular image geometry
recovers translation up to scale; metric information must come from additional
measurements or validated constraints. This independently supports the scale
requirement. It does not validate a specific RGB/2D-lidar fusion implementation.

### P11. Pomerleau et al., Autonomous Robots 2013

*Comparing ICP Variants on Real-World Data Sets: Open-Source Library and
Experimental Protocol.* 34(3), 133–148. DOI: 10.1007/s10514-013-9327-2.
[Author's full manuscript](https://www.franciscolas.net/publications/2013_Pomerleau_AutonomousRobots_Comparing.pdf),
[author-lab publication record](https://norlab.ulaval.ca/bibliography/Pomerleau2013b/).

The study evaluates sensitivity to initial pose, scan overlap and environment.
Its registration experiments use 3D point clouds; extrapolation to a 2D
scanner is conditional. It supports a controlled benchmark and warns against
assuming package or algorithm identity guarantees a correct registration.

### P12. Aljalbout et al., Annual Review of Control, Robotics, and Autonomous Systems 2026

*The Reality Gap in Robotics: Challenges, Solutions, and Best Practices.*
9, 403–432. DOI: 10.1146/annurev-control-031924-100130.
[Published review](https://www.annualreviews.org/content/journals/10.1146/annurev-control-031924-100130),
[author manuscript](https://rpg.ifi.uzh.ch/docs/AR25_Aljalbout.pdf).

The publisher assigns 2026, despite the author's PDF filename containing 25.
This review covers sensing, dynamics, actuation and system-design gaps across
robotics. Its actuator and low-level-control discussion includes nonlinear
response, latency and conversion between high-level setpoints and motor inputs.
It independently corroborates measuring those gaps and testing real hardware.
It is synthesis of multiple studies, not a new empirical result on Homer or a
prescription that maximum visual fidelity always produces best transfer.

## Plan corrections and implementation decisions

1. Start with the known system failure and inspect actual raw inputs. Keep RF2O
   as a comparison baseline; ICP is an experiment, not the predetermined winner.
2. Before promising camera-aided metric odometry, verify lidar-plane visibility
   in the camera, intrinsic/extrinsic calibration and stable range-to-image
   association. A shared view alone does not prove valid correspondences.
3. If adopting P4’s approach, make it a separate measured estimator prototype:
   landmark tracking → calibrated range constraints → scale/pose optimization.
   Benchmark processing rate and failure detection before navigation integration.
4. Derive a low-speed operating envelope from measured tracking quality,
   geometry/texture conditions, command response and braking. Stop outside that
   validated envelope; do not promise arbitrary unknown-room reliability.
5. Use direct scan updates for the local obstacle model and retain live SLAM
   for global planning. Measure local latency separately from map rendering.
6. Treat covariance, residuals, correspondence counts and degeneracy measures as
   diagnostic inputs, not calibrated confidence by default. Prove how the gate
   handles fresh-but-wrong poses; fail deployment if that remains unresolved.
7. Separate injected-fault supervisor tests from diagnosis. Artificially imposing
   50 cm/9 cm tests a response path; it does not establish the physical cause.
8. Keep real speed/stopping qualification after simulation. Endpoint thresholds
   and fault timeouts remain engineering proposals until validated physically.

## Documentation sources, not peer-reviewed proof

[D1] [Nav2 Jazzy: navigating while mapping](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/).
[D2] [Nav2 Jazzy Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/).
[D3] [RTAB-Map ROS 2 odometry documentation](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/README.md).

These support interfaces/configuration. They do not convert package availability
into peer-reviewed experimental proof for Homer. Preprints, blog posts, forum
success stories, and 3D-lidar/IMU results were not used to certify the present
RGB/2D-lidar configuration.
