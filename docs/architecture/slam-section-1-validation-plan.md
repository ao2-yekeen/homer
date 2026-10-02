# Section 1 MVP — lidar SLAM with live RViz

Status: **REQUIRES_HARDWARE_TEST**. No autonomous drive publisher is added.

## Geometry and scope

The operator requested use of the canonical URDF position for the temporary
LDS-01. Keep `geometry:=urdf`: scan origin relative to `base_link` is
`x=0.090, y=0.075, z=0.093` metres, `roll=pitch=yaw=0` radians. `base_link`
is the ground projection of the drive-wheel midpoint; x forward, y left,
z up. The model base outline is 0.318 × 0.350 m.

Mounting must match this selected transform. URDF selection is not independent
confirmation of physical mounting, scan orientation, or the stowed arm's full
footprint. Check physical alignment before a movement trial. The original
measured-override template remains available if a discrepancy is found.

Retain the working LDS-01 driver and current 0.12–3.5 m processing limits,
checking those limits against `/scan` metadata. Keep encoder odometry disabled.

## Runtime and read-only checks

On the Pi, after deploying/building the package:

```bash
source /opt/ros/jazzy/setup.bash
source ~/homer_slam_ws/install/setup.bash
source ~/.config/homer-slam/network/ros.sh
systemctl --user start homer-slam.service
ros2 run homer_slam mvp_check ~/homer-evidence/section-1/live.json --seconds 15
```

The check warms up discovery for three seconds and waits up to 30 more seconds
for publishers and TF after lifecycle startup. It then captures valid returns,
publisher counts, map occupancy, pose validity, timestamps and TF, and writes
JSON exclusively: choose a new path for every trial. Quick live checks default
to 10 seconds; the explicit longer commands below are qualification trials,
not required for routine development. Exit 0 means that check's
gates passed; exit 1 means failed or unconfirmed gates. Missing observations
are never passes. Live-check acceptance alone does not complete this MVP.

Use `python3 -m homer_slam.mvp_check` as an alternative when running directly
from source with `PYTHONPATH=~/mobile-robot/pi/homer_slam`.

The laptop's existing temporary live viewer can be reopened with:

```bash
bash ~/.local/state/homer-slam/lds01-live-view/start.sh
```

Run only one `homer-lds01-live-view` container. Live laser points are enabled
in the checked-in RViz configuration. The SSH relay is read-only; the RViz
initial-pose tool does not send a pose back to the Pi. It does not qualify
bidirectional ROS or independently synchronized host clocks.

## Physical checklist — complete in order

1. Secure lidar and confirm its mounting matches the selected URDF transform.
2. Inspect a broad obstacle in front, left, and right. Enable scan points and
   confirm their physical correspondence. For one reference distance measured
   from the optical scan origin, run:

   ```bash
   ros2 run homer_slam mvp_check ~/homer-evidence/section-1/reference.json \
     --seconds 15 --reference-angle-deg 0 --reference-distance-m ACTUAL_DISTANCE
   ```

   The angle is in the laser frame. A ±2° sector is used; keep the target
   surface broad enough to cover that sector. Pass: distance error ≤10 cm.
3. Leave the robot/lidar stationary throughout one full 60-second recording:

   ```bash
   ros2 run homer_slam mvp_check ~/homer-evidence/section-1/stationary.json \
     --mode stationary --seconds 60 --operator-confirmed
   ```

   Pass: maximum observed odometry excursion <5 cm and <3°. Only supply
   `--operator-confirmed` if physical immobility is established.
4. Mark a start pose. Start the next capture, make a short movement using
   existing teleoperation, and return to that physical mark and heading:

   ```bash
   ros2 run homer_slam mvp_check ~/homer-evidence/section-1/return.json \
     --mode return --seconds 90 --operator-confirmed
   ```

   Finish the return before recording ends. Pass: endpoint error <20 cm and
   <5°, observed translation at least 20 cm, and visually aligned map walls.
   Physical return must be confirmed; estimator coordinates cannot prove it.
   If the return was not completed, invalidate the trial and repeat. The helper
   publishes no motion commands.
5. Export and reload once using the commands below; compare live scans with
   saved-map walls. Capture a screenshot and note physical correspondence.

## Save/reload and rollback

Run save on the estimator host; destination must not already exist:

```bash
ros2 run homer_slam save_map ~/homer-maps/section-1-mvp
systemctl --user stop homer-slam.service
ros2 launch homer_slam localization.launch.py geometry:=urdf \
  min_range:=0.12 max_range:=3.5 map_file:=$HOME/homer-maps/section-1-mvp/map
```

From another sourced Pi shell, capture a 15-second live check to a new
`reload.json` evidence path. Nonempty files and live checks establish technical
reload; physical scan/map alignment still requires operator observation.

Stop localization with Ctrl-C before restarting `homer-slam.service`. Restarting
mapping creates a fresh map origin; the saved map is preserved. Do not launch
manual processing alongside the service. Teleoperation and driver services
remain independent. Restore the original lidar only using the driver/range
rollback in the weekend handover.

## Acceptance and deferred work

- [ ] Physical mounting and front/left/right alignment checked.
- [ ] Reference distance error ≤10 cm.
- [ ] Stationary trial passed, with operator confirmation.
- [ ] Movement/return trial passed, with operator confirmation and aligned walls.
- [ ] Nonempty occupancy map and pose graph saved and reloaded.
- [ ] Live scan, odometry, TF and map checks passed without persistent stale data
      or duplicate publishers.

Record evidence paths, configuration, screenshots, observations and failures in
the [implementation results](slam-section-1-results-2026-10-02.md).

Three room loops, repeated reloads, ten-minute qualification, full stowed
footprint validation, direct control transport and fault/stopping tests are
deferred. Complete them before autonomous exploration. No MVP result enables
autonomous movement automatically.
