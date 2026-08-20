# Robot gamepad coordinator

`src/robot_teleop/node.py` is the Pi-side coordinator for the complete robot:

```text
gamepad -> /joy -> robot_teleop
                    |- /teleop/cmd_vel              -> ESP32 base controller
                    |- /soarm/command_delta_ticks   -> SO-ARM bridge
                    `- /set_autonomous               -> ESP32 mode gate
```

The code is deliberately split into two layers:

- `src/robot_teleop/control_model.py` contains controller mappings, dead-man rules, mode
  transitions, axis deadzones, and rate-limited arm deltas. It has no ROS or
  hardware dependency and is unit tested.
- `src/robot_teleop/node.py` is the thin ROS adapter. It only turns `/joy` into the
  three established ROS messages.

## Control layout

| Control | Action | Status |
| --- | --- | --- |
| A | Select Teleop | Intended operator control; verify receiver index before enablement |
| B | Select Autonomous / stopped mode | Intended operator control; verify receiver index before enablement |
| Hold L1 + left stick | Base drive | L1 button 4 and stick axes 0/1 confirmed |
| Hold R1 + left stick | Shoulder pan / lift | Must verify R1 button index and direction |
| Hold R1 + right stick | Elbow / wrist flex | Must verify axis indices and direction |
| Hold R1 + D-pad | Wrist roll / gripper | Must verify axis indices and direction |
| Hold R1 + triggers | Camera/neck up / down | Must verify trigger indices and direction |

It starts in Autonomous mode, so it publishes zero manual base commands and
does not command the arm until Teleop is explicitly selected. L1 and R1 are
separate dead-man switches. Arm movement is limited to three relative ticks per
20 Hz cycle (50 ticks/second at full stick) before the existing SO-ARM bridge
applies its own 20-tick cap.

## Verification before installation

Do **not** enable this service until each indicated Xbox mapping is confirmed
from the Pi, with the arm supported and base wheels clear of the floor:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joy
```

The current source uses defaults based on Xbox receiver conventions; only L1,
the left-stick axes, and the base teleop mapping have been confirmed. R1, A,
B, right-stick, D-pad, and trigger mappings remain assumptions. See the
repository root `README.md` for runtime checks and power-safety requirements.

After mapping confirmation, copy this repository to `~/mobile-robot` on the
Pi, install `systemd/robot-teleop.service` in `~/.config/systemd/user/`, and run:

```bash
systemctl --user daemon-reload
systemctl --user enable --now robot-teleop.service
```

The ESP32 must first run the firmware in `src/main.cpp`, which explicitly
subscribes to `/teleop/cmd_vel`. The test suite runs without ROS:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```
