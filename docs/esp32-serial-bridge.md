# ESP32 serial bridge

The production firmware keeps motor, encoder, odometry, and neck control on
the ESP32. `pi/esp32_serial_bridge.py` provides the same ROS command and
telemetry topics as the former micro-ROS image. `/drive/link` is a transient
local `std_msgs/Bool` that reports whether a HOM1 greeting was received.

## Wire format

The USB serial port runs at 115200 8N1. Each ASCII frame is at most 128 bytes:

```text
HOM1|TYPE|SEQUENCE|FIELD...|CRC16\n
```

CRC16 is CCITT-FALSE (initial value `0xffff`, polynomial `0x1021`), computed
over everything before the final `|CRC16`. The checksum is four uppercase hex
digits. The sequence wraps at 65535. Invalid and incomplete frames are ignored.

Pi to ESP32: `T` teleop drive and `A` autonomous/override drive contain linear
and angular values; `M` selects mode 0 teleop or 1 autonomous; `V` sets mode
override (`-1` clears, `0..2` selects); `N` sets neck angle 80..150; `S` stops
the motors. ESP32 to Pi: `R` carries a boot ID, `O` carries uptime milliseconds
and x, y, heading, linear velocity, angular velocity; `D` carries uptime,
command linear, left/right PWM, command angular, mode, and command age.

The ESP32 sends `R` every second and odometry/status every 200 ms. It processes
at most 96 input bytes per control loop and skips telemetry when the serial
transmit buffer is full. The Pi discards pending commands on disconnect. It
sends `S` and clears override when a fresh greeting arrives; motion requires
new ROS commands after reconnection. The ESP32 stops its motors after one
second without an accepted drive command even if the bridge dies.

## Cutover and rollback

Run `./deploy.sh esp32dev`. It saves the previously staged firmware and Agent
unit in `~/esp32_fw/backups/before-serial-<UTC time>` on the Pi, verifies each
flashed region, enables `esp32-serial-bridge.service`, and checks `/drive/link`,
`/odom`, and `/drive/status`. The two serial services must not run together.

To restore the saved image, stop and disable the serial bridge on the Pi,
then use the same `esptool --no-stub write_flash` and `verify_flash` offsets in
`deploy.sh`, pointing at the four files in the selected backup directory.
Restore the saved `micro-ros-agent.service` to
`~/.config/systemd/user/`, run `systemctl --user daemon-reload`, and enable
and start the Agent. This returns to the verified pre-migration image without
rebuilding it.

For non-motion checks, use:

```bash
systemctl --user status esp32-serial-bridge.service
ros2 topic echo /drive/link std_msgs/msg/Bool --once --qos-durability transient_local
ros2 topic echo /odom --once
ros2 topic echo /drive/status --once --qos-reliability best_effort
```

## Verification on 2026-09-27

The serial image was flashed and all four regions passed `verify_flash`.
The bridge reported a HOM1 boot greeting and provided `/odom` at 5 Hz and
`/drive/status`. A repeated zero `/cmd_vel` stream left both motor PWM values
at zero and reduced command age to 27 ms. Bridge service restarts and one
ESP32 reset reconnected without reflashing. The saved micro-ROS image is in
`~/esp32_fw/backups/before-serial-20260927T191036Z` on the Pi.
Corrupt and oversized serial frames were ignored while ESP32 telemetry
continued.

Physical USB replug, multiple cold power cycles, active motor and neck
commands, and an extended idle run still need validation before claiming
long-term reliability.

## Future work: prevent firmware mismatches

The 2026-09-28 reverse-drive investigation found that the ESP32 was running an
older image while the Pi held the current production binary. The older image
sent an odometry frame the bridge could not parse. The exact flash command
that installed it was not recorded.

- [ ] Include a firmware build ID and serial protocol version in the ESP32 boot
  greeting. Have the Pi bridge reject incompatible versions, keep
  `/drive/link` false, send a stop command, and log the observed and expected
  versions.
- [ ] Make deployment record the source commit and image hash, then compare
  them with the image read back from the ESP32 after flashing. Keep diagnostic
  images clearly labelled and document how to restore production firmware.
- [ ] Monitor receipt of actual `/odom` and `/drive/status` messages, including
  their age and frame validity. Alert when either stream stops or the reported
  drive output disagrees with a sustained command.
- [ ] Validate the safeguards with an incompatible image and a missing
  telemetry stream before relying on them for driving.
