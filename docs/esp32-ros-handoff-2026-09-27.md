# ESP32 and ROS handoff — 2026-09-27

This records the current state for the next agent. The working tree is on
`main` at `b605591`, with **uncommitted** changes in `deploy.sh`,
`firmware/src/config/robot_config.h`,
`firmware/src/micro_ros_node/micro_ros_node.cpp`,
`firmware/src/micro_ros_node/micro_ros_node.h`, and
`pi/micro-ros-agent.service`. Do not discard those changes.

## Verified findings and changes

- The Pi's device routing file is
  `~/.config/mobile-robot/device-routes.json`. It records the ESP32 as
  `/dev/ttyUSB1`, the LiDAR as `/dev/ttyUSB0`, and the RGB camera by its V4L
  identity. The discovery service exposes the ESP32 at
  `/run/user/1002/mobile-robot-devices/esp32` (currently a link to
  `/dev/ttyUSB1`). The installed Agent had used a fixed, wrong USB port.
  `pi/micro-ros-agent.service` now uses the discovery link at 115200 baud,
  without an `esptool` reset in its startup path. The updated unit was copied
  to the Pi and started.
- ROS initialization formerly stopped forever on any failed call.
  `MicroRosNode` now waits for an Agent, creates its ROS entities, and can
  release and recreate them after a connection failure. It pings the Agent
  only while disconnected. A previous version also pinged during an active
  session; Agent logs showed repeated session teardown and recreation.
- `/odom` previously appeared in the ROS graph but delivered no message.
  Its Odometry payload exceeds the micro-ROS library's 512-byte best-effort
  serial frame. The firmware now publishes `/odom` reliably, with a bounded
  publisher timeout, and schedules it every 200 ms. A complete `/odom`
  message was received. `/drive/status` remains best effort and also
  delivered a message.
- `deploy.sh` now selects boot data from the same Arduino core as the build,
  flashes through the discovered ESP32 link at 460800 baud, verifies the
  bootloader, partitions, boot data, and application, installs the updated
  Agent unit, and requires one message from each ESP32 telemetry topic before
  reporting success. This is a minimal presence check, per the user's request;
  no topic rate measurement is required.

## Last deployment and recovery test

`./deploy.sh esp32dev` completed successfully after the latest firmware
change. Its application SHA-256 was
`e1379e815e60d29595ca6d1002d777c3f682313ac282dfa09b3208c2d2403ab0`.
The script verified all four flash regions and received one `/odom` and one
`/drive/status` message. An independent restart of
`micro-ros-agent.service` was followed by successful receipt of both topics
without reflashing. The Pi is reachable with the `pi` SSH alias and the
installed Agent was active at the last completed check.

The earlier saved firmware image had SHA-256
`3dbd73a7f82d918a58ebe99773e0c4c68372d0586188773114a7771b070c66d7`.
A previous fresh build was judged disconnected while a direct serial capture
held the port; that test could not establish Agent connectivity. The later
deployment and ROS checks above are the useful evidence for the current
image.

## Open issues and limits of verification

- A historical serial capture of an earlier image contained Guru Meditation
  panics. No new reset-to-boot serial capture or extended stability run was
  taken for the latest image. The latest test proves connection and recovery
  across one Agent restart, not long-term freedom from panics or session churn.
- The final check of independent devices was interrupted. Earlier checks had
  `/joy`, RGB image, and a ToF depth sample. They found no responding LiDAR
  link or `/scan`, and the SO-ARM bridge lacked `/dev/robot-soarm`. Treat
  these as earlier observations, not a current all-healthy claim.
- The firmware build succeeds but emits warnings where ROS cleanup and the
  best-effort status publish return values are ignored. Review those when
  continuing stability work.
- The changes are deployed but not committed. Before further edits, inspect
  `git diff`, the active Pi unit, and the currently flashed image. Keep the
  next health check limited to the checks needed to resolve a concrete issue.

No motion commands were sent during these checks.
