#!/usr/bin/env bash
# Run on the Pi. Generates a single profile and backs up existing managed drop-ins.
set -euo pipefail
TASK_PACKAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-hotspot}"
NETWORK_DIR="$HOME/.config/homer-slam/network"
PYTHONPATH="$TASK_PACKAGE${PYTHONPATH:+:$PYTHONPATH}" python3 -m homer_slam.network \
  --config "$TASK_PACKAGE/config/network.json" --profile "$PROFILE" --role pi --output "$NETWORK_DIR"
BACKUP_DIR="$HOME/.local/state/homer-slam/network-backups/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$BACKUP_DIR"
UNITS=(device-core-discovery.service esp32-serial-bridge.service rplidar.service \
  lidar-scan-watchdog.service tof-depth-publisher.service rgb-object-detection.service \
  gamepad-joy.service robot-teleop.service soarm-bridge.service)
for unit in "${UNITS[@]}"; do
  if ! systemctl --user cat "$unit" >/dev/null 2>&1; then continue; fi
  dropin="$HOME/.config/systemd/user/$unit.d/50-homer-network.conf"
  mkdir -p "$(dirname "$dropin")"
  if [ -f "$dropin" ]; then cp "$dropin" "$BACKUP_DIR/$unit.conf"; fi
  printf '[Service]\nEnvironmentFile=%s\n' "$NETWORK_DIR/ros.env" > "$dropin"
done
systemctl --user daemon-reload
# Restart only services already active; do not enable previously disabled cameras/arm.
for unit in "${UNITS[@]}"; do
  if systemctl --user is-active --quiet "$unit"; then systemctl --user restart "$unit"; fi
done
printf 'Applied %s. Backups: %s\n' "$PROFILE" "$BACKUP_DIR"
