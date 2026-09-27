#!/usr/bin/env bash
# Install the Pi user services that should start automatically at boot.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOME="$(ssh pi 'printf %s "$HOME"')"
REMOTE_PI_DIR="$PI_HOME/mobile-robot/pi"
REMOTE_UNIT_DIR="$PI_HOME/.config/systemd/user"
PRESERVE_RGB_DISABLED="$(ssh pi 'if test -f "$HOME/.config/systemd/user/rgb-object-detection.service" && test "$(systemctl --user is-enabled rgb-object-detection.service 2>/dev/null)" = disabled; then echo yes; else echo no; fi')"

if [ "$(ssh pi 'loginctl show-user $(id -un) -p Linger --value')" != yes ]; then
  echo "ERROR: Pi user lingering is disabled; enable it before relying on boot startup." >&2
  exit 1
fi

ssh pi "mkdir -p '$REMOTE_PI_DIR' '$REMOTE_UNIT_DIR'"
scp -q "$PROJECT_DIR/pi/device_discovery.py" \
  "$PROJECT_DIR/pi/device_paths.py" \
  "$PROJECT_DIR/pi/wait_for_lidar.py" \
  "$PROJECT_DIR/pi/lidar_scan_watchdog.py" \
  "$PROJECT_DIR/pi/esp32_serial_bridge.py" \
  "$PROJECT_DIR/pi/esp32_port.py" \
  "$PROJECT_DIR/pi/serial_protocol.py" "pi:$REMOTE_PI_DIR/"

for unit in device-core-discovery.service esp32-serial-bridge.service \
            rplidar.service lidar-scan-watchdog.service rgb-object-detection.service \
            soarm-bridge.service soarm-bridge.path; do
  scp -q "$PROJECT_DIR/pi/$unit" "pi:$REMOTE_UNIT_DIR/$unit"
done

ssh pi "set -e
  systemctl --user daemon-reload
  systemctl --user disable --now micro-ros-agent.service || true
  systemctl --user disable --now gamepad-teleop.service || true
  systemctl --user stop soarm-bridge.path || true
  if [ \"\$(systemctl --user is-enabled soarm-bridge.service 2>/dev/null)\" = enabled ]; then
    systemctl --user disable --now soarm-bridge.service
  else
    systemctl --user stop soarm-bridge.service || true
  fi
  systemctl --user enable device-core-discovery.service
  systemctl --user enable --now soarm-bridge.path
  systemctl --user enable gamepad-joy.service robot-teleop.service
  systemctl --user enable tof-depth-publisher.service
  systemctl --user enable esp32-serial-bridge.service rplidar.service lidar-scan-watchdog.service
  if [ '$PRESERVE_RGB_DISABLED' = no ]; then
    systemctl --user enable --now rgb-object-detection.service
    systemctl --user restart rgb-object-detection.service
  fi
  systemctl --user restart rplidar.service || true
  systemctl --user restart lidar-scan-watchdog.service
  for attempt in {1..15}; do
    if systemctl --user is-active --quiet rplidar.service; then break; fi
    sleep 2
  done
  systemctl --user is-active esp32-serial-bridge.service rplidar.service lidar-scan-watchdog.service soarm-bridge.path"
