#!/usr/bin/env bash
# Periodic snapshot: refresh the mirrored Pi config, commit whatever changed
# locally, and push. No-ops (no commit/push) if nothing changed.
set -euo pipefail

REPO_DIR="/home/ao2-yekeen/PlatformIO/Projects/esp32_robot_bt"
cd "$REPO_DIR"

timeout 8 ssh -o ConnectTimeout=4 pi "cat ~/.config/systemd/user/micro-ros-agent.service" \
  > pi/micro-ros-agent.service 2>/dev/null || true
timeout 8 ssh -o ConnectTimeout=4 pi "cat ~/.config/systemd/user/gamepad-teleop.service" \
  > pi/gamepad-teleop.service 2>/dev/null || true

git add -A
if ! git diff --cached --quiet; then
  git commit -m "auto: periodic sync $(date '+%Y-%m-%d %H:%M')"
  git push origin main
fi
