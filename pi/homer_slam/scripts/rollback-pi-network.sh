#!/usr/bin/env bash
set -euo pipefail
for dropin in "$HOME"/.config/systemd/user/*.service.d/50-homer-network.conf; do
  [ -f "$dropin" ] || continue
  unit="$(basename "$(dirname "$dropin")" .d)"
  rm "$dropin"
  systemctl --user daemon-reload
  if systemctl --user is-active --quiet "$unit"; then systemctl --user restart "$unit"; fi
done
