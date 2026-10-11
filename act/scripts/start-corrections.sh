#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
for arg in "$@"; do
  case "$arg" in
    --help|-h) exec python3 "$ACT_DIR/runtime/record_correction.py" "$@" ;;
  esac
done
if [[ ! -t 0 ]]; then
  echo 'Correction recording needs an interactive terminal: ssh -t pi' >&2
  exit 1
fi
# Match the existing recorder's device ownership handoff.
for service in soarm-bridge.path soarm-bridge robot-teleop rgb-object-detection gamepad-teleop gamepad-arm-test; do
  if [[ "$(systemctl --user show "$service" --property=LoadState --value)" == loaded ]]; then
    systemctl --user stop "$service"
  fi
done
echo 'Controllers remain stopped after exit. Saved-pose mode returns on startup and after saving; h releases torque for recording.'
exec python3 "$ACT_DIR/runtime/record_correction.py" "$@"
