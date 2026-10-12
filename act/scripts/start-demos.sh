#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Read-only/review modes retain their existing device ownership checks.
for arg in "$@"; do
  case "$arg" in
    --help|-h|--review|--check|--camera-check|--joystick-check)
      exec python3 "$ACT_DIR/runtime/record.py" --no-preview "$@"
      ;;
  esac
done
has_replay=false
recording_approach=false
for arg in "$@"; do
  case "$arg" in
    --replay|--replay=*) has_replay=true ;;
    --approach-source) recording_approach=true ;;
  esac
done
if [[ "$has_replay" == false && "$recording_approach" == false ]]; then
  set -- --replay "$ACT_DIR/data/source" "$@"
fi
if [[ ! -t 0 ]]; then
  echo 'Recording needs an interactive terminal. Connect with: ssh -t pi' >&2
  exit 1
fi
echo 'Releasing cameras and arm for demonstration recording...'
# Stop the trigger first so opening the arm cannot restart its other owner.
if [[ "$(systemctl --user show soarm-bridge.path --property=LoadState --value)" == loaded ]]; then
  systemctl --user stop soarm-bridge.path
fi
systemctl --user stop soarm-bridge robot-teleop rgb-object-detection
for service in gamepad-teleop gamepad-arm-test; do
  if [[ "$(systemctl --user show "$service" --property=LoadState --value)" == loaded ]]; then
    systemctl --user stop "$service"
  fi
done
echo 'Controllers remain stopped after exit. Support the arm during guidance.'
exec python3 "$ACT_DIR/runtime/record.py" --no-preview "$@"
