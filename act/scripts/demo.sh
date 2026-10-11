#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ACT_DIR/.venv/bin/python" -u "$ACT_DIR/training/run_policy.py" \
  --checkpoint "$ACT_DIR/outputs/kaggle-corrections-trained/policy" \
  --output "$ACT_DIR/outputs/demo-$(date +%Y%m%d-%H%M%S)" --steps 342 "$@"
