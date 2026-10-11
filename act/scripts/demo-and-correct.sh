#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ACT_DIR/.venv/bin/python" -u "$ACT_DIR/training/demo_and_correct.py" "$@"
