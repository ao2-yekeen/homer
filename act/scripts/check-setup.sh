#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$ACT_DIR/runtime/record.py" --camera-check "$@"
python3 "$ACT_DIR/runtime/record.py" --check "$@"
