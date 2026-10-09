#!/usr/bin/env bash
set -euo pipefail
ACT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ACT_DIR/runtime/record.py" --no-preview "$@"
