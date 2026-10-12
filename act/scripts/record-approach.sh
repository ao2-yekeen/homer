#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo 'Record approach source: support the arm, press s, hand-guide it above the object, then press p to save and exit.'
exec bash "$SCRIPT_DIR/start-demos.sh" --approach-source "$@"
