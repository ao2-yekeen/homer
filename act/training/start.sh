#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
task_root="$PWD"
mkdir -p "$task_root/.tools"
if [ ! -x "$task_root/.tools/uv" ]; then
    printf '%s\n' 'Downloading the local Python installer...'
    curl -LsSf https://astral.sh/uv/install.sh -o "$task_root/.tools/install-uv.sh"
    UV_UNMANAGED_INSTALL="$task_root/.tools" sh "$task_root/.tools/install-uv.sh"
fi
export UV_PYTHON_INSTALL_DIR="$task_root/.tools/python"
export UV_PYTHON_PREFERENCE=only-managed
export UV_CACHE_DIR="$task_root/.cache/uv"
export PIP_CACHE_DIR="$task_root/.cache/pip"
export TORCH_HOME="$task_root/.cache/torch"
export HF_HOME="$task_root/.cache/huggingface"
printf '%s\n' 'Preparing local Python 3.11, then starting training...'
exec "$task_root/.tools/uv" run --no-project --python 3.11 run.py "$@"
