#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PROFILE="${HOMER_NETWORK_PROFILE:-hotspot}"
NETWORK_DIR="$HOME/.config/homer-slam/laptop-$PROFILE"
command -v docker >/dev/null || { echo 'Install Docker Engine in WSL2 (sudo apt install docker.io), then start its service.' >&2; exit 1; }
if ! docker info >/dev/null 2>&1; then
  if id -nG "$USER" | tr ' ' '\n' | grep -qx docker && ! id -nG | tr ' ' '\n' | grep -qx docker; then
    exec sg docker -c "$(printf '%q ' "$0" "$@")"
  fi
  echo 'Docker daemon is unavailable or your session lacks access; start Docker and reopen the session.' >&2
  exit 1
fi
PYTHONPATH="$TASK_ROOT/pi/homer_slam${PYTHONPATH:+:$PYTHONPATH}" python3 -m homer_slam.network \
  --config "$TASK_ROOT/pi/homer_slam/config/network.json" --profile "$PROFILE" --role laptop --output "$NETWORK_DIR"
# Rewrite only the container mount location; retain role/profile/interface selection.
sed 's|^export CYCLONEDDS_URI=.*|export CYCLONEDDS_URI=/network/cyclonedds.xml|' "$NETWORK_DIR/ros.sh" > "$NETWORK_DIR/container.sh"
docker build -t homer-slam:jazzy -f "$TASK_ROOT/pi/homer_slam/Dockerfile" "$TASK_ROOT"
mkdir -p "$HOME/.local/share/homer-slam"
ARGS=(--rm --network host --user "$(id -u):$(id -g)" -e HOME=/tmp -e "DISPLAY=${DISPLAY:-:0}" \
  -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}" -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
  -v "$NETWORK_DIR:/network:ro" -v "$NETWORK_DIR/container.sh:/network/ros.sh:ro" \
  -v "$HOME/.local/share/homer-slam:/data")
if [ -d /mnt/wslg ]; then ARGS+=(-v /mnt/wslg:/mnt/wslg); fi
if [ -d /tmp/.X11-unix ]; then ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro); fi
if [ -e /dev/dxg ]; then ARGS+=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib); fi
if [ -d /dev/dri ]; then ARGS+=(--device /dev/dri); fi
if [ "$#" -eq 0 ]; then set -- ros2 launch homer_slam view.launch.py; fi
exec docker run "${ARGS[@]}" homer-slam:jazzy "$@"
