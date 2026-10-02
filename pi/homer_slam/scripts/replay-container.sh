#!/usr/bin/env bash
# Offline inspection: no Pi discovery, sensor ports, or live estimators.
set -euo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ "$#" -ne 1 ]; then echo "Usage: $0 <recorded-bag-directory>" >&2; exit 1; fi
BAG_DIR="$(realpath "$1")"
[ -f "$BAG_DIR/metadata.yaml" ] || { echo 'Bag metadata.yaml missing' >&2; exit 1; }
if ! docker info >/dev/null 2>&1; then
  if id -nG "$USER" | tr ' ' '\n' | grep -qx docker && ! id -nG | tr ' ' '\n' | grep -qx docker; then
    exec sg docker -c "$(printf '%q ' "$0" "$@")"
  fi
  echo 'Docker daemon/session access unavailable' >&2; exit 1
fi
REPLAY_DIR="$(mktemp -d)"; trap 'rm -rf "$REPLAY_DIR"' EXIT
cat > "$REPLAY_DIR/dds.xml" <<'XML'
<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="94"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>99</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>
XML
cat > "$REPLAY_DIR/view.sh" <<'VIEW'
#!/usr/bin/env bash
set -e
ros2 launch homer_slam view.launch.py use_sim_time:=true &
VIEW_PID=$!
trap 'kill -INT "$VIEW_PID" 2>/dev/null || true' EXIT
ros2 bag play /bag --clock --rate 0.5 --delay 5
# Keep the window open after playback; bag time is paused at the final frame.
wait "$VIEW_PID"
VIEW
docker build -t homer-slam:jazzy -f "$TASK_ROOT/pi/homer_slam/Dockerfile" "$TASK_ROOT"
ARGS=(--rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -e "DISPLAY=${DISPLAY:-:0}"
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e ROS_DOMAIN_ID=94 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -e CYCLONEDDS_URI=/replay/dds.xml -v "$REPLAY_DIR:/replay:ro" -v "$BAG_DIR:/bag:ro")
if [ -d /mnt/wslg ]; then ARGS+=(-v /mnt/wslg:/mnt/wslg); fi
if [ -d /tmp/.X11-unix ]; then ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro); fi
if [ -e /dev/dxg ]; then ARGS+=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib); fi
if [ -d /dev/dri ]; then ARGS+=(--device /dev/dri); fi
echo 'RECORDED DATA: offline RViz replay, not the live robot. Window stays open after playback.'
docker run "${ARGS[@]}" homer-slam:jazzy bash /replay/view.sh
