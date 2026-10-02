#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/homer_slam_ws/install/setup.bash
if [ -f /network/ros.sh ]; then source /network/ros.sh; fi
exec "$@"
