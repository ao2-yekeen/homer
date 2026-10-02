#!/usr/bin/env bash
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SLAM_WORKSPACE="${HOMER_SLAM_WS:-$HOME/homer_slam_ws}"
RF2O_REV=b38c68e46387b98845ecbfeb6660292f967a00d3
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="${MAKEFLAGS:--j1}"
source /opt/ros/jazzy/setup.bash
mkdir -p "$SLAM_WORKSPACE/src"
ln -sfn "$TASK_ROOT/pi/homer_slam" "$SLAM_WORKSPACE/src/homer_slam"
if [ ! -d "$SLAM_WORKSPACE/src/rf2o_laser_odometry/.git" ]; then
  git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git "$SLAM_WORKSPACE/src/rf2o_laser_odometry"
fi
RF2O_SOURCE="$SLAM_WORKSPACE/src/rf2o_laser_odometry"
if [ "$(git -C "$RF2O_SOURCE" rev-parse HEAD)" != "$RF2O_REV" ]; then
  if [ -n "$(git -C "$RF2O_SOURCE" status --porcelain)" ]; then
    echo 'RF2O has local changes; refusing to overwrite them' >&2; exit 1
  fi
  git -C "$RF2O_SOURCE" checkout "$RF2O_REV"
fi
if git -C "$RF2O_SOURCE" apply --check --quiet "$TASK_ROOT/pi/homer_slam/patches/rf2o-startup.patch"; then
  git -C "$RF2O_SOURCE" apply "$TASK_ROOT/pi/homer_slam/patches/rf2o-startup.patch"
elif ! git -C "$RF2O_SOURCE" apply --reverse --check --quiet "$TASK_ROOT/pi/homer_slam/patches/rf2o-startup.patch"; then
  echo 'RF2O patch does not match source; inspect local changes' >&2; exit 1
fi
cd "$SLAM_WORKSPACE"
colcon build --packages-select rf2o_laser_odometry homer_slam --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG"
