#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y ros-jazzy-nav2-map-server ros-jazzy-slam-toolbox ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-fastcdr ros-jazzy-fastrtps \
  ros-jazzy-rclpy ros-jazzy-rosidl-generator-py ros-jazzy-rosidl-runtime-py ros-jazzy-diagnostic-msgs \
  ros-jazzy-std-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-visualization-msgs \
  ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rosidl-typesupport-fastrtps-c ros-jazzy-rosidl-typesupport-fastrtps-cpp \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-py ros-jazzy-tf2-msgs ros-jazzy-tf2-geometry-msgs ros-jazzy-eigen3-cmake-module \
  libeigen3-dev libboost-dev python3-colcon-common-extensions python3-pytest python3-vcstool
