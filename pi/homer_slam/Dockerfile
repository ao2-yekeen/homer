FROM ros:jazzy-ros-base-noble@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 ros-jazzy-nav2-map-server ros-jazzy-slam-toolbox ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-robot-state-publisher ros-jazzy-tf2-geometry-msgs ros-jazzy-eigen3-cmake-module \
    libeigen3-dev libboost-dev python3-colcon-common-extensions python3-pytest \
    python3-yaml git iproute2 mesa-utils && rm -rf /var/lib/apt/lists/*
COPY pi/homer_slam/patches /rf2o-patches
ENV HOMER_SLAM_WS=/opt/homer_slam_ws CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git /opt/homer_slam_ws/src/rf2o_laser_odometry && \
    git -C /opt/homer_slam_ws/src/rf2o_laser_odometry checkout b38c68e46387b98845ecbfeb6660292f967a00d3 && \
    git -C /opt/homer_slam_ws/src/rf2o_laser_odometry apply /rf2o-patches/rf2o-startup.patch && \
    bash -c 'source /opt/ros/jazzy/setup.bash && cd /opt/homer_slam_ws && colcon build --packages-select rf2o_laser_odometry --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG"'
COPY . /project
RUN /project/pi/homer_slam/scripts/build-workspace.sh
RUN dpkg-query -W > /opt/homer_slam_ws/debian-packages.txt
COPY pi/homer_slam/scripts/container-entrypoint.sh /usr/local/bin/homer-entrypoint
ENTRYPOINT ["/usr/local/bin/homer-entrypoint"]
CMD ["ros2", "launch", "homer_slam", "view.launch.py"]
