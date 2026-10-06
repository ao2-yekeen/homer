# ROS commands on the robot

- Before importing `rclpy` in a standalone Pi tool, call
  `ros_runtime.ensure_ros_environment()`. This loads and validates the same DDS
  middleware, domain, interfaces and peers as the deployed robot services.
- For ad hoc ROS CLI commands, source `/opt/ros/jazzy/setup.bash` and run them
  through `python3 ~/mobile-robot/pi/ros_runtime.py ros2 ...`. Sourcing the ROS
  distribution alone does not load the robot's network configuration.
- Do not use Fast DDS for robot commands: debugger evidence showed a native
  segfault during node creation on this Pi. Do not bypass environment errors.
- When uploading a standalone tool that imports `ros_runtime`, upload the module
  too. Check configuration and controller discovery without motion first.
- After changing ROS startup configuration, run `pi/tests/test_ros_runtime.py`
  and verify a node starts with the deployed configuration on the Pi. Report
  publication and physical movement separately; a ROS command has no shaft
  position feedback.
