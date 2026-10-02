from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from .core import load_geometry, map_prefix
from .model import model_geometry, display_description


_leases = []

def launch(mode):
    share = Path(get_package_share_directory('homer_slam'))
    args = [DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('geometry', default_value='urdf'),
            DeclareLaunchArgument('urdf', default_value=str(share / 'urdf/home_mobile_manipulator_v6_real_base_layout.urdf'))]
    if mode == 'view':
        args += [DeclareLaunchArgument('show_model', default_value='true')]
    if mode != 'view':
        args += [DeclareLaunchArgument('min_range'), DeclareLaunchArgument('max_range')]
    if mode == 'localization':
        args += [DeclareLaunchArgument('map_file')]
    return LaunchDescription(args + [OpaqueFunction(function=lambda context: assemble(context, mode, share))])


def assemble(context, mode, share):
    geometry_path = LaunchConfiguration('geometry').perform(context)
    urdf = LaunchConfiguration('urdf').perform(context)
    geometry = model_geometry(urdf) if geometry_path == 'urdf' else load_geometry(geometry_path)
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    common = {'use_sim_time': sim}
    if mode == 'view':
        result = [Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'rviz/topdown.rviz')], parameters=[common]),
                  Node(package='homer_slam', executable='viewer_status', parameters=[common])]
        if LaunchConfiguration('show_model').perform(context).lower() == 'true':
            center = model_geometry(urdf)['model_center']
            result += [Node(package='robot_state_publisher', executable='robot_state_publisher', name='display_model', parameters=[common, {'robot_description': display_description(urdf), 'frame_prefix': 'display/'}]),
                       Node(package='tf2_ros', executable='static_transform_publisher', parameters=[common], arguments=['--x', str(-center[0]), '--y', str(-center[1]), '--frame-id', 'base_link', '--child-frame-id', 'display/base_footprint'])]
        return result
    low = float(LaunchConfiguration('min_range').perform(context))
    high = float(LaunchConfiguration('max_range').perform(context))
    import math
    if not (math.isfinite(low) and math.isfinite(high) and 0 <= low < high):
        raise ValueError('Supply finite scan metadata: 0 <= min_range < max_range')
    import fcntl, os
    lease = open(Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / ('homer-slam-%s.lock' % os.getuid()), 'w')
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lease.close()
        raise ValueError('Another Homer odometry/mapping/localization launch is running on this host')
    _leases.append(lease)
    # Discover external ownership before starting local estimator processes.
    import rclpy, time
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node as ProbeNode
    probe_context = Context()
    rclpy.init(args=[], context=probe_context)
    probe = ProbeNode('homer_bringup_preflight', context=probe_context)
    probe_executor = SingleThreadedExecutor(context=probe_context)
    probe_executor.add_node(probe)
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            probe_executor.spin_once(timeout_sec=0.1)
        if probe.get_publishers_info_by_topic('/odom') or any(name in ('rf2o_laser_odometry', 'slam_toolbox') for name, _ in probe.get_node_names_and_namespaces()):
            raise ValueError('Existing odometry/SLAM publisher found: stop it before starting another session')
    finally:
        probe_executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown(context=probe_context)
    laser = geometry['laser']
    static = Node(package='tf2_ros', executable='static_transform_publisher',
                  arguments=[item for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw') for item in ('--'+k, str(laser[k]))] +
                  ['--frame-id', 'base_link', '--child-frame-id', 'laser'], parameters=[common])
    rf2o = Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
                output='screen', parameters=[common, {
                    'laser_scan_topic': '/scan', 'odom_topic': '/odom', 'publish_tf': True,
                    'base_frame_id': 'base_link', 'odom_frame_id': 'odom',
                    'init_pose_from_topic': '', 'freq': 20.0}])
    result = [static, rf2o,
              Node(package='homer_slam', executable='robot_markers', parameters=[common, {'geometry': geometry_path, 'urdf': urdf}]),
              Node(package='homer_slam', executable='health_monitor', parameters=[common, {'mapping': mode != 'odometry'}])]
    if mode != 'odometry':
        config = yaml.safe_load((share/'config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
        config.update(common, min_laser_range=low, max_laser_range=high, mode='localization' if mode == 'localization' else 'mapping')
        if mode == 'localization':
            config['map_file_name'] = str(map_prefix(LaunchConfiguration('map_file').perform(context)))
            config['map_start_at_dock'] = True
        slam = LifecycleNode(package='slam_toolbox', executable='localization_slam_toolbox_node' if mode == 'localization' else 'async_slam_toolbox_node',
                    name='slam_toolbox', namespace='', output='screen', parameters=[config])
        activate = RegisterEventHandler(OnStateTransition(target_lifecycle_node=slam,
            start_state='configuring', goal_state='inactive', entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_ACTIVATE))]))
        configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE))
        # Only one launch/session on a host; remote ownership is checked by health_monitor.
        result += [activate, slam, configure]
    guarded = [action for action in result if isinstance(action, Node)]
    def stop_session(event, launch_context):
        if launch_context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='A processing component exited; restart explicitly'))]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=action, on_exit=stop_session)) for action in guarded]
    return handlers + result
