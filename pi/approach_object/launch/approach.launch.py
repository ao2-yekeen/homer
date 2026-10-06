from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    default = str(Path(get_package_share_directory('approach_object')) / 'config/calibration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('calibration', default_value=default),
        DeclareLaunchArgument('model', description='Path to checksum-pinned NCNN model directory'),
        DeclareLaunchArgument('recognition_target', default_value='', description='Optional stationary recognition preview class'),
        DeclareLaunchArgument('surface_distance', default_value='false', description='Publish median ToF range in recognition preview'),
        Node(package='approach_object', executable='supervisor', parameters=[{'calibration': LaunchConfiguration('calibration')}]),
        Node(package='approach_object', executable='detector', parameters=[{'calibration': LaunchConfiguration('calibration'), 'model': LaunchConfiguration('model'), 'recognition_target': LaunchConfiguration('recognition_target'), 'surface_distance_enabled': LaunchConfiguration('surface_distance')}]),
    ])
