"""Operator tools: no drive publishers and no silent map replacement."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time


def call(node, service_type, name, request, timeout):
    import rclpy
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('Service unavailable: ' + name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('Service failed or timed out: ' + name)
        return future.result()
    finally:
        node.destroy_client(client)


def save_main():
    parser = argparse.ArgumentParser(description='Save occupancy map and pose graph on the SLAM host')
    parser.add_argument('directory', type=Path)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    destination = args.directory.expanduser().resolve()
    if destination.exists():
        parser.error('Destination already exists; choose a new directory')
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.homer-map-', dir=destination.parent))
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SaveMap, SerializePoseGraph
    from ament_index_python.packages import get_package_share_directory
    rclpy.init(); node = Node('homer_save_map')
    try:
        # A successful service/file export can still contain a zero-width map.
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, DurabilityPolicy
        received = []
        subscription = node.create_subscription(OccupancyGrid, '/map', received.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + args.timeout
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(subscription)
        if not received or received[-1].info.width <= 0 or received[-1].info.height <= 0 or not received[-1].data or not any(v >= 0 for v in received[-1].data):
            raise RuntimeError('Occupancy map is missing, empty, or entirely unknown; verify valid LiDAR ranges before saving')
        request = SaveMap.Request(); request.name.data = str(staging/'map')
        response = call(node, SaveMap, '/slam_toolbox/save_map', request, args.timeout)
        if response.result != 0:
            raise RuntimeError('Occupancy export failed, result=' + str(response.result))
        request = SerializePoseGraph.Request(); request.filename = str(staging/'map')
        response = call(node, SerializePoseGraph, '/slam_toolbox/serialize_map', request, args.timeout)
        if response.result != 0:
            raise RuntimeError('Pose graph export failed, result=' + str(response.result))
        for suffix in ('.yaml', '.posegraph', '.data'):
            if not (staging/('map'+suffix)).is_file():
                raise RuntimeError('Expected export missing; run save_map on the same host as SLAM: map'+suffix)
        import yaml
        metadata = yaml.safe_load((staging/'map.yaml').read_text())
        image = Path(metadata['image'])
        image = (staging/image).resolve()
        if image.parent != staging or not image.is_file():
            raise RuntimeError('Occupancy image missing from the exported directory')
        metadata['image'] = image.name
        (staging/'map.yaml').write_text(yaml.safe_dump(metadata))
        share = Path(get_package_share_directory('homer_slam'))
        shutil.copytree(share/'config', staging/'config')
        shutil.copytree(share/'urdf', staging/'urdf')
        shutil.copytree(share/'patches', staging/'patches')
        shutil.copy2(share/'dependencies.repos', staging/'dependencies.repos')
        parameters = subprocess.run(['ros2','param','dump','--no-daemon','--spin-time','2','--timeout','10','/slam_toolbox'], capture_output=True, text=True, check=True)
        (staging/'slam_parameters.yaml').write_text(parameters.stdout)
        # Preserve the actual launched geometry/URDF parameters, including user overrides.
        for name in ('/homer_robot_markers', '/rf2o_laser_odometry'):
            params = subprocess.run(['ros2','param','dump','--no-daemon','--spin-time','2','--timeout','10',name], capture_output=True, text=True, check=True)
            (staging/(name.strip('/')+'_parameters.yaml')).write_text(params.stdout)
            if name == '/homer_robot_markers':
                actual = yaml.safe_load(params.stdout)[name]['ros__parameters']
                urdf = Path(actual['urdf'])
                shutil.copy2(urdf, staging/'urdf'/'active-model.urdf')
                if actual['geometry'] != 'urdf':
                    shutil.copy2(actual['geometry'], staging/'config'/'active-geometry.yaml')
        versions = subprocess.run(['dpkg-query','-W','ros-jazzy-slam-toolbox','ros-jazzy-rmw-cyclonedds-cpp'], capture_output=True, text=True)
        (staging/'versions.txt').write_text(versions.stdout)
        (staging/'session.json').write_text(json.dumps({'saved_unix_time': time.time(), 'hostname': os.uname().nodename}, indent=2))
        # mkdir reserves the destination atomically; never overwrite another operator's save.
        destination.mkdir()
        for item in staging.iterdir(): shutil.move(str(item), destination/item.name)
        staging.rmdir()
        print('Saved occupancy map and pose graph:', destination)
    except Exception:
        print('Save failed. Partial exports retained for inspection:', staging)
        raise
    finally:
        node.destroy_node(); rclpy.shutdown()


def record_main():
    parser = argparse.ArgumentParser(description='Record on the laptop; do not start live processing during replay')
    parser.add_argument('directory')
    parser.add_argument('--camera-topics', nargs='+', default=[])
    args = parser.parse_args()
    topics = ['/scan','/odom','/map','/tf','/tf_static','/homer/robot_markers','/homer/slam_status','/diagnostics','/drive/link','/drive/status']
    subprocess.run(['ros2','bag','record','-o',args.directory,*topics,*args.camera_topics], check=True)
