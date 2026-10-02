"""Read-only MVP evidence capture. Never publishes drive commands."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time


def pose_xy_yaw(pose):
    q = pose.orientation
    values = [pose.position.x, pose.position.y, q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite odometry pose')
    norm = math.sqrt(sum(v*v for v in values[2:]))
    if abs(norm - 1.) > .01:
        raise ValueError('Odometry quaternion is not normalized')
    x, y, z, w = (v/norm for v in values[2:])
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return values[0], values[1], yaw


def displacement(first, last):
    angle = last[2] - first[2]
    return math.hypot(last[0]-first[0], last[1]-first[1]), abs(math.degrees(math.atan2(math.sin(angle), math.cos(angle))))


def tracking_result(poses, mode, seconds, operator_confirmed):
    if len(poses) < 2:
        return {'gate': False, 'reason': 'At least two odometry samples required'}
    changes = [displacement(poses[0], p) for p in poses]
    final = changes[-1]
    maximum = tuple(max(v[i] for v in changes) for i in (0, 1))
    measured = maximum if mode == 'stationary' else final
    limits = (.05, 3.) if mode == 'stationary' else (.20, 5.)
    gate = measured[0] < limits[0] and measured[1] < limits[1]
    if mode == 'stationary' and seconds < 60:
        gate = False
    if mode == 'return' and maximum[0] < .20:
        gate = False  # A stationary sensor cannot qualify the movement trial.
    return {'gate': gate if operator_confirmed else None,
            'numerical_pass': gate, 'operator_confirmed': operator_confirmed,
            'endpoint_translation_m': final[0], 'endpoint_yaw_deg': final[1],
            'maximum_translation_m': maximum[0], 'maximum_yaw_deg': maximum[1],
            'note': 'Operator must confirm stationary hardware or physical return to the marked start. Odometry alone cannot prove either.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--mode', choices=('live', 'stationary', 'return'), default='live')
    parser.add_argument('--seconds', type=float, default=10.)
    parser.add_argument('--operator-confirmed', action='store_true', help='Confirm hardware stayed still, or actually moved and returned to the marked pose')
    parser.add_argument('--reference-angle-deg', type=float, help='Known target bearing in the laser frame')
    parser.add_argument('--reference-distance-m', type=float, help='Measured distance from scan origin to a broad target surface')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('seconds must be finite and positive')
    if args.mode == 'stationary' and args.seconds < 60:
        parser.error('stationary trials require at least 60 seconds')
    reference = args.reference_distance_m is not None
    if reference != (args.reference_angle_deg is not None):
        parser.error('Supply both reference angle and distance')
    if reference and (not math.isfinite(args.reference_angle_deg) or not math.isfinite(args.reference_distance_m) or args.reference_distance_m <= 0):
        parser.error('Reference values must be finite, with positive distance')
    output = args.output.expanduser()
    if output.exists():
        parser.error('Evidence already exists; choose a new output path')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    from .core import age_seconds, freshness

    rclpy.init()
    node = Node('homer_mvp_check')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    scans, poses, odom_times, scan_times, ages, graph, transforms, errors = [], [], [], [], [], [], [], []
    tf_observations = []
    ranges, references, maps = [], [], []
    started = None

    def scan(msg):
        if started is None:
            return
        scan_times.append(time.monotonic())
        ages.append(age_seconds(node.get_clock().now().nanoseconds, msg.header.stamp))
        valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        scans.append(len(valid))
        ranges.append([msg.range_min, msg.range_max])
        if reference:
            selected = []
            target = math.radians(args.reference_angle_deg)
            for index, value in enumerate(msg.ranges):
                angle = msg.angle_min + index*msg.angle_increment - target
                if abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(2) and math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                    selected.append(value)
            if selected:
                references.append(statistics.median(selected))

    def odom(msg):
        if started is None:
            return
        odom_times.append(time.monotonic())
        ages.append(age_seconds(node.get_clock().now().nanoseconds, msg.header.stamp))
        try:
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                raise ValueError('Expected odom -> base_link odometry')
            poses.append(pose_xy_yaw(msg.pose.pose))
        except ValueError as error:
            errors.append(str(error))

    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(OccupancyGrid, '/map', lambda msg: maps.append({'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution, 'known_cells': sum(v >= 0 for v in msg.data)}), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def health():
        if started is None:
            return
        graph.append({topic: len(node.get_publishers_info_by_topic(topic)) for topic in ('/scan', '/odom', '/map')})
        good = True
        observations = {}
        for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('base_link', 'laser')):
            try:
                transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
                if parent != 'base_link':
                    # SLAM Toolbox postdates map TF by its 0.2 s transform timeout.
                    future_tolerance = .25 if parent == 'map' else .05
                    age = age_seconds(node.get_clock().now().nanoseconds, transform.header.stamp)
                    state = freshness(age, .5, future_tolerance)
                    observations[parent + ' -> ' + child] = {'age_seconds': age, 'state': state}
                    good &= state == 'fresh'
            except Exception as error:
                observations[parent + ' -> ' + child] = {'error': str(error)}
                good = False
        transforms.append(bool(good))
        tf_observations.append(observations)

    node.create_timer(1., health)
    try:
        warmup = time.monotonic() + 3
        while time.monotonic() < warmup:
            rclpy.spin_once(node, timeout_sec=.1)
        # Lifecycle activation and pose-graph loading can take longer than discovery.
        # Wait before starting the measured trial, without excusing dropouts inside it.
        def ready():
            if any(len(node.get_publishers_info_by_topic(topic)) != 1 for topic in ('/scan', '/odom', '/map')):
                return False
            try:
                for parent, child in (('map', 'odom'), ('odom', 'base_link'), ('base_link', 'laser')):
                    transform = buffer.lookup_transform(parent, child, rclpy.time.Time())
                    if parent != 'base_link' and freshness(age_seconds(node.get_clock().now().nanoseconds, transform.header.stamp), .5, .25 if parent == 'map' else .05) != 'fresh':
                        return False
            except Exception:
                return False
            return True
        deadline = time.monotonic() + 30
        while not ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        startup_ready = ready()
        started = time.monotonic()
        print('Recording read-only %s check for %.1f seconds' % (args.mode, args.seconds), flush=True)
        while time.monotonic() - started < args.seconds and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
        ended = time.monotonic()
        elapsed = ended - started
        def continuous(times):
            points = [started, *times, ended]
            return len(times) >= 2 and max(b-a for a, b in zip(points, points[1:])) < .5
        gates = {'startup_ready': startup_ready, 'valid_scans': bool(scans) and min(scans) >= 16,
                 'scan_continuity': continuous(scan_times), 'odom_continuity': continuous(odom_times),
                 'fresh_timestamps': bool(ages) and all(-.05 <= age <= .5 for age in ages),
                 'finite_normalized_poses': len(poses) >= 2 and not errors,
                 'single_publishers': bool(graph) and all(all(v == 1 for v in counts.values()) for counts in graph),
                 'tf_available_and_fresh': bool(transforms) and all(transforms),
                 'nonempty_map': bool(maps) and maps[-1]['width'] > 0 and maps[-1]['height'] > 0 and maps[-1]['known_cells'] > 0}
        result = {'mode': args.mode, 'elapsed_seconds': elapsed, 'scan_hz': len(scans)/elapsed,
                  'minimum_valid_returns': min(scans) if scans else None, 'scan_range_metadata': ranges[-1] if ranges else None,
                  'map': maps[-1] if maps else None, 'odom_samples': len(poses), 'publisher_counts': graph,
                  'errors': errors, 'gates': gates, 'tf_observations': tf_observations,
                  'note': 'This check does not certify physical mounting, visual alignment, map reload, or autonomous navigation.'}
        if args.mode != 'live':
            result['tracking'] = tracking_result(poses, args.mode, elapsed, args.operator_confirmed)
            gates['tracking'] = result['tracking']['gate']
        if reference:
            value = statistics.median(references) if references else None
            result['reference'] = {'angle_deg': args.reference_angle_deg, 'measured_distance_m': args.reference_distance_m, 'scan_distance_m': value}
            gates['reference_distance'] = value is not None and abs(value-args.reference_distance_m) <= .10
        result['accepted'] = all(value is True for value in gates.values())
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps({key: value for key, value in result.items() if key not in ('publisher_counts', 'tf_observations')}, indent=2, allow_nan=False))
        return 0 if result['accepted'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
