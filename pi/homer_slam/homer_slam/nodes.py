import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from .core import load_geometry, age_seconds, freshness
from .model import model_geometry


class RobotMarkers(Node):
    def __init__(self):
        super().__init__('homer_robot_markers')
        geometry = self.declare_parameter('geometry', 'urdf').value
        urdf = self.declare_parameter('urdf', '').value
        self.geometry = model_geometry(urdf) if geometry == 'urdf' else load_geometry(geometry)
        self.pub = self.create_publisher(MarkerArray, '/homer/robot_markers', 5)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.2, self.tick)

    def tick(self):
        try:
            transform = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            if freshness(age_seconds(self.get_clock().now().nanoseconds, transform.header.stamp), 0.5) != 'fresh':
                return  # finite lifetime removes stale robot rather than freezing it indefinitely
        except Exception:
            return
        markers = []
        for number, kind in ((0, Marker.ARROW), (1, Marker.LINE_STRIP)):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = transform.header.stamp
            marker.ns = 'homer'; marker.id = number; marker.type = kind
            marker.action = Marker.ADD; marker.pose.orientation.w = 1.
            marker.lifetime = Duration(seconds=0.6).to_msg()
            marker.color.r = 0.1; marker.color.g = 0.65; marker.color.b = 1.; marker.color.a = 1.
            if kind == Marker.ARROW:
                marker.pose.position.z = 0.04
                marker.scale.x = 0.22; marker.scale.y = 0.04; marker.scale.z = 0.04
            else:
                marker.scale.x = 0.012
                points = self.geometry['footprint']
                marker.points = [Point(x=float(x), y=float(y), z=0.03) for x,y in points + points[:1]]
            markers.append(marker)
        self.pub.publish(MarkerArray(markers=markers))


class ViewerStatus(Node):
    """Local warning survives loss of the robot's diagnostics connection."""
    def __init__(self):
        super().__init__('homer_view_status')
        self.odom = None
        self.valid_ranges = None
        self.buffer = Buffer(node=self); self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, '/homer/view_status', 5)
        self.create_timer(1., self.tick)

    def on_odom(self, msg):
        self.odom = msg.header.stamp

    def on_scan(self, msg):
        self.valid_ranges = sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        state = freshness(age_seconds(now, self.odom) if self.odom else None, .5)
        frame = 'map'
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            frame = 'base_link'
            if state == 'fresh' and freshness(age_seconds(now, transform.header.stamp), .5, .25) != 'fresh':
                state = 'stale transform'
        except Exception:
            if state == 'fresh': state = 'missing transform'
        lidar_error = self.valid_ranges is not None and self.valid_ranges < 16
        if lidar_error: state = 'insufficient valid ranges'
        marker = Marker(); marker.header.frame_id = frame
        marker.ns = 'homer_view'; marker.id = 0; marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.DELETE if state == 'fresh' else Marker.ADD
        marker.pose.orientation.w = 1.; marker.pose.position.y = .5; marker.pose.position.z = .35
        marker.scale.z = .08; marker.color.r = 1.; marker.color.g = .3; marker.color.a = 1.
        marker.lifetime = Duration(seconds=2.5).to_msg()
        marker.text = ('LIDAR: ' if lidar_error else 'ODOMETRY: ') + state
        self.pub.publish(MarkerArray(markers=[marker]))


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('homer_slam_health')
        self.mapping = self.declare_parameter('mapping', True).value
        self.scan = self.odom = None
        self.invalid_odom = False
        self.valid_ranges = 0
        self.tf_owners = {}
        self.odom_authorities = None
        self.restarted = False
        self.first = time.monotonic()
        self.buffer = Buffer(node=self); self.listener = TransformListener(self.buffer, self)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.status_pub = self.create_publisher(MarkerArray, '/homer/slam_status', 5)
        self.create_timer(1., self.tick)

    def on_scan(self, msg):
        self.scan = msg.header.stamp
        self.valid_ranges = sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)

    def on_odom(self, msg):
        if self.odom and (msg.header.stamp.sec, msg.header.stamp.nanosec) < (self.odom.sec, self.odom.nanosec):
            self.restarted = True
        self.odom = msg.header.stamp
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.invalid_odom = msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link' or not all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01

    def on_tf(self, msg, info):
        # Earlier Jazzy rclpy builds expose only timestamps in a dictionary.
        raw_gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
        if raw_gid is None:
            return
        gid = bytes(raw_gid)
        for transform in msg.transforms:
            child = transform.child_frame_id
            if child in ('base_link', 'odom'):
                previous = self.tf_owners.get(child)
                if previous is not None and previous != gid:
                    self.restarted = True
                self.tf_owners[child] = gid

    def tick(self):
        now = self.get_clock().now().nanoseconds
        statuses = []
        for name, stamp in (('scan', self.scan), ('odometry', self.odom)):
            age = age_seconds(now, stamp) if stamp else None
            state = freshness(age, 0.5)
            statuses.append(DiagnosticStatus(name='homer/'+name, level=DiagnosticStatus.OK if state == 'fresh' else DiagnosticStatus.ERROR,
                message=state + '; freshness does not certify tracking accuracy', hardware_id='homer',
                values=[KeyValue(key='age_seconds', value=str(age))]))
        problems = []
        if self.scan and self.valid_ranges < 16: problems.append('insufficient valid laser ranges: ' + str(self.valid_ranges))
        if self.invalid_odom: problems.append('invalid odometry frame or nonfinite pose')
        if self.restarted: problems.append('estimator authority/time changed: restart the complete session explicitly')
        publishers = self.get_publishers_info_by_topic('/odom')
        authorities = {bytes(p.endpoint_gid) for p in publishers}
        if authorities:
            if self.odom_authorities is not None and authorities != self.odom_authorities:
                self.restarted = True
            self.odom_authorities = authorities
        if len(publishers) > 1: problems.append('multiple /odom publishers')
        nodes = [name for name,_ in self.get_node_names_and_namespaces()]
        if nodes.count('rf2o_laser_odometry') > 1 or nodes.count('slam_toolbox') > 1:
            problems.append('duplicate estimator nodes; stop the extra launch')
        pairs = [('base_link','laser'),('odom','base_link')]
        if self.mapping: pairs += [('map','odom')]
        for parent,child in pairs:
            try:
                tf = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                # SLAM Toolbox deliberately postdates map TF by transform_timeout (0.2 s).
                future_tolerance = 0.25 if parent == 'map' else 0.05
                if child != 'laser' and freshness(age_seconds(now, tf.header.stamp), 0.5, future_tolerance) != 'fresh':
                    problems.append('stale '+parent+' -> '+child)
            except Exception:
                problems.append('missing '+parent+' -> '+child)
        statuses.append(DiagnosticStatus(name='homer/transforms', level=DiagnosticStatus.ERROR if problems else DiagnosticStatus.OK,
            message='; '.join(problems) if problems else 'frames available; accuracy requires physical validation', hardware_id='homer'))
        self.pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=statuses))
        marker = Marker()
        marker.header.frame_id = 'map' if self.mapping else 'odom'
        marker.ns = 'homer_health'; marker.id = 0; marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD; marker.pose.orientation.w = 1.
        marker.pose.position.z = 0.3; marker.scale.z = 0.15
        marker.color.r = 1.; marker.color.g = 0.5; marker.color.a = 1.
        marker.lifetime = Duration(seconds=2.5).to_msg()
        errors = [s.name + ': ' + s.message for s in statuses if s.level != DiagnosticStatus.OK]
        marker.text = '\n'.join(errors) if errors else 'Sensors fresh; tracking accuracy unverified'
        self.status_pub.publish(MarkerArray(markers=[marker]))


def run(cls):
    rclpy.init()
    node = None
    try:
        node = cls(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def markers_main(): run(RobotMarkers)
def health_main(): run(HealthMonitor)
def viewer_main(): run(ViewerStatus)
