"""20 Hz Pi-local command owner; perception never writes drive commands."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger
from approach_object_interfaces.srv import StartApproach
from .configuration import load
from .core import Controller, Observation, ACTIVE
from .geometry import transform

class Supervisor(Node):
    def __init__(self):
        super().__init__('approach_object_supervisor')
        self.c = load(self.declare_parameter('calibration', '').value)
        self.controller = Controller(self.c)
        self.mode = self.link = False
        self.override = False
        self.bridge_authority = False
        self.authority_at = 0.0
        self.mode_at = self.status_at = self.scan_at = 0.0
        self.reported_mode = -1
        self.scan = None
        self.tilt = None
        self.session = 0
        self.last_perception_stamp = float("-inf")
        self.drive = self.create_publisher(TwistStamped, '/approach_object/drive', 1)
        self.neck = self.create_publisher(Int32, '/teleop/neck_angle', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/approach_object/status', qos)
        self.request = self.create_publisher(String, '/approach_object/request', qos)
        self.create_subscription(Bool, '/drive/approach_authority', self.authority_callback, 1)
        self.create_subscription(Bool, '/set_autonomous', self.mode_callback, 1)
        self.create_subscription(Int32, '/mode_override', self.override_callback, 1)
        self.create_subscription(Bool, '/drive/link', self.link_callback, qos)
        self.create_subscription(Twist, '/drive/status', self.drive_status, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/approach_object/observations', self.observe, 1)
        self.create_service(StartApproach, '/approach_object/start', self.start)
        self.create_service(Trigger, '/approach_object/cancel', self.cancel)
        self.create_timer(.05, self.tick)

    def authority_callback(self, m):
        self.bridge_authority, self.authority_at = m.data, time.monotonic()

    def mode_callback(self, m):
        self.mode, self.mode_at = m.data, time.monotonic()

    def override_callback(self, m):
        self.override = m.data >= 0

    def link_callback(self, m):
        self.link = m.data

    def drive_status(self, m):
        self.reported_mode = m.angular.x
        self.status_at = time.monotonic()

    def scan_callback(self, m):
        self.scan, self.scan_at = m, time.monotonic()

    def authorized(self):
        now = time.monotonic()
        return self.bridge_authority and now-self.authority_at < .25 and self.mode and self.link and not self.override and self.reported_mode == 1 and now-self.mode_at < 1.5 and now-self.status_at < .5

    def start(self, req, res):
        res.success, res.message = self.controller.start(req.target, time.monotonic(), self.authorized())
        if res.success:
            self.session += 1
            self.tilt = None
            self.last_perception_stamp = float("-inf")
        return res

    def cancel(self, req, res):
        self.controller.finish('CANCELLED', 'operator cancellation')
        self.publish_drive(0, 0)
        res.success, res.message = True, 'cancelled; explicit start required'
        return res

    def observe(self, msg):
        try:
            d = json.loads(msg.data)
            if d['session'] != self.session or d['target'] != self.controller.target:
                return
            if d['valid'] and d['stamp'] <= self.last_perception_stamp:
                return
            if d['valid']:
                self.last_perception_stamp = d['stamp']
            age = self.get_clock().now().nanoseconds * 1e-9 - d['stamp']
            # Include measured capture buffering in the effective sample age.
            delay = self.c.get('capture_delay_bound_s') or 0
            obs = Observation(time.monotonic() - age - delay, d['sequence'], d['identity'], d['bearing'], d['clearance'], d['view'], d['valid'], d['reason'])
            self.controller.observe(obs, time.monotonic())
        except (KeyError, TypeError, ValueError):
            if self.controller.state in ACTIVE:
                self.controller.finish('FAILED', 'malformed perception')

    def path_clear(self):
        s = self.scan
        if not self.c['qualified'] or s is None or time.monotonic()-self.scan_at > .5:
            return False
        age = self.get_clock().now().nanoseconds * 1e-9 - (s.header.stamp.sec+s.header.stamp.nanosec*1e-9)
        if not 0 <= age <= .5 or len(s.ranges) == 0 or s.angle_increment == 0:
            return False
        if not np.isfinite([s.angle_min, s.angle_increment, s.range_min, s.range_max]).all() or s.range_min < 0 or s.range_max <= s.range_min:
            return False
        if abs(s.angle_increment) > .03 or (len(s.ranges)-1)*abs(s.angle_increment) < 2*math.pi-.05:
            return False # first version requires measured full-circle lidar coverage
        ranges = np.asarray(s.ranges)
        angles = s.angle_min + np.arange(len(ranges)) * s.angle_increment
        unit = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        rays_base = unit @ np.asarray(self.c['lidar_to_base'])[:3,:3].T
        front = rays_base[:,0] > 0
        turning = self.controller.state == 'ALIGN'
        if turning:
            front = np.ones(len(angles), dtype=bool)
        # Unknown or out-of-range beams in the forward half-plane block movement.
        valid = np.isfinite(ranges) & (ranges >= s.range_min) & (ranges <= s.range_max)
        if not front.any() or not valid[front].all():
            return False
        points = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid]), np.zeros(valid.sum())))
        base = transform(points, self.c['lidar_to_base'])
        if turning and np.any(np.linalg.norm(base[:,:2], axis=1) < self.c['turn_clearance_radius_m']):
            return False
        return not np.any((base[:, 0] > 0) & (base[:, 0] < self.c['lidar_front_limit_m']) & (np.abs(base[:, 1]) < self.c['lidar_half_width_m']))

    def publish_drive(self, linear, angular):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = float(linear), float(angular)
        self.drive.publish(m)

    def tick(self):
        linear, angular = self.controller.tick(time.monotonic(), self.authorized(), self.path_clear())
        self.publish_drive(linear, angular)
        ctl = self.controller
        if ctl.state in ACTIVE and self.tilt != ctl.view:
            self.neck.publish(Int32(data=self.c['views'][ctl.view]['servo_deg']))
            self.tilt = ctl.view
        self.request.publish(String(data=json.dumps({'session': self.session, 'target': ctl.target, 'view': ctl.view, 'active': ctl.state in ACTIVE, 'settled_after': self.get_clock().now().nanoseconds * 1e-9 + ctl.view_started + self.c['settle_s'][ctl.view] - time.monotonic()})))
        self.status.publish(String(data=json.dumps({'state': ctl.state, 'target': ctl.target, 'clearance_m': ctl.clearance, 'reason': ctl.reason or (self.c.get('movement_blocker', '') if not self.c['qualified'] else ''), 'qualification': 'REQUIRES_HARDWARE_TEST' if not self.c['qualified'] else 'MEASURED_CONFIG', 'view': ctl.view})))


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    finally:
        node.publish_drive(0, 0)
        node.destroy_node()
        rclpy.shutdown()
