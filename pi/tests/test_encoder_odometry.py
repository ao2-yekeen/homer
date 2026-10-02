"""Run with ROS sourced; skipped on machines without ROS dependencies."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    import rclpy
    from rclpy.parameter import Parameter
    from esp32_serial_bridge import Esp32SerialBridge
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 runtime not installed')
class EncoderOdometryTests(unittest.TestCase):
    def tearDown(self):
        if rclpy.ok(): rclpy.shutdown()

    def test_default_has_no_encoder_publisher_and_ignores_frames(self):
        rclpy.init()
        node = Esp32SerialBridge()
        try:
            self.assertIsNone(node.odom_pub)
            node.connected = True
            node._handle('O', ['123','1','2','0','0','0'])
            self.assertEqual(node.decoder.errors,0)
            self.assertNotIn('/odom',[name for name,_ in node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())])
        finally: node.destroy_node()

    def test_explicit_enable_preserves_legacy_publishing(self):
        rclpy.init(args=['--ros-args','-p','publish_encoder_odometry:=true'])
        node = Esp32SerialBridge()
        try:
            self.assertIsNotNone(node.odom_pub)
            node.connected = True
            node._handle('O',['123','1','2','0','0','0'])
            self.assertEqual(node.decoder.errors,0)
            node._handle('O',['123','nan','2','0','0','0'])
            self.assertEqual(node.decoder.errors,1)
        finally: node.destroy_node()
