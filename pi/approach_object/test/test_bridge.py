"""Exercise real bridge handlers with fake ROS plumbing and no serial device."""
import sys
import time
from pathlib import Path
from types import SimpleNamespace
sys.path[:0] = [str(Path(__file__).parents[1]), str(Path(__file__).parents[2]), str(Path(__file__).parents[3])]
import pytest
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, Int32
import esp32_serial_bridge as bridge

class Harness(bridge.Esp32SerialBridge):
    def declare_parameter(self, name, default):
        return SimpleNamespace(value=True if name == 'approach_mode' else default)
    def create_publisher(self, *args):
        return SimpleNamespace(publish=lambda m: None)
    def create_subscription(self, message_type, topic, callback, qos):
        self.captured_subscriptions.append((topic, callback))
    def create_timer(self, *args): pass
    def _send(self, kind, *fields): self.sent.append((kind, fields))
    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    def _open_next(self, now): pass

@pytest.fixture
def harness(monkeypatch):
    monkeypatch.setattr(bridge.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: 1.)
    h = Harness.__new__(Harness)
    h.captured_subscriptions, h.sent = [], []
    Harness.__init__(h)
    h.connected = True
    h._mode(Bool(data=True))
    h.sent.clear()
    return h

def command(stamp=10., linear=.1):
    m = TwistStamped()
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
    m.twist.linear.x = linear
    return m

def test_cmd_vel_cannot_bypass(harness):
    topics = dict(harness.captured_subscriptions)
    assert '/cmd_vel' not in topics
    assert '/approach_object/drive' in topics
    harness._leased_autonomous(command())
    assert harness.sent[-1] == ('A', ('0.1000', '0.0000'))

def test_bridge_stops_when_supervisor_dies(harness, monkeypatch):
    harness._leased_autonomous(command())
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: 1.26)
    harness.port = None
    harness.next_attempt = 2
    harness._tick()
    assert harness.sent[-1] == ('A', ('0.0000', '0.0000'))
    harness._leased_autonomous(command())
    assert harness.sent[-1] == ('A', ('0.0000', '0.0000'))

def test_override_and_teleop_stop_accepting_approach(harness):
    harness._override(Int32(data=2))
    harness.sent.clear()
    harness._leased_autonomous(command())
    assert not harness.sent
    harness._override(Int32(data=-1))
    harness._mode(Bool(data=False))
    harness.sent.clear()
    harness._leased_autonomous(command())
    assert not harness.sent
    teleop = Twist()
    teleop.linear.x = .1
    harness._teleop(teleop)
    assert harness.sent[-1][0] == 'T'

def test_old_queued_command_rejected(harness):
    harness._leased_autonomous(command(9.7))
    assert not harness.sent
