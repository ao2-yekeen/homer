"""Real service and lidar handlers with in-memory messages; no publishers/devices."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1]))
from types import SimpleNamespace as S
import numpy as np
import pytest
pytest.importorskip('rclpy')
pytest.importorskip('approach_object_interfaces.srv')
from approach_object.supervisor import Supervisor
from approach_object.core import Controller
import approach_object.supervisor as module

@pytest.fixture
def supervisor(monkeypatch):
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    n = Supervisor.__new__(Supervisor)
    n.c = dict(qualified=True, settle_s=[.1,.1], search_s=1, braking_m=.01, uncertainty_m=.02, forward_command=-.1, turn_command=.1, lidar_to_base=np.eye(4).tolist(), lidar_front_limit_m=.6, lidar_half_width_m=.25, turn_clearance_radius_m=.35)
    n.controller = Controller(n.c)
    n.mode = n.link = n.bridge_authority = True
    n.override = False
    n.mode_at = n.status_at = n.authority_at = 10.
    n.reported_mode = 1
    n.session = 0
    n.scan_at = 10.
    n.scan = S(header=S(stamp=S(sec=10,nanosec=0)), ranges=[2.]*360, angle_min=-np.pi, angle_increment=2*np.pi/360, range_min=.03, range_max=4.)
    n.sent = []
    n.publish_drive = lambda x,z: n.sent.append((x,z))
    n.get_clock = lambda: S(now=lambda: S(nanoseconds=10_000_000_000))
    return n

def test_start_does_not_select_autonomous_mode(supervisor):
    supervisor.mode = False
    res = supervisor.start(S(target='bottle'), S())
    assert not res.success and supervisor.mode is False
    supervisor.mode = True
    res = supervisor.start(S(target='bottle'), S())
    assert res.success and supervisor.controller.state == 'SEARCH'
    assert not supervisor.sent

def test_cancellation_immediate_zero(supervisor):
    supervisor.start(S(target='bottle'), S())
    res = supervisor.cancel(S(), S())
    assert res.success and supervisor.sent == [(0,0)]
    assert supervisor.controller.state == 'CANCELLED'

@pytest.mark.parametrize('fault',['clear','obstacle','stale','partial','unknown','turning_obstacle'])
def test_lidar_corridor_and_rotation(supervisor, fault):
    n = supervisor
    if fault == 'obstacle': n.scan.ranges[180] = .4
    if fault == 'stale': n.scan.header.stamp.sec = 9
    if fault == 'partial': n.scan.ranges = n.scan.ranges[:10]
    if fault == 'unknown': n.scan.ranges[180] = float('nan')
    if fault == 'turning_obstacle':
        n.controller.state = 'ALIGN'
        n.scan.ranges[90] = .2
    assert n.path_clear() == (fault == 'clear')
