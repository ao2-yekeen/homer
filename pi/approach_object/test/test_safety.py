import sys
from pathlib import Path
sys.path[:0] = [str(Path(__file__).parents[1]), str(Path(__file__).parents[2]), str(Path(__file__).parents[3])]
import numpy as np
import pytest
from approach_object.core import Controller, Observation
from approach_object.geometry import estimate, Identity
from approach_object.configuration import load
from autonomous_lease import AutonomousLease

CONFIG = dict(qualified=True, settle_s=[.1, .1], search_s=1, braking_m=.02, uncertainty_m=.03, forward_command=-.12, turn_command=-.08)

def aligned():
    c = Controller(CONFIG)
    assert c.start('bottle', 1, True)[0]
    for i in range(3):
        c.observe(Observation(1.2+i*.05, i, 'one', 0, .8, 0), 1.2+i*.05)
    return c

def test_confirmation_and_fixed_view():
    c = aligned()
    assert c.state == 'ALIGN'
    assert c.tick(1.4, True, True) == (-.12, 0)
    assert c.state == 'APPROACH' and c.view == 0

@pytest.mark.parametrize('target', ['person', 'chair', 'book', 'remote', 'toothbrush', 'umbrella', 'banana', 'keyboard'])
def test_any_detector_category_can_be_selected_for_approach(target):
    c = Controller(CONFIG)
    assert c.start(target, 1, True) == (True, 'search started')
    assert c.target == target

@pytest.mark.parametrize('target', ['', '  '])
def test_empty_target_is_rejected(target):
    c = Controller(CONFIG)
    assert not c.start(target, 1, True)[0]

def test_duplicate_observations_cannot_confirm():
    c = Controller(CONFIG)
    c.start('cup', 1, True)
    for i in range(3):
        c.observe(Observation(1.2, 1, 'one', 0, .8, 0), 1.2)
    assert c.state == 'SEARCH'

def test_both_views_and_absent_target():
    c = Controller(CONFIG)
    c.start('cup', 1, True)
    assert c.tick(2.2, True, True) == (0, 0) and c.view == 1
    c.tick(3.4, True, True)
    assert c.state == 'FAILED'
    assert c.tick(4, True, True) == (0, 0)

@pytest.mark.parametrize('fault', ['stale', 'takeover', 'blocked', 'depth', 'identity', 'cancel'])
def test_faults_are_latched(fault):
    c = aligned()
    if fault == 'depth':
        c.observe(Observation(1.4, 5, 'one', 0, 0, 0, False, 'invalid depth'), 1.4)
    elif fault == 'identity':
        c.observe(Observation(1.4, 5, 'two', 0, .8, 0), 1.4)
    elif fault == 'cancel':
        c.finish('CANCELLED', 'cancelled')
    cmd = c.tick(2 if fault == 'stale' else 1.4, fault != 'takeover', fault != 'blocked')
    assert cmd == (0, 0)
    assert c.state in ('FAILED', 'CANCELLED')
    assert c.tick(3, True, True) == (0, 0)

@pytest.mark.parametrize('distance', [.1, .3, .349])
def test_stop_never_reverses(distance):
    c = aligned()
    c.observe(Observation(1.4, 4, 'one', 0, distance, 0), 1.4)
    assert c.tick(1.4, True, True) == (0, 0)
    assert c.state == 'STOPPED'

def test_turn_polarity_is_measured():
    c = aligned()
    c.observe(Observation(1.4, 4, 'one', .2, .8, 0), 1.4)
    assert c.tick(1.4, True, True) == (0, -.08)

def test_unqualified_cannot_start():
    c = load(str(Path(__file__).parents[1]/'config/calibration.yaml'))
    ok, reason = Controller(c).start('bottle', 1, True)
    assert not ok and 'LATENCY_BLOCKER' in reason
    assert not Controller(CONFIG).start('cup', 1, False)[0]

def test_lease_expiry_and_no_heartbeat_rearm():
    lease = AutonomousLease()
    lease.mode(True, 1)
    assert lease.accept(10, 10.1, 1, .1, 0)
    assert not lease.tick(1.14)
    assert lease.tick(1.15)
    lease.mode(True, 1.16)
    assert not lease.accept(10.2, 10.2, 1.16, .1, 0)
    lease.mode(False, 1.17)
    lease.mode(True, 1.18)
    assert lease.accept(10.3, 10.3, 1.18, .1, 0)

@pytest.mark.parametrize('stamp,linear', [(9, .1), (11, .1), (10, float('nan')), (10, 6)])
def test_lease_rejects_bad_commands(stamp, linear):
    lease = AutonomousLease()
    lease.mode(True, 1)
    assert not lease.accept(stamp, 10, 1, linear, 0)

def geometry_config():
    k = [[100,0,2],[0,100,2],[0,0,1]]
    # optical z->base x; optical x->base -y; optical y->base -z
    pose = [[0,0,1,0],[-1,0,0,0],[0,-1,0,.2],[0,0,0,1]]
    return dict(tof_shape=[5,5], rgb_shape=[5,5], tof_K=k, tof_D=[0]*5, rgb_K=k, rgb_D=[0]*5, depth_semantics='optical_z', depth_min_m=.05, depth_max_m=2, tof_to_rgb=np.eye(4).tolist(), views=[{'tof_to_base':pose}], floor_reject_m=.01, max_floor_fraction=.2, min_samples=4, max_object_depth_span_m=.15, front_edge_m=.1)

def test_mm_projection_front_edge_and_radial():
    c = geometry_config()
    distance, bearing = estimate(np.full((5,5), 400), np.ones((5,5)), c, 0)
    assert distance == pytest.approx(.3) and bearing == pytest.approx(0)
    c['depth_semantics'] = 'radial'
    distance, _ = estimate(np.full((5,5), 400), np.ones((5,5)), c, 0)
    assert .299 < distance < .3

@pytest.mark.parametrize('fault', ['floor', 'background', 'nan', 'mask', 'semantics'])
def test_reject_contaminated_depth(fault):
    c = geometry_config()
    depth = np.full((5,5), 400.)
    mask = np.ones((5,5))
    if fault == 'floor': c['views'][0]['tof_to_base'][2][3] = 0
    if fault == 'background': depth[:,3:] = 1000
    if fault == 'nan': depth[:] = np.nan
    if fault == 'mask': mask[:] = 0
    if fault == 'semantics': c['depth_semantics'] = 'unknown'
    with pytest.raises(ValueError): estimate(depth, mask, c, 0)

def test_target_identity_and_ambiguity():
    t = Identity()
    assert t.select([[0,0,10,10],[40,40,60,60]],100,100) == 1
    assert t.select([[41,40,61,60]],100,100) == 0
    with pytest.raises(ValueError): t.select([[41,40,61,60],[42,40,62,60]],100,100)

def test_future_perception_and_settling_frames_rejected():
    c = Controller(CONFIG)
    c.start('cup', 1, True)
    c.observe(Observation(1.05, 1, 'one', 0, .8, 0), 1.05)
    assert c.confirmations == 0
    c.observe(Observation(3, 2, 'one', 0, .8, 0), 1.4)
    assert c.state == 'FAILED'

def test_motion_enable_without_measurements_rejected(tmp_path):
    path = tmp_path/'invalid.yaml'
    path.write_text('enable_motion: true\n')
    with pytest.raises(ValueError, match='qualification'):
        load(path)
