"""Named-object contact geometry and rejection cases, independent of model scores."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import torch
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import named_object_distance_test as diagnostic


@pytest.fixture
def scene():
    shape = (100, 100)
    mask = np.zeros(shape, bool)
    mask[10:90, 20:80] = True
    k = [[100., 0., 50.], [0., 100., 50.], [0., 0., 1.]]
    pose = [[0., 0., 1., 0.], [-1., 0., 0., 0.], [0., -1., 0., .355], [0., 0., 0., 1.]]
    calibration = dict(rgb_frame='rgb_camera', tof_frame='tof_camera', rgb_shape=shape,
        tof_shape=shape, rgb_K=k, tof_K=k, rgb_D=[0.] * 5, tof_D=[0.] * 5,
        tof_to_rgb=np.eye(4).tolist(), depth_semantics='optical_z', depth_min_m=.05,
        depth_max_m=4., front_edge_m=.2, views=[dict(servo_deg=100., tof_to_base=pose)],
        floor_reject_m=.04, max_object_depth_span_m=.15,
        **{name: True for name in diagnostic.FLAGS})
    metadata = dict(rgb_frame='rgb_camera', tof_frame='tof_camera', rgb_stamp=1.,
        depth_stamp=1.02, receipt_stamp=1.1, depth_units='mm', servo_command_deg=100.)
    return mask, np.full(shape, 1000.), calibration, metadata


def analyse(scene, **kwargs):
    mask, depth, calibration, metadata = scene
    return diagnostic.analyse_mask(mask, depth, kwargs.pop('confidence', None),
        calibration, metadata, kwargs.pop('servo_deg', 100.), **kwargs)


def test_metric_contact_and_front_subtraction(scene):
    result, native, contact = analyse(scene)
    assert result['clearance_m'] == pytest.approx(.8)
    assert result['contact_samples'] >= 12
    assert np.all((contact == 0) | (native > 0))
    assert np.count_nonzero(native) < np.count_nonzero(scene[0])


def test_tilt_uses_pixel_ray_and_pose_translation(scene):
    angle = np.deg2rad(30)
    pose = scene[2]['views'][0]['tof_to_base']
    pose[:3] = [[0., -np.sin(angle), np.cos(angle), .12],
                [-1., 0., 0., 0.],
                [0., -np.cos(angle), -np.sin(angle), np.sin(angle) + np.cos(angle) * .355]]
    result, _, contact = analyse(scene)
    ys = np.where(contact)[0]
    expected = .12 + np.cos(angle) - np.sin(angle) * np.median((ys - 50.) / 100.) - .2
    assert result['clearance_m'] == pytest.approx(expected)


@pytest.mark.parametrize('lateral', [0., .12])
def test_floor_contact_transform_agrees_with_pythagoras(lateral):
    angle = np.deg2rad(36.)
    height, camera_forward, front_edge = .6, .1, .2
    pose = np.array([[0., -np.sin(angle), np.cos(angle), camera_forward],
                     [-1., 0., 0., 0.],
                     [0., -np.cos(angle), -np.sin(angle), height], [0., 0., 0., 1.]])
    floor_contact = np.array([[.58, lateral, 0.]])
    camera_point = (floor_contact - pose[:3, 3]) @ pose[:3, :3]
    ray_distance = np.linalg.norm(camera_point[0])
    expected = np.sqrt(ray_distance**2 - height**2 - lateral**2) + camera_forward - front_edge
    actual = diagnostic.transform(camera_point, pose)[0, 0] - front_edge
    assert actual == pytest.approx(.38)
    assert actual == pytest.approx(expected)


def test_box_top_ray_uses_height_difference_not_floor_height():
    lens_height, box_height, forward = .6, .35, .48
    bottom_ray = np.hypot(forward, lens_height)
    top_ray = np.hypot(forward, lens_height - box_height)
    assert np.sqrt(bottom_ray**2 - lens_height**2) == pytest.approx(forward)
    assert np.sqrt(top_ray**2 - (lens_height - box_height)**2) == pytest.approx(forward)
    assert top_ray < lens_height  # using floor height on the top ray is impossible


def test_native_indices_survive_invalid_depth_and_projection(scene):
    _, depth, calibration, _ = scene
    depth[:] = 0
    depth[60, 55] = 1000
    depth[20, 30] = np.nan
    calibration['tof_to_rgb'][0][3] = .1
    indices, pixels, points = diagnostic.project_native(depth, calibration, depth.shape)
    assert indices.tolist() == [60 * 100 + 55]
    assert pixels.tolist() == [[65, 60]]
    assert points[0].tolist() == pytest.approx([.05, .1, 1.])
    calibration['depth_semantics'] = 'radial'
    _, _, radial = diagnostic.project_native(depth, calibration, depth.shape)
    assert np.linalg.norm(radial[0]) == pytest.approx(1.)
    assert radial[0, 2] < points[0, 2]


def test_projection_keeps_frontmost_return_per_rgb_pixel(scene):
    _, depth, calibration, _ = scene
    depth[:] = 0
    depth[50, 50] = 500
    depth[50, 51] = 1000
    calibration['tof_K'] = [[100000., 0., 50.], [0., 100000., 50.], [0., 0., 1.]]
    calibration['tof_to_rgb'] = np.eye(4).tolist()
    calibration['rgb_K'] = [[100000., 0., 50.], [0., 100000., 50.], [0., 0., 1.]]
    calibration['rgb_D'] = [0.] * 5
    calibration['tof_D'] = [0.] * 5
    indices, pixels, points = diagnostic.project_native(depth, calibration, depth.shape)
    collisions = np.flatnonzero(np.all(pixels == [50, 50], axis=1))
    assert len(collisions) == 1
    assert indices[collisions[0]] == 50 * 100 + 50
    assert points[collisions[0], 2] == pytest.approx(.5)


@pytest.mark.parametrize('problem,reason', [
    ('clipped', 'clipped'), ('invalid_depth', 'insufficient'),
    ('mixed_depth', 'spread'), ('unknown_pose', 'POSE_BLOCKER'),
    ('missing_front', 'front_edge_m'), ('unqualified', 'intrinsics_measured'),
    ('wrong_frame', 'frame IDs'), ('above_floor', 'ambiguous floor'),
    ('missing_confidence_min', 'confidence_min'), ('empty_mask', 'empty'),
    ('missing_servo_metadata', 'at acquisition'), ('servo_mismatch', 'at acquisition'),
])
def test_unsafe_or_incomplete_geometry_returns_null(scene, problem, reason):
    mask, depth, calibration, metadata = scene
    options = {}
    if problem == 'clipped': mask[:, 0:2] = True
    if problem == 'invalid_depth': depth[:] = 0
    if problem == 'mixed_depth': depth[85:, :] = 2000
    if problem == 'unknown_pose': options['servo_deg'] = None
    if problem == 'missing_front': calibration['front_edge_m'] = None
    if problem == 'unqualified': calibration['intrinsics_measured'] = False
    if problem == 'wrong_frame': metadata['rgb_frame'] = 'another_camera'
    if problem == 'above_floor': calibration['views'][0]['tof_to_base'][2][3] = 1.
    if problem == 'missing_confidence_min': options['confidence'] = np.ones(depth.shape)
    if problem == 'empty_mask': mask[:] = False
    if problem == 'missing_servo_metadata': metadata.pop('servo_command_deg')
    if problem == 'servo_mismatch': metadata['servo_command_deg'] = 80.
    result, _, _ = analyse(scene, **options)
    assert result['clearance_m'] is None
    assert reason in result['reason']


def test_unqualified_and_nominal_preserve_useful_masks(scene):
    scene[2]['intrinsics_measured'] = False
    result, native, contact = analyse(scene)
    assert result['clearance_m'] is None and native.any() and contact.any()
    scene[2]['tof_K'] = None
    result, native, _ = analyse(scene, nominal=dict(rgb_diagonal_deg=90., tof_diagonal_deg=70., baseline_x_m=.04))
    assert result['projection_available'] and native.any()
    assert result['clearance_m'] is None


def test_partial_rgb_mask_has_surface_range_without_floor_clearance(scene):
    scene[0][:] = False
    scene[0][0:45, 35:65] = True
    result, native, _ = analyse(scene)
    assert result['mask_clipped'] is True and native.any()
    assert result['visible_surface_range_m'] is not None
    assert result['clearance_m'] is None


def test_partial_tof_overlap_keeps_only_observed_surface(scene):
    scene[0][:] = False
    scene[0][20:80, 60:99] = True
    scene[2]['tof_to_rgb'][0][3] = -.3
    result, native, _ = analyse(scene)
    assert result['tof_boundary_clipped'] is True
    assert result['visible_surface_range_m'] is not None
    assert native.any() and np.count_nonzero(native) < np.count_nonzero(scene[0])


def test_nominal_or_mixed_partial_surface_does_not_claim_range(scene):
    scene[2]['alignment_measured'] = False
    result, _, _ = analyse(scene)
    assert result['visible_surface_range_m'] is None
    assert 'alignment_measured' in result['surface_range_reason']
    scene[2]['alignment_measured'] = True
    scene[1][10:50] = 2000.
    result, _, _ = analyse(scene)
    assert result['visible_surface_range_m'] is None
    assert 'spread' in result['surface_range_reason']


def test_factory_intrinsics_override_fov_only_for_unqualified_preview(scene):
    mask, depth, calibration, metadata = scene
    metadata['tof_camera_metadata'] = dict(factory_intrinsics_available=True,
        width=100, height=100, K=calibration['tof_K'])
    result, _, _ = analyse(scene, nominal=dict(rgb_diagonal_deg=90., tof_diagonal_deg=70., baseline_x_m=.04))
    assert result['projection_provenance']['tof_intrinsics'] == 'SDK factory controls'
    assert result['clearance_m'] is None and result['projection_provenance']['qualified'] is False
    metadata['tof_camera_metadata']['width'] = 240
    result, _, _ = analyse(scene, nominal=dict(rgb_diagonal_deg=90., tof_diagonal_deg=70., baseline_x_m=.04))
    assert 'dimensions differ' in result['reason']


def write_capture(path, scene, stamp=1.):
    path.mkdir()
    metadata = dict(scene[3], rgb_stamp=stamp, depth_stamp=stamp + .02, receipt_stamp=stamp + .1)
    (path / 'capture.json').write_text(json.dumps(metadata))
    cv2.imwrite(str(path / 'rgb.png'), np.zeros((*scene[0].shape, 3), np.uint8))
    np.save(path / 'depth-mm.npy', scene[1])


@pytest.mark.parametrize('problem,reason', [('units', 'units'), ('stale', '500 ms'), ('pairing', '80 ms'), ('duplicate', 'duplicate')])
def test_capture_contract(tmp_path, scene, problem, reason):
    write_capture(tmp_path / 'frame-000', scene)
    write_capture(tmp_path / 'frame-001', scene, 1.2)
    metadata_path = tmp_path / 'frame-001/capture.json'
    metadata = json.loads(metadata_path.read_text())
    if problem == 'units': metadata.pop('depth_units')
    if problem == 'stale': metadata['receipt_stamp'] += 1.
    if problem == 'pairing': metadata['depth_stamp'] += .2
    if problem == 'duplicate': metadata.update(rgb_stamp=1., depth_stamp=1.02, receipt_stamp=1.1)
    metadata_path.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match=reason): diagnostic.load_inputs(tmp_path)


def test_burst_records_calibration_failures_and_clears_absent_target(tmp_path, scene, monkeypatch):
    import ultralytics
    root = tmp_path / 'capture'
    root.mkdir()
    for index in range(3): write_capture(root / f'frame-{index:03d}', scene, 1. + index * .2)
    calibration = tmp_path / 'calibration.yaml'
    scene[2]['intrinsics_measured'] = False
    calibration.write_text(yaml.safe_dump(scene[2]))
    model_path = tmp_path / 'model.pt'
    model_path.write_bytes(b'synthetic checkpoint for orchestration only')
    monkeypatch.setattr(diagnostic, 'MODEL_SHA256', diagnostic.hashlib.sha256(model_path.read_bytes()).hexdigest())
    monkeypatch.setattr(diagnostic, 'ensure_asset', lambda *args: model_path)
    prompts = []
    class Model:
        count = 0
        def __init__(self, *args, **kwargs): pass
        def set_classes(self, classes): prompts.extend(classes)
        def predict(self, *args, **kwargs):
            self.count += 1
            boxes = torch.tensor([[20., 10., 80., 90.]]) if self.count < 3 else torch.empty((0, 4))
            return [SimpleNamespace(boxes=SimpleNamespace(xyxy=boxes, conf=torch.tensor([.9])),
                masks=SimpleNamespace(data=torch.tensor(scene[0][None, ...], dtype=torch.float32)))]
    monkeypatch.setattr(ultralytics, 'YOLOE', Model)
    for target in ('box', 'shoe'):
        output = tmp_path / target
        monkeypatch.setattr(sys, 'argv', ['test', '--target', target, '--model', str(model_path),
            '--capture', str(root), '--calibration', str(calibration), '--output', str(output),
            '--reference-clearance-m', '.38'])
        diagnostic.main()
        report = json.loads((output / 'result.json').read_text())
        assert report['prompt'] == target and len(report['observations']) == 3
        assert report['observations'][0]['detected'] is True
        assert report['observations'][2]['detected'] is False
        assert report['observations'][2]['clearance_m'] is None
        assert report['clearance_m'] is None and report['motion_authority'] is False
        assert report['physical_reference_clearance_m'] == .38
        assert report['physical_validation']['status'] == 'not_evaluable'
        for record in report['observations']:
            for artifact in report['artifacts']: assert (output / record['artifact_directory'] / artifact).is_file()
    assert prompts == ['box', 'shoe']


def test_ambiguous_identity_rejected():
    identity = diagnostic.Identity()
    identity.select([[10, 10, 70, 70]], 100, 100)
    with pytest.raises(ValueError, match='ambiguous'):
        identity.select([[10, 10, 70, 70], [12, 12, 72, 72]], 100, 100)


def test_urdf_probe_does_not_qualify_missing_calibration(scene):
    scene[2]['intrinsics_measured'] = False
    result, _, _ = analyse(scene, urdf_geometry=dict(tof_to_base=scene[2]['views'][0]['tof_to_base']))
    assert result['clearance_m'] is None
    assert result['reason'].startswith('CALIBRATION_BLOCKER')
    probe = result['urdf_contact_probe']
    assert probe['candidate_floor_samples'] >= 12
    assert probe['front_clearance_m'] is None
    assert probe['labels'] == 'ESTIMATED / UNQUALIFIED'
