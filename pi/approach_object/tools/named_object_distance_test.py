#!/usr/bin/env python3
"""Offline visual-object surface distance. No floor, ROS or motion required.

Return provisional camera-to-visible-surface range using RGB/ToF alignment.
Saved-image inference latency does not qualify online perception.
"""
import argparse
import hashlib
import importlib.metadata as metadata
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from capture_preview import data_file, depth_preview

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from approach_object.geometry import Identity, transform
from approach_object.visual_target import VisualTarget
from approach_object.surface_distance import select_surface, segment_object, STATES
from approach_object.depth_registration import point_footprints, refine_translation
from approach_object.named_object_pin import (MODEL_SHA256, DEPENDENCIES, CLIP_REVISION,
    TEXT_ENCODER_SHA256, TEXT_ENCODER_URL, ensure_asset)

FLAGS = ('intrinsics_measured', 'alignment_measured', 'poses_measured', 'depth_validated')
VIEW_ANGLE_MAP = [(80.,54.),(90.,61.),(100.,71.),(110.,80.),(120.,90.),(130.,100.),(140.,111.),(150.,121.)]


def estimated_view_angle(command):
    if command is None or not 80 <= command <= 150:
        return None
    return float(np.interp(command, [p[0] for p in VIEW_ANGLE_MAP], [p[1] for p in VIEW_ANGLE_MAP]))


def load_inputs(path):
    root = Path(path)
    folders = sorted(root.glob('frame-*')) or [root]
    frames = []
    previous = None
    for folder in folders:
        meta = json.loads(data_file(folder, 'capture.json').read_text())
        rgb = cv2.imread(str(folder / 'rgb.png'), cv2.IMREAD_COLOR)
        depth_path = data_file(folder, 'depth-mm.npy')
        if rgb is None or not depth_path.is_file():
            raise ValueError(f'{folder}: RGB image or depth-mm.npy missing/unreadable')
        depth = np.load(depth_path, allow_pickle=False)
        if depth.ndim != 2 or depth.dtype.kind not in 'fiu':
            raise ValueError(f'{folder}: depth must be a numeric 2D array')
        if not meta.get('rgb_frame') or not meta.get('tof_frame'):
            raise ValueError(f'{folder}: missing RGB or ToF frame ID')
        if not np.isfinite([meta['rgb_stamp'], meta['depth_stamp'], meta.get('receipt_stamp', np.nan)]).all():
            raise ValueError(f'{folder}: missing or invalid acquisition/receipt timestamps')
        if abs(float(meta['rgb_stamp']) - float(meta['depth_stamp'])) > .08:
            raise ValueError(f'{folder}: RGB/ToF timestamp separation exceeds 80 ms')
        age = float(meta['receipt_stamp']) - min(float(meta['rgb_stamp']), float(meta['depth_stamp']))
        if age < 0 or age > .5:
            raise ValueError(f'{folder}: capture was older than 500 ms on receipt')
        units = meta.get('depth_units', '')
        if units not in ('mm', 'millimetres') and not units.startswith('publisher millimetres'):
            raise ValueError(f'{folder}: depth units are missing or are not millimetres')
        stamps = (float(meta['rgb_stamp']), float(meta['depth_stamp']))
        if previous is not None and any(current <= old for current, old in zip(stamps, previous)):
            raise ValueError(f'{folder}: duplicate or unordered acquisition frames')
        previous = stamps
        frames.append((folder, meta, rgb, depth))
    if not frames:
        raise ValueError('capture contains no frames')
    return frames


def unletterbox_mask(mask, shape):
    h, w = shape
    mh, mw = mask.shape
    gain = min(mw / w, mh / h)
    px, py = (mw - w * gain) / 2, (mh - h * gain) / 2
    x0, y0 = int(round(px)), int(round(py))
    x1, y1 = int(round(mw-px)), int(round(mh-py))
    return cv2.resize(mask[y0:y1, x0:x1], (w, h), interpolation=cv2.INTER_NEAREST) > .5


def project_native(depth_mm, calibration, rgb_shape, offset=(0., 0.)):
    """Return unique native samples surviving a surface depth buffer."""
    owners, points, _, projected = point_footprints(depth_mm, calibration, rgb_shape, offset)
    indices = np.unique(owners[owners >= 0])
    pixels = np.rint(projected[indices]).astype(int)
    rh, rw = rgb_shape
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < rw) & (pixels[:, 1] >= 0) & (pixels[:, 1] < rh)
    return indices[inside], pixels[inside], points[indices[inside]]


def projection_geometry(c, rgb_shape, depth_shape, nominal=None, factory_metadata=None):
    """Validate RGB/ToF projection geometry independently of robot geometry."""
    if nominal is None:
        geometry = c
    else:
        from fov_overlap_preview import nominal_k
        matrix = np.eye(4)
        matrix[0, 3] = nominal['baseline_x_m']
        geometry = dict(c, rgb_shape=rgb_shape, tof_shape=depth_shape,
                        rgb_K=nominal_k(rgb_shape, nominal['rgb_diagonal_deg']),
                        tof_K=nominal_k(depth_shape, nominal['tof_diagonal_deg']),
                        rgb_D=[0.] * 5, tof_D=[0.] * 5,
                        tof_to_rgb=matrix.tolist(), depth_semantics=nominal.get('depth_semantics', 'radial'))
        if factory_metadata and factory_metadata.get('factory_intrinsics_available') is True:
            if (factory_metadata.get('height'), factory_metadata.get('width')) != tuple(depth_shape):
                raise ValueError('PROJECTION_BLOCKER: factory camera mode dimensions differ from capture')
            geometry['tof_K'] = factory_metadata.get('K')
    for name in ('rgb_K', 'tof_K'):
        k = np.asarray(geometry.get(name), dtype=float)
        if k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError(f'PROJECTION_BLOCKER: {name} is missing or invalid')
    for name in ('rgb_D', 'tof_D'):
        d = np.asarray(geometry.get(name), dtype=float)
        if d.ndim != 1 or d.size not in (4, 5, 8, 12, 14) or not np.isfinite(d).all():
            raise ValueError(f'PROJECTION_BLOCKER: {name} is missing or invalid')
    transform(np.empty((0, 3)), geometry.get('tof_to_rgb'))
    if tuple(geometry.get('rgb_shape', ())) != tuple(rgb_shape) or tuple(geometry.get('tof_shape', ())) != tuple(depth_shape):
        raise ValueError('PROJECTION_BLOCKER: geometry dimensions do not match images')
    if geometry.get('depth_semantics') not in ('radial', 'optical_z'):
        raise ValueError('PROJECTION_BLOCKER: depth semantics are unknown')
    return geometry


def analyse_mask(mask, depth, confidence, c, meta, servo_deg=None, nominal=None, urdf_geometry=None, rgb=None, amplitude=None, evidence=None):
    """Associate visible RGB extent with ToF range; no floor/servo pose needed.

    Legacy clearance fields remain null for consumers; motion has no authority.
    The third return is the accepted surface mask, not a floor contact mask.
    """
    native = np.zeros(depth.shape, np.uint8)
    accepted_native = np.zeros(depth.shape, np.uint8)
    diagnostics = dict(clearance_m=None, visible_surface_range_m=None,
                       distance_cm=None, distance_status='unavailable',
                       distance_origin='tof_camera', surface_samples=0,
                       surface_range_reason='no associated visible surface',
                       projection_available=False)
    try:
        if not mask.any():
            raise ValueError('segmentation mask is empty')
        if meta.get('rgb_frame') != c.get('rgb_frame') or meta.get('tof_frame') != c.get('tof_frame'):
            raise ValueError('CAPTURE_BLOCKER: frame IDs do not match calibration')
        geometry = projection_geometry(c, mask.shape, depth.shape, nominal, meta.get('tof_camera_metadata'))
        confidence_applied = False
        reliable = np.ones(depth.shape, bool)
        if confidence is not None and confidence.shape != depth.shape:
            raise ValueError('confidence dimensions differ from ToF depth')
        if c.get('confidence_min') is not None:
            threshold = float(c['confidence_min'])
            if not np.isfinite(threshold) or threshold < 0:
                raise ValueError('invalid confidence_min')
            if confidence is None:
                raise ValueError('required confidence stream is missing')
            # Preserve membership even when the measurement cannot support range.
            reliable = np.isfinite(confidence) & (confidence >= threshold)
            confidence_applied = True
        owners, all_points, _, projected = point_footprints(depth, geometry, mask.shape)
        registration = dict(status='unavailable', offset_xy_px=[0., 0.], reason='RGB/IR refinement inputs missing', qualified=False)
        if rgb is not None and c.get('registration_refine_background', True) and nominal is None:
            offset, registration = refine_translation(rgb, amplitude, owners, mask, geometry)
            if registration['status'] == 'provisional':
                owners, all_points, _, projected = point_footprints(depth, geometry, mask.shape, offset)
            # A failed cross-modal fit provides no displacement evidence.
            # Keep the measured calibration rather than treating an unsupported
            # optimizer proposal as proof that geometry is wrong.
        registration['applied'] = registration['status'] == 'provisional'
        registration['geometry_used'] = ('calibration_plus_background_translation'
                                         if registration['applied'] else 'calibration')
        registration['alignment_evidence'] = ('supported_translation'
                                              if registration['applied'] else 'inconclusive')
        indices, in_object, support, native, accepted_native, membership_info, maps = segment_object(
            mask, depth, owners, all_points, projected, reliable, c)
        points = all_points[indices]
        valid_owners = owners >= 0
        diagnostics.update(membership_info)
        warnings = []
        if registration['status'] == 'provisional':
            warnings.append(registration['reason'])
        else:
            warnings.append('using fitted geometry; background alignment check inconclusive: '
                            + registration.get('reason', 'no supported translation'))
        if confidence is not None and not confidence_applied:
            warnings.append('confidence stream has no calibrated threshold; not used')
        kept, surface = select_surface(indices, points, support, depth.shape, c)
        accepted_native[:] = 0
        accepted_native.ravel()[indices[kept]] = 255
        maps['sample_state'][maps['sample_state'] == 6] = 5
        maps['sample_state'].ravel()[indices[kept]] = 6
        diagnostics['sample_audit_counts'] = {name: int((maps['sample_state'] == code).sum())
                                            for code, name in STATES.items()}
        if evidence is not None:
            evidence.update(maps)
        diagnostics.update(surface)
        diagnostics.update(projection_available=True, frame_registration=registration,
                           visibility_method='bounded_native_point_footprints_zbuffer',
                           registered_rgb_depth_coverage=float(valid_owners.mean()),
                           native_object_samples=int(in_object.sum()),
                           rejected_surface_samples=int(in_object.sum()-kept.sum()),
                           object_depth_samples=int(in_object.sum()),
                           distance_support_fraction=float(kept.sum()/max(in_object.sum(), 1)),
                           object_depth_definition='visible target membership from RGB-guided native graph; SDK reliability affects distance support separately',
                           mask_clipped=bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any()),
                           tof_boundary_clipped=bool(native[0].any() or native[-1].any() or native[:, 0].any() or native[:, -1].any()),
                           confidence_filter_applied=False,
                           confidence_reliability_threshold_applied=confidence_applied,
                           quality_warnings=warnings,
                           projection_provenance=dict(source='nominal' if nominal else c.get('alignment_source', 'calibration file'),
                                                      depth_semantics=geometry['depth_semantics'],
                                                      qualified=nominal is None and registration['status'] != 'provisional' and all(c.get(f) is True for f in
                                                          ('intrinsics_measured', 'alignment_measured', 'depth_validated'))))
        diagnostics['reason'] = diagnostics['surface_range_reason']
    except (ValueError, KeyError, TypeError, IndexError, cv2.error) as e:
        diagnostics.update(reason=str(e), surface_range_reason=str(e))
    return diagnostics, native, accepted_native


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', help='free-form text prompt, or display label with --reference')
    p.add_argument('--reference', type=Path, help='visual example image; avoids class-name recognition')
    p.add_argument('--box', nargs=4, type=float, help='reference-image box x0 y0 x1 y1')
    p.add_argument('--selection-box', nargs=4, type=float, help='optional first-capture box to select among reference matches')
    p.add_argument('--model', required=True, help='screened YOLOE-26n-seg checkpoint')
    p.add_argument('--capture', required=True, help='single capture directory or frame-NNN burst')
    p.add_argument('--calibration', required=True)
    p.add_argument('--servo-deg', type=float, help='known servo command at acquisition; omit if unknown')
    p.add_argument('--urdf', help=argparse.SUPPRESS) # Legacy argument; range uses no robot pose.
    p.add_argument('--nominal', action='store_true', help='explicitly enable unqualified nominal ToF projection previews')
    p.add_argument('--rgb-diagonal-deg', type=float, default=90.)
    p.add_argument('--tof-diagonal-deg', type=float, default=70.)
    p.add_argument('--baseline-x-m', type=float, default=.04)
    p.add_argument('--nominal-depth-semantics', choices=('radial', 'optical_z'), default='radial')
    p.add_argument('--imgsz', type=int, default=640, help='stationary inference input size; 640 default, 960 tested for box')
    p.add_argument('--reference-clearance-m', type=float, help=argparse.SUPPRESS)
    p.add_argument('--reference-distance-cm', type=float, help='measured distance from ToF camera to visible object surface')
    p.add_argument('--output', required=True)
    a = p.parse_args()
    target = a.target.strip() if a.target is not None else ('selected object' if a.reference else '')
    if not target:
        p.error('provide a nonempty --target or a visual --reference with --box')
    reference = None
    if a.reference is not None:
        reference = cv2.imread(str(a.reference))
        if reference is None or a.box is None or not np.isfinite(a.box).all():
            p.error('visual prompting requires a readable --reference and finite --box')
        x0, y0, x1, y1 = a.box
        if not (0 <= x0 < x1 <= reference.shape[1] and 0 <= y0 < y1 <= reference.shape[0]):
            p.error('--box must lie inside reference image')
    elif a.box is not None or a.selection_box is not None:
        p.error('--box and --selection-box require --reference')
    if a.servo_deg is not None and not np.isfinite(a.servo_deg):
        p.error('--servo-deg must be finite')
    if not 320 <= a.imgsz <= 1280 or a.imgsz % 32:
        p.error('--imgsz must be a multiple of 32 between 320 and 1280')
    if a.reference_clearance_m is not None and (not np.isfinite(a.reference_clearance_m) or a.reference_clearance_m < 0):
        p.error('--reference-clearance-m must be finite and nonnegative')
    if a.reference_distance_cm is not None and (not np.isfinite(a.reference_distance_cm) or a.reference_distance_cm <= 0):
        p.error('--reference-distance-cm must be finite and positive')
    nominal = None
    if a.nominal:
        if not all(0 < v < 180 for v in (a.rgb_diagonal_deg, a.tof_diagonal_deg)) or not np.isfinite(a.baseline_x_m):
            p.error('invalid nominal geometry')
        nominal = dict(rgb_diagonal_deg=a.rgb_diagonal_deg, tof_diagonal_deg=a.tof_diagonal_deg,
                       baseline_x_m=a.baseline_x_m, depth_semantics=a.nominal_depth_semantics)
    model_path = Path(a.model).resolve()
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        p.error('checkpoint differs from screened YOLOE-26n-seg (SHA-256 mismatch)')
    c = yaml.safe_load(Path(a.calibration).read_text())
    frames = load_inputs(a.capture)
    urdf_geometry = None # Robot pose is not used for camera surface range.
    runtime_dependencies = {}
    for package, required in DEPENDENCIES.items():
        try:
            installed = metadata.version(package)
        except metadata.PackageNotFoundError:
            p.error(f'required workstation dependency is missing: {package}=={required}')
        runtime_dependencies[package] = installed
        if installed != required:
            p.error(f'{package}=={installed} is installed; use pinned {package}=={required}')
    clip_source = metadata.distribution('clip').read_text('direct_url.json')
    clip_revision = json.loads(clip_source or '{}').get('vcs_info', {}).get('commit_id')
    if clip_revision != CLIP_REVISION:
        p.error('CLIP tokenizer differs from pinned source revision')
    output = Path(a.output).resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        p.error('output must be a fresh or empty directory to avoid stale artifacts')
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    encoder = ensure_asset(model_path.parent / 'mobileclip2_b.ts', TEXT_ENCODER_URL, TEXT_ENCODER_SHA256)
    from ultralytics import YOLOE
    model = YOLOE(str(model_path), task='segment')
    original_cwd = Path.cwd()
    try:
        os.chdir(encoder.parent)
        if reference is None:
            model.set_classes([target])
    finally:
        os.chdir(original_cwd)
    setup_latency = time.perf_counter() - started
    identity = Identity()
    visual_identity = VisualTarget() if reference is not None else None
    visual_prompt_ready = False
    records, native_masks, surface_masks = [], [], []
    first_rgb_mask = None
    for index, (folder, meta, rgb, depth) in enumerate(frames):
        rec = dict(frame=folder.name, rgb_stamp=meta['rgb_stamp'], tof_stamp=meta['depth_stamp'],
                   rgb_tof_delta_s=abs(meta['rgb_stamp']-meta['depth_stamp']), target=target,
                   receipt_stamp=meta['receipt_stamp'],
                   acquisition_age_s=meta['receipt_stamp']-min(meta['rgb_stamp'], meta['depth_stamp']),
                   capture_servo_command_deg=meta.get('servo_command_deg'),
                   capture_pose_source=meta.get('pose_source', 'not recorded'),
                   rgb_sha256=hashlib.sha256((folder/'rgb.png').read_bytes()).hexdigest(),
                   depth_sha256=hashlib.sha256(data_file(folder, 'depth-mm.npy').read_bytes()).hexdigest(),
                   detected=False, clearance_m=None, visible_surface_range_m=None,
                   distance_cm=None, distance_status='unavailable', surface_samples=0,
                   visibility='lost', reason='')
        inference_started = time.perf_counter()
        mask = np.zeros(rgb.shape[:2], bool)
        native = np.zeros(depth.shape, np.uint8)
        surface_native = np.zeros(depth.shape, np.uint8)
        evidence = {}
        try:
            predict_options = {}
            if visual_identity is not None and not visual_prompt_ready:
                from ultralytics.models.yolo.yoloe.predict import YOLOEVPSegPredictor
                predict_options = dict(predictor=YOLOEVPSegPredictor, refer_image=reference,
                    visual_prompts=dict(bboxes=np.array([a.box]), cls=np.array([0])))
            result = model.predict(rgb, imgsz=a.imgsz, conf=.5, verbose=False, device='cpu', **predict_options)[0]
            if visual_identity is not None:
                visual_prompt_ready = True  # reference embedding persists in YOLOE
            boxes = result.boxes.xyxy.cpu().numpy() if result.boxes is not None else np.empty((0, 4))
            scores = result.boxes.conf.cpu().numpy() if result.boxes is not None else np.empty(0)
            if visual_identity is not None:
                candidates = [] if result.masks is None else [
                    unletterbox_mask(m.cpu().numpy(), rgb.shape[:2]) for m in result.masks.data]
                chosen, tracking = visual_identity.select(rgb, candidates, meta['rgb_stamp'],
                    a.selection_box if index == 0 else None)
                rec.update(tracking)
            else:
                chosen = identity.select([b.tolist() for b in boxes], rgb.shape[1], rgb.shape[0])
            if chosen is None:
                raise ValueError('target absent; no distance retained')
            if result.masks is None:
                raise ValueError('segmentation mask absent')
            mask = unletterbox_mask(result.masks.data[chosen].cpu().numpy(), rgb.shape[:2])
            rec.update(detected=True, selected_identity=visual_identity.identity if visual_identity is not None else identity.identity,
                       score=float(scores[chosen]), bbox_xyxy=boxes[chosen].tolist(),
                       mask_pixels=int(mask.sum()))
            if visual_identity is None:
                rec['visibility'] = 'visible_extent_unknown'
            if index == 0:
                first_rgb_mask = mask
            confidence_file = data_file(folder, 'confidence.npy')
            confidence = np.load(confidence_file, allow_pickle=False) if confidence_file.is_file() else None
            amplitude_file = data_file(folder, 'amplitude.npy')
            amplitude = np.load(amplitude_file, allow_pickle=False) if amplitude_file.is_file() else None
            diagnostics, native, surface_native = analyse_mask(mask, depth, confidence, c, meta, a.servo_deg, nominal, urdf_geometry,
                                                               rgb=rgb, amplitude=amplitude, evidence=evidence)
            rec.update(diagnostics)
        except (ValueError, KeyError, TypeError, IndexError, cv2.error) as e:
            rec['reason'] = str(e)
            rec['distance_status'] = 'ambiguous' if 'ambiguous' in str(e) else 'unavailable'
            if not rec['detected']:
                rec['visibility'] = 'ambiguous' if 'ambiguous' in str(e) else 'lost'
        native_masks.append(native)
        surface_masks.append(surface_native)
        rec['processing_latency_s'] = time.perf_counter() - inference_started
        destination = output if len(frames) == 1 else output / 'frames' / f'frame-{index:03d}'
        save_artifacts(destination, folder, rgb, depth, mask, native, surface_native, target, rec,
                       projection_geometry(c, mask.shape, depth.shape, nominal, meta.get('tof_camera_metadata')) if rec.get('projection_available') else None)
        if evidence:
            save_evidence(destination, evidence)
            if index == 0 and len(frames) > 1:
                save_evidence(output, evidence)
        rec['artifact_directory'] = str(destination.relative_to(output))
        records.append(rec)

    elapsed = time.perf_counter() - started
    # Root artifacts retain the first observation for convenient review.
    folder, meta, rgb, depth = frames[0]
    mask = first_rgb_mask if first_rgb_mask is not None else np.zeros(rgb.shape[:2], bool)
    native = native_masks[0]
    surface = surface_masks[0]
    if len(frames) > 1:
        save_artifacts(output, folder, rgb, depth, mask, native, surface, target, records[0],
                       projection_geometry(c, mask.shape, depth.shape, nominal, meta.get('tof_camera_metadata')) if records[0].get('projection_available') else None)

    distances = [r['distance_cm'] for r in records if r.get('distance_cm') is not None]
    all_readable = len(distances) == len(records)
    # Do not hide disagreeing frames behind a median. Require the measured
    # surface intervals to overlap without adding a scene-specific tolerance.
    intervals = [r.get('surface_range_percentiles_cm') for r in records]
    interval_overlap = None
    if all_readable and all(v is not None for v in intervals):
        interval_overlap = [max(v[0] for v in intervals), min(v[2] for v in intervals)]
    consistent = (interval_overlap is not None and interval_overlap[0] <= interval_overlap[1])
    aggregate_distance = float(np.median(distances)) if all_readable and consistent else None
    aggregate_clearance = None
    error = abs(aggregate_distance-a.reference_distance_cm) if aggregate_distance is not None and a.reference_distance_cm is not None else None
    blocker = next((r['reason'] for r in records if r.get('distance_cm') is None), None)
    if all_readable and not consistent:
        blocker = 'consecutive surface range intervals disagree; no aggregate distance'
    confidence_path = data_file(folder, 'confidence.npy')
    report = dict(mode='OFFLINE_VISUAL_TARGET_DIAGNOSTIC' if reference is not None else 'OFFLINE_OBJECT_SURFACE_DISTANCE', target=target,
                  prompt_mode='visual' if reference is not None else 'text',
                  reference_sha256=hashlib.sha256(a.reference.read_bytes()).hexdigest() if reference is not None else None,
                  reference_box=a.box, selection_box=a.selection_box,
                  tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  prompt=target if reference is None else None, imgsz=a.imgsz, confidence_threshold=.5,
                  checkpoint=str(model_path), checkpoint_sha256=digest,
                  text_encoder_sha256=hashlib.sha256(encoder.read_bytes()).hexdigest(),
                  clip_source_revision=clip_revision,
                  runtime_dependencies=runtime_dependencies,
                  requirements_lock_sha256=hashlib.sha256((Path(__file__).resolve().parents[1]/'requirements-yoloe-workstation.txt').read_bytes()).hexdigest(),
                  pinned_dependencies=DEPENDENCIES,
                  servo_command_deg=a.servo_deg, estimated_mast_viewing_angle_deg=estimated_view_angle(a.servo_deg),
                  observations=records, clearance_m=aggregate_clearance,
                  distance_cm=aggregate_distance, distance_origin='tof_camera',
                  depth_statistic='median', aggregation_statistic='median_of_frame_medians',
                  distance_status='provisional' if aggregate_distance is not None else ('ambiguous' if (all_readable and not consistent) or any(r.get('distance_status') == 'ambiguous' for r in records) else 'unavailable'),
                  temporal_consistency=dict(frames=len(records), all_frames_readable=all_readable,
                                            status=('single_frame' if len(records) == 1 and consistent else
                                                    'consistent' if consistent else 'inconsistent' if all_readable else 'not_evaluable'),
                                            frame_distances_cm=[r.get('distance_cm') for r in records],
                                            shared_surface_interval_cm=interval_overlap if consistent else None,
                                            criterion='overlap of measured P5-P95 surface range intervals; not an accuracy bound'),
                  visible_surface_range_m=aggregate_distance/100 if aggregate_distance is not None else None,
                  reference_distance_cm=a.reference_distance_cm,
                  physical_validation=dict(status='not_evaluable' if error is None else ('pass_this_scene' if error <= 5.0 else 'fail_this_scene'),
                                           absolute_error_cm=error, tolerance_cm=5.0,
                                           qualification_complete=False),
                  surface_samples=int(records[0].get('surface_samples', 0)),
                  surface_depth_spread_cm=records[0].get('surface_depth_spread_cm'),
                  quality_reason=blocker or 'estimated visible surface range; accuracy unverified',
                  calibration_provenance={'path':str(Path(a.calibration)), 'flags':{f:c.get(f) for f in FLAGS},
                                           'sha256':hashlib.sha256(Path(a.calibration).read_bytes()).hexdigest(),
                                           'depth_semantics':c.get('depth_semantics'),
                                           'alignment_source':c.get('alignment_source')},
                  nominal_geometry=nominal, urdf_geometry=urdf_geometry,
                  geometry_assumptions=[] if nominal is None else ['parallel optical axes', 'centred uncropped RGB pinhole image', 'assumed ToF ' + a.nominal_depth_semantics,
                      'nominal RGB FOV; current crop unvalidated', 'SDK ToF K when available, nominal ToF FOV otherwise', 'unknown distortion assumed zero for preview'],
                  confidence_stream_available=confidence_path.is_file(),
                  confidence_min=c.get('confidence_min'),
                  setup_latency_s=setup_latency,
                  processing_latency_s=elapsed, motion_authority=False,
                  labels='ESTIMATED / UNQUALIFIED',
                  artifacts=['distance.png', 'tof-depth.png', 'isolated-depth.png', 'data/object-mask.png',
                             'data/tof-object-mask.png', 'data/surface-sample-mask.png', 'data/isolated-depth-mm.npy',
                             'membership.png', 'sample-states.png', 'data/membership_probability.npy',
                             'data/rgb_membership_prior.npy', 'data/sample_state.npy', 'data/depth_reliable.npy'])
    (output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


def save_evidence(output, evidence):
    """Save machine-readable membership/reliability plus labelled audit views."""
    data = output / 'data'
    data.mkdir(parents=True, exist_ok=True)
    for name, array in evidence.items():
        np.save(data / (name+'.npy'), array)
    probability = evidence['membership_probability']
    image = cv2.applyColorMap((np.nan_to_num(probability)*255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    image[~np.isfinite(probability)] = 0
    image = cv2.resize(image, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    header = np.zeros((45, image.shape[1], 3), np.uint8)
    cv2.putText(header, 'Membership: purple 0, yellow 1 | black unknown', (8, 27), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
    cv2.imwrite(str(output/'membership.png'), np.vstack((header, image)))
    palette = np.array([[0,0,0], [100,100,100], [110,60,30], [65,65,65],
                        [0,180,255], [180,70,180], [70,200,70]], np.uint8)
    image = cv2.resize(palette[evidence['sample_state']], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    header = np.zeros((24*len(STATES), image.shape[1], 3), np.uint8)
    for code, label in STATES.items():
        cv2.putText(header, f'{code}: {label}', (8, 20+24*code), cv2.FONT_HERSHEY_SIMPLEX, .45, tuple(int(x) for x in palette[code]) if code else (220,220,220), 1)
    cv2.imwrite(str(output/'sample-states.png'), np.vstack((header, image)))
    (data/'sample-state-legend.json').write_text(json.dumps(STATES, indent=2)+'\n')


def save_artifacts(output, folder, rgb, depth, mask, native, accepted, target, diagnostics=None, geometry=None):
    """One RGB distance view, one depth preview; masks are kept under data/."""
    output.mkdir(parents=True, exist_ok=True)
    diagnostics = diagnostics or {}
    annotated = rgb.copy()
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(annotated, contours, -1, (0, 255, 255), 2)
    if geometry is not None:
        offset = diagnostics.get('frame_registration', {}).get('offset_xy_px', [0., 0.])
        indices, pixels, _ = project_native(depth, geometry, mask.shape, offset)
        selected = accepted.ravel()[indices] > 0
        for x, y in pixels[selected]:
            cv2.circle(annotated, (int(x), int(y)), 1, (255, 0, 255), -1)
    canvas = np.zeros((rgb.shape[0]+90, rgb.shape[1], 3), np.uint8)
    canvas[90:] = annotated
    distance = diagnostics.get('distance_cm')
    title = f'{distance:.1f} cm | provisional' if distance is not None else 'No distance | ' + diagnostics.get('distance_status', 'unavailable')
    spread = diagnostics.get('surface_depth_spread_cm')
    detail = f"ToF camera | {diagnostics.get('surface_samples', 0)} samples"
    if spread is not None:
        detail += f' | P5-P95 spread {spread:.1f} cm'
    for y, text in ((22, title), (46, detail), (72, 'Yellow: object | Magenta: accepted depth samples')):
        cv2.putText(canvas, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    cv2.imwrite(str(output/'distance.png'), canvas)
    cv2.imwrite(str(output/'tof-depth.png'), depth_preview(depth, native))
    # Object extent must not be replaced by the subset used to estimate range.
    # Preserve disconnected and uncertain measured patches for inspection.
    isolated = np.where(native > 0, depth, np.nan)
    preview = depth_preview(isolated)
    preview[:60] = 24
    isolated_title = 'Isolated object | all associated ToF measurements'
    cv2.putText(preview, isolated_title, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,255), 1)
    cv2.putText(preview, 'Black = outside association or invalid; colour is not validation', (12, 48), cv2.FONT_HERSHEY_SIMPLEX, .43, (255,255,255), 1)
    cv2.imwrite(str(output/'isolated-depth.png'), preview)
    data = output / 'data'
    data.mkdir(exist_ok=True)
    np.save(data/'isolated-depth-mm.npy', isolated)
    cv2.imwrite(str(data/'object-mask.png'), mask.astype(np.uint8)*255)
    cv2.imwrite(str(data/'tof-object-mask.png'), native)
    cv2.imwrite(str(data/'surface-sample-mask.png'), accepted)


if __name__ == '__main__':
    main()
