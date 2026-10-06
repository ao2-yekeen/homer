#!/usr/bin/env python3
"""Offline named-object mask and floor-contact diagnostic. No ROS or motion.

The numeric front clearance is emitted only from fully measured calibration.
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from approach_object.geometry import Identity, transform
from approach_object.visual_target import VisualTarget
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
        meta = json.loads((folder / 'capture.json').read_text())
        rgb = cv2.imread(str(folder / 'rgb.png'), cv2.IMREAD_COLOR)
        depth_path = folder / 'depth-mm.npy'
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


def project_native(depth_mm, calibration, rgb_shape):
    """Project samples to RGB, retaining their original ToF (x,y) indices."""
    h, w = depth_mm.shape
    yy, xx = np.indices((h, w))
    raw = np.asarray(depth_mm, dtype=float).ravel() * .001
    uv = np.column_stack((xx.ravel(), yy.ravel())).astype(float)
    rays = cv2.undistortPoints(uv.reshape(-1, 1, 2), np.asarray(calibration['tof_K'], float),
                               np.asarray(calibration['tof_D'], float)).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    semantics = calibration.get('depth_semantics')
    if semantics == 'radial':
        rays /= np.linalg.norm(rays, axis=1)[:, None]
    elif semantics != 'optical_z':
        raise ValueError('depth_semantics must be radial or optical_z')
    valid = np.isfinite(raw) & (raw >= float(calibration.get('depth_min_m', .05))) & (raw <= float(calibration.get('depth_max_m', 4)))
    indices = np.flatnonzero(valid)
    points = rays[valid] * raw[valid, None]
    rgb_points = transform(points, calibration['tof_to_rgb'])
    front = rgb_points[:, 2] > 0
    indices, points, rgb_points = indices[front], points[front], rgb_points[front]
    if not len(rgb_points):
        return indices, np.empty((0, 2), dtype=int), points
    projected, _ = cv2.projectPoints(rgb_points, np.zeros(3), np.zeros(3),
                                     np.asarray(calibration['rgb_K'], float), np.asarray(calibration['rgb_D'], float))
    pixels = np.rint(projected.reshape(-1, 2)).astype(int)
    rh, rw = rgb_shape
    inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < rw) & (pixels[:, 1] >= 0) & (pixels[:, 1] < rh)
    indices, pixels, points = indices[inside], pixels[inside], points[inside]
    if len(indices):
        # Keep only the frontmost ToF return when multiple native pixels land
        # on the same RGB pixel. This avoids counting a background wall behind
        # a visible target silhouette in the projected target surface.
        linear = pixels[:, 1].astype(np.int64) * rw + pixels[:, 0]
        order = np.lexsort((rgb_points[inside, 2], linear))
        ordered_pixels, ordered_indices = linear[order], order
        first = np.r_[True, ordered_pixels[1:] != ordered_pixels[:-1]]
        keep = ordered_indices[first]
        indices, pixels, points = indices[keep], pixels[keep], points[keep]
    return indices, pixels, points


def calibration_blocker(c, meta, shape, depth_shape, servo_deg):
    if meta['rgb_frame'] != c.get('rgb_frame') or meta['tof_frame'] != c.get('tof_frame'):
        return 'CAPTURE_BLOCKER: frame IDs do not match calibration'
    if tuple(shape) != tuple(c.get('rgb_shape', ())) or tuple(depth_shape) != tuple(c.get('tof_shape', ())):
        return 'CALIBRATION_BLOCKER: image dimensions do not match calibrated dimensions'
    for flag in FLAGS:
        if c.get(flag) is not True:
            return f'CALIBRATION_BLOCKER: {flag} is not qualified'
    if c.get('depth_semantics') not in ('radial', 'optical_z'):
        return 'CALIBRATION_BLOCKER: depth semantics are unknown'
    for key in ('rgb_K', 'rgb_D', 'tof_K', 'tof_D', 'tof_to_rgb', 'front_edge_m', 'rgb_frame', 'tof_frame', 'rgb_shape', 'tof_shape'):
        if c.get(key) is None:
            return f'CALIBRATION_BLOCKER: {key} is missing'
    if servo_deg is None:
        return 'POSE_BLOCKER: servo command at acquisition is unknown'
    held_command = meta.get('servo_command_deg')
    if held_command is None or not np.isfinite(held_command) or abs(held_command - servo_deg) > .01:
        return 'POSE_BLOCKER: capture does not record the requested servo command at acquisition'
    views = c.get('views')
    if not isinstance(views, list) or not views:
        return 'CALIBRATION_BLOCKER: calibrated view poses are missing'
    view = min(views, key=lambda item: abs(float(item.get('servo_deg', 1e9))-servo_deg))
    if abs(float(view.get('servo_deg', 1e9))-servo_deg) > .01 or view.get('tof_to_base') is None:
        return 'CALIBRATION_BLOCKER: no measured ToF-to-base pose for this servo command'
    return None


def projection_geometry(c, rgb_shape, depth_shape, nominal=None, factory_metadata=None):
    """Allow explicit nominal previews without qualifying metric clearance."""
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


def analyse_mask(mask, depth, confidence, c, meta, servo_deg, nominal=None, urdf_geometry=None):
    """Return masks even when measured geometry cannot support clearance."""
    native = np.zeros(depth.shape, np.uint8)
    contact_native = np.zeros(depth.shape, np.uint8)
    diagnostics = dict(clearance_m=None, visible_surface_range_m=None,
                       surface_range_reason='no reliable associated surface',
                       contact_samples=0, projection_available=False)
    blocker = calibration_blocker(c, meta, mask.shape, depth.shape, servo_deg)
    if not mask.any():
        return diagnostics | {'reason': 'segmentation mask is empty'}, native, contact_native
    try:
        if meta['rgb_frame'] != c.get('rgb_frame') or meta['tof_frame'] != c.get('tof_frame'):
            raise ValueError('CAPTURE_BLOCKER: frame IDs do not match calibration')
        factory = meta.get('tof_camera_metadata')
        geometry = projection_geometry(c, mask.shape, depth.shape, nominal, factory)
        indices, pixels, points = project_native(depth, geometry, mask.shape)
        eroded = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        in_object = eroded[pixels[:, 1], pixels[:, 0]]
        confidence_blocker = None
        if confidence is not None:
            if confidence.shape != depth.shape:
                raise ValueError('confidence dimensions differ from ToF depth')
            if c.get('confidence_min') is None:
                confidence_blocker = 'QUALITY_BLOCKER: no calibrated confidence_min'
            else:
                threshold = float(c['confidence_min'])
                if not np.isfinite(threshold) or threshold < 0:
                    raise ValueError('QUALITY_BLOCKER: invalid confidence_min')
                in_object &= np.isfinite(confidence.ravel()[indices]) & (confidence.ravel()[indices] >= threshold)
        elif c.get('confidence_min') is not None:
            confidence_blocker = 'QUALITY_BLOCKER: required confidence stream is missing'
        native.ravel()[indices[in_object]] = 255
        ys = np.where(mask)[0]
        contact_rgb = mask & (np.indices(mask.shape)[0] >= np.quantile(ys, .90))
        contact_eroded = cv2.erode(contact_rgb.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        selected = contact_eroded[pixels[:, 1], pixels[:, 0]] & in_object
        contact_native.ravel()[indices[selected]] = 255
        diagnostics.update(projection_available=True, native_object_samples=int(in_object.sum()),
                           candidate_contact_samples=int(selected.sum()),
                           mask_clipped=bool(mask[-2:, :].any() or mask[:2, :].any() or mask[:, :2].any() or mask[:, -2:].any()),
                           confidence_filter_applied=confidence is not None and c.get('confidence_min') is not None)
        diagnostics['projection_provenance'] = dict(
            tof_intrinsics='SDK factory controls' if nominal and factory and factory.get('factory_intrinsics_available') else ('nominal FOV' if nominal else 'calibration file'),
            depth_semantics=geometry['depth_semantics'], qualified=blocker is None and nominal is None)
        if in_object.any():
            ranges = np.linalg.norm(points[in_object], axis=1)
            percentiles = np.percentile(ranges, [5, 50, 95])
            diagnostics['raw_object_range_percentiles_m'] = percentiles.tolist()
            diagnostics['tof_boundary_clipped'] = bool(native[0].any() or native[-1].any()
                                                       or native[:, 0].any() or native[:, -1].any())
            surface_blocker = next((f'{flag} is not qualified' for flag in
                ('intrinsics_measured', 'alignment_measured', 'depth_validated')
                if c.get(flag) is not True), None)
            if nominal is not None:
                surface_blocker = 'nominal projection is unqualified'
            if confidence_blocker:
                surface_blocker = confidence_blocker
            if in_object.sum() < max(12, int(c.get('min_samples', 12))):
                surface_blocker = 'insufficient visible surface samples'
            if percentiles[2] - percentiles[0] > float(c.get('max_object_depth_span_m', .15)):
                surface_blocker = 'visible surface spread too large; possible mixed returns'
            diagnostics['surface_range_reason'] = surface_blocker or 'range to observed surface; unseen extent unknown'
            if surface_blocker is None:
                diagnostics['visible_surface_range_m'] = float(percentiles[1])
        if selected.any():
            diagnostics['depth_spread_m'] = float(np.ptp(np.linalg.norm(points[selected], axis=1)))
        if urdf_geometry is not None and selected.any():
            model_points = transform(points[selected], urdf_geometry['tof_to_base'])
            diagnostics['urdf_contact_probe'] = dict(
                labels='ESTIMATED / UNQUALIFIED',
                height_p5_p50_p95_m=np.percentile(model_points[:, 2], [5, 50, 95]).tolist(),
                candidate_floor_samples=int((np.abs(model_points[:, 2]) <= float(c.get('floor_reject_m', .015))).sum()),
                front_clearance_m=None,
                reason='URDF body origins and estimated optical pitch; no metric qualification')
        if blocker or confidence_blocker or nominal is not None:
            raise ValueError(blocker or confidence_blocker or 'CALIBRATION_BLOCKER: nominal geometry cannot provide verified clearance')
        if diagnostics['mask_clipped']:
            raise ValueError('contact region clipped by image boundary')
        if selected.sum() < max(12, int(c.get('min_samples', 12))):
            raise ValueError('insufficient reliable projected contact samples')
        if diagnostics['depth_spread_m'] > float(c.get('max_object_depth_span_m', .15)):
            raise ValueError('contact depth spread too large; mixed background or floor returns')
        view = min(c['views'], key=lambda item: abs(float(item['servo_deg']) - servo_deg))
        base = transform(points[selected], view['tof_to_base'])
        floor_support = np.abs(base[:, 2]) <= float(c.get('floor_reject_m', .015))
        base_contact = base[floor_support]
        if len(base_contact) < max(12, int(c.get('min_samples', 12))):
            raise ValueError('ambiguous floor contact: insufficient samples on calibrated floor plane')
        clearance = float(np.median(base_contact[:, 0]) - float(c['front_edge_m']))
        if not np.isfinite(clearance) or clearance < 0:
            raise ValueError('invalid contact clearance or contact behind robot front edge')
        contact_native[:] = 0
        contact_native.ravel()[indices[selected][floor_support]] = 255
        diagnostics.update(clearance_m=clearance, contact_samples=len(base_contact),
                           reason='calibrated stationary estimate; physical tape validation required')
    except (ValueError, KeyError, TypeError, IndexError, cv2.error) as e:
        diagnostics['reason'] = str(e)
        if blocker and blocker != str(e):
            diagnostics['calibration_blocker'] = blocker
    return diagnostics, native, contact_native


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
    p.add_argument('--urdf', help='robot model for unqualified contact-height diagnostics')
    p.add_argument('--nominal', action='store_true', help='explicitly enable unqualified nominal ToF projection previews')
    p.add_argument('--rgb-diagonal-deg', type=float, default=90.)
    p.add_argument('--tof-diagonal-deg', type=float, default=70.)
    p.add_argument('--baseline-x-m', type=float, default=.04)
    p.add_argument('--nominal-depth-semantics', choices=('radial', 'optical_z'), default='radial')
    p.add_argument('--imgsz', type=int, default=640, help='stationary inference input size; 640 default, 960 tested for box')
    p.add_argument('--reference-clearance-m', type=float, help='independent operator measurement for this captured scene')
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
    urdf_geometry = None
    if a.urdf:
        angle = estimated_view_angle(a.servo_deg)
        if angle is None:
            p.error('--urdf requires a known command within the estimated head mapping')
        from approach_object.urdf_geometry import camera_geometry
        urdf_geometry = camera_geometry(a.urdf, 90-angle)
        urdf_geometry['source_sha256'] = hashlib.sha256(Path(a.urdf).read_bytes()).hexdigest()
        urdf_geometry['pitch_source'] = 'existing image-derived head mapping; not shaft feedback'
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
    records, native_masks, contact_masks = [], [], []
    first_rgb_mask = None
    for index, (folder, meta, rgb, depth) in enumerate(frames):
        rec = dict(frame=folder.name, rgb_stamp=meta['rgb_stamp'], tof_stamp=meta['depth_stamp'],
                   rgb_tof_delta_s=abs(meta['rgb_stamp']-meta['depth_stamp']), target=target,
                   receipt_stamp=meta['receipt_stamp'],
                   acquisition_age_s=meta['receipt_stamp']-min(meta['rgb_stamp'], meta['depth_stamp']),
                   capture_servo_command_deg=meta.get('servo_command_deg'),
                   capture_pose_source=meta.get('pose_source', 'not recorded'),
                   rgb_sha256=hashlib.sha256((folder/'rgb.png').read_bytes()).hexdigest(),
                   depth_sha256=hashlib.sha256((folder/'depth-mm.npy').read_bytes()).hexdigest(),
                   detected=False, clearance_m=None, visible_surface_range_m=None,
                   visibility='lost', reason='')
        inference_started = time.perf_counter()
        mask = np.zeros(rgb.shape[:2], bool)
        native = np.zeros(depth.shape, np.uint8)
        contact_native = np.zeros(depth.shape, np.uint8)
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
            confidence_file = folder / 'confidence.npy'
            confidence = np.load(confidence_file, allow_pickle=False) if confidence_file.is_file() else None
            diagnostics, native, contact_native = analyse_mask(mask, depth, confidence, c, meta, a.servo_deg, nominal, urdf_geometry)
            rec.update(diagnostics)
        except (ValueError, KeyError, TypeError, IndexError, cv2.error) as e:
            rec['reason'] = str(e)
            if not rec['detected']:
                rec['visibility'] = 'ambiguous' if 'ambiguous' in str(e) else 'lost'
        native_masks.append(native)
        contact_masks.append(contact_native)
        rec['processing_latency_s'] = time.perf_counter() - inference_started
        destination = output / f'frame-{index:03d}'
        save_artifacts(destination, folder, rgb, depth, mask, native, contact_native, target)
        rec['artifact_directory'] = destination.name
        records.append(rec)

    elapsed = time.perf_counter() - started
    # Root artifacts retain the first observation for convenient review.
    folder, meta, rgb, depth = frames[0]
    mask = first_rgb_mask if first_rgb_mask is not None else np.zeros(rgb.shape[:2], bool)
    native = native_masks[0]
    contact = contact_masks[0]
    save_artifacts(output, folder, rgb, depth, mask, native, contact, target)

    valid_clearances = [r['clearance_m'] for r in records if r.get('clearance_m') is not None]
    aggregate_clearance = float(np.median(valid_clearances)) if valid_clearances and len(valid_clearances) == len(records) else None
    error = abs(aggregate_clearance - a.reference_clearance_m) if aggregate_clearance is not None and a.reference_clearance_m is not None else None
    blocker = next((r['reason'] for r in records if r.get('clearance_m') is None), None)
    confidence_path = folder/'confidence.npy'
    report = dict(mode='OFFLINE_VISUAL_TARGET_DIAGNOSTIC' if reference is not None else 'OFFLINE_NAMED_OBJECT_FLOOR_CONTACT_DIAGNOSTIC', target=target,
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
                  physical_reference_clearance_m=a.reference_clearance_m,
                  physical_validation=dict(status='not_evaluable' if error is None else ('pass_this_scene' if error <= .05 else 'fail_this_scene'),
                                           absolute_error_m=error, tolerance_m=.05,
                                           qualification_complete=False),
                  contact_samples=int(records[0].get('contact_samples', 0)),
                  depth_spread_m=records[0].get('depth_spread_m'),
                  quality_reason=blocker or 'stationary estimate; physical tape validation required',
                  calibration_provenance={'path':str(Path(a.calibration)), 'flags':{f:c.get(f) for f in FLAGS},
                                           'sha256':hashlib.sha256(Path(a.calibration).read_bytes()).hexdigest(),
                                           'depth_semantics':c.get('depth_semantics'),
                                           'front_edge_m':c.get('front_edge_m')},
                  nominal_geometry=nominal, urdf_geometry=urdf_geometry,
                  geometry_assumptions=[] if nominal is None else ['parallel optical axes', 'centred uncropped RGB pinhole image', 'assumed ToF ' + a.nominal_depth_semantics,
                      'nominal RGB FOV; current crop unvalidated', 'SDK ToF K when available, nominal ToF FOV otherwise', 'unknown distortion assumed zero for preview'],
                  confidence_stream_available=confidence_path.is_file(),
                  confidence_min=c.get('confidence_min'),
                  setup_latency_s=setup_latency,
                  processing_latency_s=elapsed, motion_authority=False,
                  labels='ESTIMATED / UNQUALIFIED',
                  artifacts=['object-mask.png','tof-object-mask.png','contact-sample-mask.png',
                             'annotated-rgb.png','tof-amplitude.png','tof-amplitude-outline.png','masked-depth.png'])
    (output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))


def save_artifacts(output, folder, rgb, depth, mask, native, contact, target):
    output.mkdir(parents=True, exist_ok=True)
    annotated = rgb.copy()
    annotated[mask] = (.55*annotated[mask] + .45*np.array([0, 255, 0])).astype(np.uint8)
    cv2.putText(annotated, f'{target} | ESTIMATED / UNQUALIFIED', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
    cv2.imwrite(str(output/'object-mask.png'), mask.astype(np.uint8)*255)
    cv2.imwrite(str(output/'annotated-rgb.png'), annotated)
    cv2.imwrite(str(output/'tof-object-mask.png'), native)
    cv2.imwrite(str(output/'contact-sample-mask.png'), contact)
    if (folder/'amplitude.npy').is_file():
        amp = np.load(folder/'amplitude.npy', allow_pickle=False).astype(float)
        finite = np.isfinite(amp)
        preview = np.zeros(amp.shape, np.uint8)
        if finite.any():
            lo, hi = np.percentile(amp[finite], [1, 99])
            preview[finite] = np.clip((amp[finite]-lo)/max(hi-lo, 1e-6)*255, 0, 255).astype(np.uint8)
        cv2.imwrite(str(output/'tof-amplitude.png'), preview)
    else:
        cv2.imwrite(str(output/'tof-amplitude.png'), np.zeros(depth.shape, np.uint8))
    amp_outline = cv2.cvtColor(cv2.imread(str(output/'tof-amplitude.png'), cv2.IMREAD_GRAYSCALE), cv2.COLOR_GRAY2BGR)
    if np.any(native):
        contours, _ = cv2.findContours(native, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(amp_outline, contours, -1, (0, 255, 255), 1)
    cv2.imwrite(str(output/'tof-amplitude-outline.png'), amp_outline)
    depth_m = depth.astype(float)*.001
    good = np.isfinite(depth_m) & (depth_m >= .05) & (depth_m <= 4)
    depth_preview = np.zeros(depth.shape, np.uint8)
    depth_preview[good] = np.clip(depth_m[good]/4*255, 1, 255).astype(np.uint8)
    masked = cv2.applyColorMap(depth_preview, cv2.COLORMAP_TURBO)
    masked[native == 0] = (55, 55, 55)
    masked[~good] = (0, 0, 0)
    cv2.putText(amp_outline, 'UNQUALIFIED PROJECTION', (3, 12), cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 0, 255), 1)
    cv2.imwrite(str(output/'tof-amplitude-outline.png'), amp_outline)
    cv2.putText(masked, 'UNQUALIFIED | black=invalid', (3, 12), cv2.FONT_HERSHEY_SIMPLEX, .3, (255, 255, 255), 1)
    cv2.imwrite(str(output/'masked-depth.png'), masked)


if __name__ == '__main__':
    main()
