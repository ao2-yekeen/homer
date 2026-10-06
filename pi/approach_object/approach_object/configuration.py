import math
import yaml
from .geometry import transform
import numpy as np

def load(path):
    with open(path) as stream:
        c = yaml.safe_load(stream)
    c['qualified'] = False
    if c.get('enable_motion') is not True:
        return c
    checks = ('intrinsics_measured', 'alignment_measured', 'poses_measured', 'depth_validated', 'buffer_delay_measured', 'drive_measured', 'braking_measured', 'lidar_pose_measured', 'stationary_distance_verified', 'model_benchmarked')
    if not all(c.get(k) is True for k in checks):
        raise ValueError('qualification measurements missing')
    for key in ('forward_command', 'turn_command', 'speed_mps', 'braking_m', 'uncertainty_m', 'front_edge_m', 'lidar_front_limit_m', 'lidar_half_width_m', 'turn_clearance_radius_m', 'latency_p99_s', 'capture_delay_bound_s'):
        if not isinstance(c.get(key), (int, float)) or not math.isfinite(c[key]):
            raise ValueError('missing finite measurement: ' + key)
    if not 0 < c['speed_mps'] <= .05 or not 0 < abs(c['forward_command']) <= 5 or not 0 < abs(c['turn_command']) <= 5:
        raise ValueError('invalid measured drive mapping')
    if min(c['braking_m'], c['uncertainty_m'], c['capture_delay_bound_s']) < 0 or c['latency_p99_s'] + c['capture_delay_bound_s'] > .5:
        raise ValueError('latency or stopping margin invalid')
    if c['latency_p99_s'] < 0 or c['front_edge_m'] <= 0 or c['lidar_front_limit_m'] <= c['front_edge_m'] + .30 or c['lidar_half_width_m'] <= 0 or c['turn_clearance_radius_m'] < max(c['front_edge_m'], c['lidar_half_width_m']):
        raise ValueError('invalid footprint/corridor measurements')
    if c['braking_m'] + c['uncertainty_m'] > .05:
        raise ValueError('stopping margin exceeds 30 +/-5 cm acceptance; lower latency/speed')
    if c['uncertainty_m'] < c['speed_mps'] * (.5 + .25 + .05):
        raise ValueError('uncertainty must cover 500 ms perception age, 250 ms lease and 50 ms supervision')
    for camera in ('rgb', 'tof'):
        k = np.asarray(c[camera + '_K'], float)
        d = np.asarray(c[camera + '_D'], float)
        if k.shape != (3,3) or not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0 or not np.isfinite(d).all() or d.size not in (4,5,8,12,14):
            raise ValueError('invalid intrinsics/distortion')
    if not 0 < c['depth_min_m'] < c['depth_max_m'] or c['min_samples'] < 4 or not 0 <= c['max_floor_fraction'] < 1 or c['max_object_depth_span_m'] <= 0:
        raise ValueError('invalid depth filtering bounds')
    if len(c['views']) != 2 or len(c['settle_s']) != 2 or any(x <= 0 for x in c['settle_s']):
        raise ValueError('two measured views/settling times required')
    if any(not 80 <= v['servo_deg'] <= 150 for v in c['views']):
        raise ValueError('tilt outside hardware limits')
    for m in [c['tof_to_rgb'], c['lidar_to_base']] + [v['tof_to_base'] for v in c['views']]:
        transform(np.zeros((1, 3)), m)
    if c['depth_semantics'] not in ('radial', 'optical_z'):
        raise ValueError('depth interpretation unvalidated')
    c['qualified'] = True
    return c
