"""Read camera mounting geometry from URDF; never infer physical servo feedback."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array(
        [[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def origin(element):
    result = np.eye(4)
    if element is not None:
        xyz = np.array(list(map(float, element.get('xyz', '0 0 0').split())))
        rpy = np.array(list(map(float, element.get('rpy', '0 0 0').split())))
        if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
            raise ValueError('invalid URDF origin')
        result[:3, 3], result[:3, :3] = xyz, rotation(rpy)
    return result


def link_pose(root, link, joint_positions, base='base_footprint'):
    """URDF parent-to-child chain including explicit revolute joint positions."""
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain, seen = [], set()
    while link != base:
        if link in seen or link not in joints:
            raise ValueError('camera link has no acyclic path to base')
        seen.add(link)
        joint = joints[link]
        pose = origin(joint.find('origin'))
        kind = joint.get('type')
        if kind != 'fixed':
            name = joint.get('name')
            if kind not in ('revolute', 'continuous') or name not in joint_positions:
                raise ValueError('joint position required: ' + str(name))
            angle = float(joint_positions[name])
            if not np.isfinite(angle):
                raise ValueError('nonfinite joint position')
            limit = joint.find('limit')
            if kind == 'revolute' and limit is not None and not float(limit.get('lower')) <= angle <= float(limit.get('upper')):
                raise ValueError('joint outside URDF limits')
            axis_el = joint.find('axis')
            axis = np.array(list(map(float, axis_el.get('xyz', '1 0 0').split()))) if axis_el is not None else np.array([1., 0., 0.])
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) == 0:
                raise ValueError('invalid joint axis')
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            turn = np.eye(4)
            turn[:3, :3] = np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew)
            pose = pose @ turn
        chain.append(pose)
        link = joint.find('parent').get('link')
    result = np.eye(4)
    for pose in reversed(chain):
        result = result @ pose
    return result


def camera_geometry(path, pitch_down_deg):
    root = ET.parse(Path(path)).getroot()
    pitch = np.deg2rad(float(pitch_down_deg))
    # This model has body frames, not optical frames. Record this convention as
    # an assumption rather than pretending the URDF defines lens centres/axes.
    optical_to_body = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    poses = {}
    for name in ('tof_camera', 'rgb_camera'):
        pose = link_pose(root, name, {'neck_tilt_joint': pitch})
        pose[:3, :3] = pose[:3, :3] @ optical_to_body
        poses[name] = pose
    fronts = []
    for name in ('lower_waffle_plate', 'upper_waffle_plate'):
        link = root.find("link[@name='" + name + "']")
        collision = link.find('collision')
        box = collision.find('geometry/box')
        size = np.array(list(map(float, box.get('size').split())))
        pose = link_pose(root, name, {}) @ origin(collision.find('origin'))
        for sign in np.ndindex(2, 2, 2):
            corner = (np.array(sign)*2-1)*size/2
            fronts.append(float((pose @ np.r_[corner, 1])[0]))
    return {'tof_to_base': poses['tof_camera'].tolist(),
            'rgb_to_base': poses['rgb_camera'].tolist(),
            'tof_to_rgb': (np.linalg.inv(poses['rgb_camera']) @ poses['tof_camera']).tolist(),
            'front_edge_m': max(fronts), 'pitch_down_deg': float(pitch_down_deg),
            'assumptions': ['camera body origin represents lens centre',
                            'optical forward is body +X; optical right is body -Y',
                            'pitch is supplied separately, not servo feedback',
                            'robot front datum is maximum X of waffle plates'],
            'physical_accuracy_verified': False, 'motion_authority': False}
