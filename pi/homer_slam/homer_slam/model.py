"""Read the canonical URDF; display pose is explicitly illustrative, not feedback."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET


def model_geometry(path):
    root = ET.parse(path).getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}

    def xyz(link):
        if link == 'base_footprint':
            return [0., 0., 0.]
        joint = joints[link]
        if joint.get('type') != 'fixed' and link not in ('left_wheel_link', 'right_wheel_link'):
            raise ValueError('Sensor/base chain must be fixed')
        origin = joint.find('origin')
        if origin is not None and any(abs(float(v)) > 1e-9 for v in origin.get('rpy', '0 0 0').split()):
            raise ValueError('Rotated base geometry requires explicit transform composition')
        offset = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0., 0., 0.]
        return [a+b for a, b in zip(xyz(joint.find('parent').get('link')), offset)]

    wheels = [xyz(name) for name in ('left_wheel_link', 'right_wheel_link')]
    center = [(a+b)/2 for a, b in zip(*wheels)]
    center[2] = 0.
    laser = [a-b for a,b in zip(xyz('rplidar_a1'), center)]
    bounds = []
    for name in ('lower_waffle_plate', 'left_wheel_link', 'right_wheel_link', 'front_caster_ball', 'rear_caster_ball'):
        link = root.find("link[@name='%s']" % name)
        collision = link.find('collision')
        shape = collision.find('geometry')
        origin = collision.find('origin')
        local = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0.,0.,0.]
        pos = [a+b-c for a,b,c in zip(xyz(name), local, center)]
        if shape.find('box') is not None:
            sx,sy,_ = map(float, shape.find('box').get('size').split())
            hx,hy = sx/2,sy/2
        elif shape.find('cylinder') is not None:
            cylinder = shape.find('cylinder')
            hx,hy = float(cylinder.get('radius')),float(cylinder.get('length'))/2
        else:
            hx = hy = float(shape.find('sphere').get('radius'))
        bounds.append((pos[0]-hx,pos[0]+hx,pos[1]-hy,pos[1]+hy))
    xmin,xmax = min(v[0] for v in bounds),max(v[1] for v in bounds)
    ymin,ymax = min(v[2] for v in bounds),max(v[3] for v in bounds)
    if not all(math.isfinite(v) for v in laser + [xmin,xmax,ymin,ymax]):
        raise ValueError('Nonfinite URDF geometry')
    return {'verified': True, 'measurement_source': 'User-selected canonical URDF: ' + str(path),
            'laser': dict(zip(('x','y','z','roll','pitch','yaw'), laser+[0.,0.,0.])),
            'footprint': [[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]],
            'model_center': center}


def display_description(path):
    root = ET.parse(path).getroot()
    for joint in root.findall('joint'):
        joint.set('type', 'fixed')
        for tag in ('axis', 'limit', 'mimic', 'dynamics'):
            for item in joint.findall(tag):
                joint.remove(item)
    return ET.tostring(root, encoding='unicode')
