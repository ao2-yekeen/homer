"""Read factory lens controls from the existing ToF owner; no device access."""
import math


def read_camera_metadata(camera, controls):
    """SDK intrinsics use hundredths of pixels, as in its point-cloud example."""
    info = camera.getCameraInfo()
    width, height = int(info.width), int(info.height)
    raw = {key: int(camera.getControl(getattr(controls, 'INTRINSIC_' + key.upper())))
           for key in ('fx', 'fy', 'cx', 'cy')}
    values = {key: value / 100. for key, value in raw.items()}
    if (width <= 0 or height <= 0 or not all(math.isfinite(v) for v in values.values())
            or values['fx'] <= 0 or values['fy'] <= 0
            or not 0 <= values['cx'] < width or not 0 <= values['cy'] < height):
        raise ValueError('SDK factory intrinsics are missing or invalid')
    return dict(source='Arducam SDK INTRINSIC_* controls / 100',
                sdk_intrinsics_raw=raw, width=width, height=height,
                K=[[values['fx'], 0., values['cx']], [0., values['fy'], values['cy']], [0., 0., 1.]],
                distortion=None, factory_intrinsics_available=True,
                physical_accuracy_verified=False,
                depth_units='millimetres', depth_semantics='unvalidated',
                range_control_mm=int(camera.getControl(controls.RANGE)),
                motion_authority=False)
