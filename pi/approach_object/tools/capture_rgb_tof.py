#!/usr/bin/env python3
"""Read existing ROS publishers; save a timestamp-matched pair. No publishers or motion."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
# The uploaded capture bundle includes ros_runtime beside this script. For a
# checkout, find the same shared module in pi/.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from ros_runtime import ensure_ros_environment

ensure_ros_environment()
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from approach_object.model_io import image_array


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--auxiliary', action='store_true', help='Require ToF amplitude and confidence from the same frame')
    parser.add_argument('--frames', type=int, default=1, help='Save distinct matched pairs; multiple pairs go in frame-NNN directories')
    parser.add_argument('--servo-deg', type=float, help='known head command held during capture; records it without commanding motion')
    parser.add_argument('--usb-camera', default='/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0',
                        help='extra USB camera device to capture directly')
    args = parser.parse_args()
    if not 1 <= args.frames <= 100:
        parser.error('--frames must be between 1 and 100')
    if args.servo_deg is not None and (not math.isfinite(args.servo_deg) or not 80 <= args.servo_deg <= 150):
        parser.error('--servo-deg must be a known command within 80–150 degrees')
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    if args.frames > 1 and any(root.glob('frame-*')):
        parser.error('burst output already contains frames; choose a fresh output directory')
    rclpy.init()
    node = rclpy.create_node('stationary_rgb_tof_capture')
    rgb, depth = deque(maxlen=20), deque(maxlen=20)
    amplitude, confidence = deque(maxlen=20), deque(maxlen=20)
    node.create_subscription(Image, '/rgb_camera/image_raw', lambda m: rgb.append(m), qos_profile_sensor_data)
    node.create_subscription(Image, '/tof_camera/depth/image_raw', lambda m: depth.append(m), qos_profile_sensor_data)
    if args.auxiliary:
        node.create_subscription(Image, '/tof_camera/amplitude/image_raw', lambda m: amplitude.append(m), qos_profile_sensor_data)
        node.create_subscription(Image, '/tof_camera/confidence/image_raw', lambda m: confidence.append(m), qos_profile_sensor_data)
    deadline = time.monotonic() + 20 + args.frames
    captures = []
    last_rgb = last_depth = float('-inf')
    usb = None
    try:
        # Keep the auxiliary camera open across the burst. Reopening it for
        # every pair adds startup delays that can expire target tracking.
        usb = cv2.VideoCapture(args.usb_camera, cv2.CAP_V4L2)
        if not usb.isOpened():
            raise RuntimeError(f'cannot open extra USB camera: {args.usb_camera}')
        # This UVC camera stalls in its default uncompressed mode on the Pi.
        usb.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        while len(captures) < args.frames:
            pair = None
            while time.monotonic() < deadline and pair is None:
                rclpy.spin_once(node, timeout_sec=.1)
                now = node.get_clock().now().nanoseconds * 1e-9
                candidates = [(abs(stamp(r)-stamp(d)), r, d) for r in rgb for d in depth
                              if stamp(r) > last_rgb and stamp(d) > last_depth
                              and 0 <= now-stamp(r) <= .5 and 0 <= now-stamp(d) <= .5]
                if candidates:
                    delta, r, d = min(candidates, key=lambda x: x[0])
                    if delta <= .08:
                        if args.auxiliary and not all(any(stamp(x) == stamp(d) for x in stream) for stream in (amplitude, confidence)):
                            continue
                        pair = r, d, delta, now
            if pair is None:
                raise RuntimeError('no fresh RGB/ToF pair within 80 ms; check publishers/timestamps')
            r, d, delta, now = pair
            last_rgb, last_depth = stamp(r), stamp(d)
            destination = root if args.frames == 1 else root / f'frame-{len(captures):03d}'
            destination.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(destination / 'rgb.png'), image_array(r, 'bgr8', 'uint8', 3)):
                raise RuntimeError('failed to save RGB')
            ok, usb_frame = usb.read()
            if not ok or usb_frame is None:
                raise RuntimeError(f'cannot read frame from extra USB camera: {args.usb_camera}')
            if not cv2.imwrite(str(destination / 'usb-camera.png'), usb_frame):
                raise RuntimeError('failed to save extra USB camera frame')
            metadata_usb_camera = dict(device=args.usb_camera, model='Realtek Generic USB Camera',
                                       capture_unix_s=time.time(), width=int(usb_frame.shape[1]),
                                       height=int(usb_frame.shape[0]))
            np.save(destination / 'depth-mm.npy', image_array(d, '32FC1', 'float32'))
            if args.auxiliary:
                for name, stream in [('amplitude', amplitude), ('confidence', confidence)]:
                    m = next(x for x in stream if stamp(x) == stamp(d))
                    values = image_array(m, '32FC1', 'float32')
                    np.save(destination / (name+'.npy'), values)
                    valid = values[np.isfinite(values)]
                    if valid.size:
                        lo, hi = np.percentile(valid, [1, 99])
                        preview = np.nan_to_num(np.clip((values-lo)/max(hi-lo, 1e-6)*255, 0, 255)).astype(np.uint8)
                        cv2.imwrite(str(destination / (name+'.png')), preview)
            metadata = dict(rgb_stamp=stamp(r), depth_stamp=stamp(d), delta_s=delta,
                            receipt_stamp=now, rgb_frame=r.header.frame_id, tof_frame=d.header.frame_id,
                            depth_units='publisher millimetres; physical interpretation requires validation',
                            motion_commanded=False)
            metadata['servo_command_deg'] = args.servo_deg
            metadata['pose_source'] = 'caller declared held command; no shaft feedback'
            metadata['extra_usb_camera'] = metadata_usb_camera
            factory_file = Path.home() / '.local/state/homer/tof-camera-metadata.json'
            if factory_file.is_file():
                try:
                    metadata['tof_camera_metadata'] = json.loads(factory_file.read_text())
                except (OSError, ValueError) as error:
                    metadata['tof_metadata_error'] = str(error)
            (destination / 'capture.json').write_text(json.dumps(metadata, indent=2)+'\n')
            captures.append(metadata)
        if args.frames > 1:
            (root / 'burst.json').write_text(json.dumps(captures, indent=2)+'\n')
        report = captures[0] if args.frames == 1 else dict(frames=len(captures), first=captures[0], last=captures[-1])
        print(json.dumps(report))
    finally:
        if usb is not None:
            usb.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
