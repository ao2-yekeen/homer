#!/usr/bin/env python3
"""Read existing ROS publishers; save a timestamp-matched pair. No publishers or motion."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from approach_object.model_io import image_array


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('stationary_rgb_tof_capture')
    rgb, depth = deque(maxlen=20), deque(maxlen=20)
    node.create_subscription(Image, '/rgb_camera/image_raw', lambda m: rgb.append(m), qos_profile_sensor_data)
    node.create_subscription(Image, '/tof_camera/depth/image_raw', lambda m: depth.append(m), qos_profile_sensor_data)
    deadline = time.monotonic() + 20
    pair = None
    try:
        while time.monotonic() < deadline and pair is None:
            rclpy.spin_once(node, timeout_sec=.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            candidates = [(abs(stamp(r)-stamp(d)), r, d) for r in rgb for d in depth
                          if 0 <= now-stamp(r) <= .5 and 0 <= now-stamp(d) <= .5]
            if candidates:
                delta, r, d = min(candidates, key=lambda x: x[0])
                if delta <= .08:
                    pair = r, d, delta, now
        if pair is None:
            raise RuntimeError('no fresh RGB/ToF pair within 80 ms; check publishers/timestamps')
        r, d, delta, now = pair
        if not cv2.imwrite(str(root / 'rgb.png'), image_array(r, 'bgr8', 'uint8', 3)):
            raise RuntimeError('failed to save RGB')
        np.save(root / 'depth-mm.npy', image_array(d, '32FC1', 'float32'))
        metadata = dict(rgb_stamp=stamp(r), depth_stamp=stamp(d), delta_s=delta,
                        receipt_stamp=now, rgb_frame=r.header.frame_id, tof_frame=d.header.frame_id,
                        depth_units='publisher millimetres; physical interpretation requires validation',
                        motion_commanded=False)
        (root / 'capture.json').write_text(json.dumps(metadata, indent=2)+'\n')
        print(json.dumps(metadata))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
