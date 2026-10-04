"""Latest-frame RGB inference process, independent of 20 Hz supervision."""
import json
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .configuration import load
from .geometry import estimate, Identity

from .model_io import verify_model, image_array

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

class Detector(Node):
    def __init__(self):
        super().__init__('named_object_detector')
        self.c = load(self.declare_parameter('calibration', '').value)
        model = self.declare_parameter('model', '').value
        verify_model(model)
        from ultralytics import YOLO
        self.model = YOLO(model, task='segment')
        self.rgb = self.depth = None
        self.lock = threading.Lock()
        preview = self.declare_parameter('recognition_target', '').value
        if preview not in ('', 'bottle', 'cup'):
            raise ValueError('recognition_target must be bottle or cup')
        self.preview = bool(preview)
        self.request = {'active': self.preview, 'session': 0, 'target': preview, 'view': 0, 'settled_after': 0}
        self.identity = Identity()
        self.key = None
        self.sequence = 0
        self.last_rgb = None
        self.pub = self.create_publisher(String, '/approach_object/observations', 1)
        self.create_subscription(Image, '/rgb_camera/image_raw', self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/tof_camera/depth/image_raw', self.depth_callback, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if not self.preview:
            self.create_subscription(String, '/approach_object/request', self.request_callback, qos)
        self.running = True
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def rgb_callback(self, m):
        with self.lock:
            self.rgb = m

    def depth_callback(self, m):
        with self.lock:
            self.depth = m

    def request_callback(self, m):
        try:
            with self.lock:
                self.request = json.loads(m.data)
        except ValueError:
            pass

    def run(self):
        while self.running and rclpy.ok():
            with self.lock:
                rgb, depth, req = self.rgb, self.depth, dict(self.request)
            if not req.get('active') or rgb is None or stamp(rgb) == self.last_rgb:
                time.sleep(.01)
                continue
            if stamp(rgb) < req.get('settled_after', float('inf')):
                time.sleep(.01)
                continue
            self.last_rgb = stamp(rgb)
            self.sequence += 1
            key = (req['session'], req['view'])
            if self.key != key:
                self.identity, self.key = Identity(), key
            d = dict(req, sequence=self.sequence, stamp=stamp(rgb), identity='', bearing=0.0, clearance=0.0, valid=False, recognized=False, bbox_xyxy=None, reason='')
            try:
                if rgb.header.frame_id != self.c['rgb_frame']:
                    raise ValueError('unexpected RGB coordinate frame')
                frame = image_array(rgb, 'bgr8', 'uint8', 3)
                result = self.model.predict(frame, imgsz=320, conf=.5, verbose=False)[0]
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy().astype(int)
                indices = [i for i, cls in enumerate(classes) if result.names[cls] == req['target']]
                chosen = self.identity.select([boxes[i].tolist() for i in indices], rgb.width, rgb.height)
                if chosen is None:
                    raise ValueError('target absent')
                if result.masks is None:
                    raise ValueError('segmentation mask absent')
                mask = result.masks.data[indices[chosen]].cpu().numpy()
                # Ultralytics masks are letterboxed; undo padding before resize.
                mh, mw = mask.shape
                gain = min(mw/rgb.width, mh/rgb.height)
                px, py = (mw-rgb.width*gain)/2, (mh-rgb.height*gain)/2
                mask = mask[round(py-.1):mh-round(py+.1), round(px-.1):mw-round(px+.1)]
                mask = cv2.resize(mask, (rgb.width, rgb.height), interpolation=cv2.INTER_NEAREST) > .5
                d.update(recognized=True, bbox_xyxy=boxes[indices[chosen]].tolist())
                if self.preview:
                    d['reason'] = 'recognition preview; no distance or command authority'
                    self.get_logger().info(f"{req['target']}: detected at {d['bbox_xyxy']}")
                if depth is None or abs(stamp(depth)-stamp(rgb)) > .08:
                    raise ValueError('depth absent or unsynchronised')
                if depth.header.frame_id != self.c['tof_frame']:
                    raise ValueError('unexpected ToF coordinate frame')
                if self.preview or not self.c['qualified']:
                    raise ValueError('calibration unqualified; recognition only')
                clearance, bearing = estimate(image_array(depth, '32FC1', 'float32'), mask, self.c, req['view'])
                d.update(stamp=min(stamp(rgb), stamp(depth)), identity=str(self.identity.identity), clearance=clearance, bearing=bearing, valid=True)
            except Exception as error:
                d['reason'] = str(error)
            if self.preview and not d['recognized']:
                self.get_logger().info(f"{req['target']}: {d['reason']}")
            d['inference_finished_stamp'] = self.get_clock().now().nanoseconds * 1e-9
            if self.running and rclpy.ok():
                self.pub.publish(String(data=json.dumps(d)))

    def destroy_node(self):
        self.running = False
        self.worker.join(timeout=2)
        return super().destroy_node()

def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
