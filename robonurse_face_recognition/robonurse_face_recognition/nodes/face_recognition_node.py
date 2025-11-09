#!/usr/bin/env python3
"""face_recognition_node.py

Subscribes:
  - /camera/image_raw (sensor_msgs/Image)
  - /people/detections (vision_msgs/Detection2DArray)

Publishes:
  - /people/identity (std_msgs/String) with JSON: {"id": "...", "confidence": 0.92}

Uses ONNX embedding model (ArcFace-style) via onnxruntime and encrypted DB (Fernet).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import cv2
import os
import json
import time
from cryptography.fernet import Fernet
import base64

def l2_normalize(x, axis=1, epsilon=1e-10):
    norm = np.linalg.norm(x, ord=2, axis=axis, keepdims=True)
    return x / (norm + epsilon)

def cosine_distance(a, b):
    # a: (d,), b: (n, d)
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b, axis=1, keepdims=True)
    return 1.0 - np.dot(b, a)

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # params
        self.declare_parameter('model_path', '')
        self.declare_parameter('embedding_size', 512)
        self.declare_parameter('crop_size', 112)
        self.declare_parameter('match_threshold', 0.38)
        self.declare_parameter('db_path', '')
        self.declare_parameter('key_path', '')
        self.declare_parameter('use_cuda', True)

        self.model_path = str(self.get_parameter('model_path').value)
        self.emb_size = int(self.get_parameter('embedding_size').value)
        self.crop_size = int(self.get_parameter('crop_size').value)
        self.match_threshold = float(self.get_parameter('match_threshold').value)
        self.db_path = str(self.get_parameter('db_path').value)
        self.key_path = str(self.get_parameter('key_path').value)
        self.use_cuda = bool(self.get_parameter('use_cuda').value)

        # load onnx model
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"ONNX model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)
        providers = ['CPUExecutionProvider']
        if self.use_cuda:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        self.session = ort.InferenceSession(self.model_path, providers=providers)
        self.get_logger().info(f"ONNX model loaded: {self.model_path} with providers {self.session.get_providers()}")

        # load or create key
        if not os.path.exists(self.key_path):
            # create key and save
            key = Fernet.generate_key()
            os.makedirs(os.path.dirname(self.key_path), exist_ok=True)
            with open(self.key_path, 'wb') as f:
                f.write(key)
            self.get_logger().info(f"New encryption key generated at {self.key_path}")
        else:
            with open(self.key_path, 'rb') as f:
                key = f.read()
        self.fernet = Fernet(key)

        # load DB
        self.db = {}  # id -> embedding numpy array (as list)
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'rb') as f:
                    enc = f.read()
                dec = self.fernet.decrypt(enc)
                db_json = json.loads(dec.decode('utf-8'))
                for k, v in db_json.items():
                    self.db[k] = np.array(v, dtype=np.float32)
                self.get_logger().info(f"Loaded face DB with {len(self.db)} identities")
            except Exception as e:
                self.get_logger().error(f"Failed to load DB: {e}")
        else:
            # ensure directory
            os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
            self.save_db()

        # ROS pubs/subs
        self.bridge = CvBridge()
        self.image = None
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Detection2DArray, '/people/detections', self.detections_cb, 10)
        self.pub = self.create_publisher(String, '/people/identity', 10)

        self.get_logger().info('face_recognition_node started')

    def save_db(self):
        db_json = {k: v.tolist() for k, v in self.db.items()}
        data = json.dumps(db_json).encode('utf-8')
        enc = self.fernet.encrypt(data)
        with open(self.db_path, 'wb') as f:
            f.write(enc)
        self.get_logger().info(f"DB saved ({len(self.db)} identities)")

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image = cv_image
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

    def detections_cb(self, msg: Detection2DArray):
        if self.image is None:
            return
        img = self.image.copy()
        for det in msg.detections:
            if not det.results:
                continue
            cls = det.results[0].hypothesis.class_id
            # only process 'person' class (or use classes)
            if str(cls).lower() not in ['person', 'patient', 'face']:
                continue
            # bbox center/size in pixels
            cx = det.bbox.center.x
            cy = det.bbox.center.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1 = int(max(0, cx - w/2))
            y1 = int(max(0, cy - h/2))
            x2 = int(min(img.shape[1]-1, cx + w/2))
            y2 = int(min(img.shape[0]-1, cy + h/2))
            face = img[y1:y2, x1:x2]
            if face.size == 0:
                continue
            emb = self.compute_embedding(face)
            if emb is None:
                continue
            # match
            ids = list(self.db.keys())
            if ids:
                arr = np.stack([self.db[i] for i in ids], axis=0)  # (N, D)
                dists = cosine_distance(emb, arr)  # 1 - cosine_sim
                min_idx = int(np.argmin(dists))
                best_id = ids[min_idx]
                best_dist = float(dists[min_idx])
                # lower distance -> more similar
                if best_dist <= self.match_threshold:
                    # publish identity
                    msg = String()
                    payload = {'id': best_id, 'distance': best_dist}
                    msg.data = json.dumps(payload)
                    self.pub.publish(msg)
                    self.get_logger().info(f"Recognized {best_id} dist={best_dist:.3f}")
                else:
                    # unknown
                    msg = String()
                    msg.data = json.dumps({'id': 'unknown', 'distance': best_dist})
                    self.pub.publish(msg)
            else:
                msg = String()
                msg.data = json.dumps({'id': 'unknown', 'distance': None})
                self.pub.publish(msg)

    def compute_embedding(self, face_bgr):
        try:
            # align/resize to crop_size
            face = cv2.resize(face_bgr, (self.crop_size, self.crop_size))
            # convert to RGB and float32
            face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            face = face.astype(np.float32)
            # normalize: [0,255] -> [-1,1] or model specific. Many ArcFace models use (x-127.5)/128
            face = (face - 127.5) / 128.0
            # transpose to CHW
            inp = np.transpose(face, (2,0,1)).astype(np.float32)
            inp = np.expand_dims(inp, axis=0)
            # run onnx
            ort_inputs = {self.session.get_inputs()[0].name: inp}
            emb = self.session.run(None, ort_inputs)[0]
            emb = np.squeeze(emb)
            emb = emb.astype(np.float32)
            emb = emb / np.linalg.norm(emb)
            return emb
        except Exception as e:
            self.get_logger().error(f"Embedding computation failed: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down face_recognition_node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
