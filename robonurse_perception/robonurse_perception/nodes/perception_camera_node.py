#!/usr/bin/env python3
"""perception_camera_node.py

ROS2 node for camera capture + YOLO inference using OpenCV DNN.
Publishes:
 - /camera/image_raw          (sensor_msgs/Image)
 - /people/detections         (vision_msgs/Detection2DArray)
 - /people/detections_json    (std_msgs/String)  -- human-readable JSON summary

Designed to be robust on Jetson Nano. Uses OpenCV DNN with ONNX model.
If OpenCV on your Jetson is built with CUDA, set use_cuda: true in params for acceleration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os

def xywh2xyxy(x, y, w, h):
    x1 = x - w/2
    y1 = y - h/2
    x2 = x + w/2
    y2 = y + h/2
    return [x1, y1, x2, y2]

class PerceptionCameraNode(Node):
    def __init__(self):
        super().__init__('perception_camera_node')

        # parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('model_path', '')
        self.declare_parameter('classes_path', '')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('use_cuda', True)
        self.declare_parameter('publish_image_raw', True)
        self.declare_parameter('publish_detections_json', True)

        self.cam_index = self.get_parameter('camera_index').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.classes_path = str(self.get_parameter('classes_path').value)
        self.input_size = int(self.get_parameter('input_size').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.nms_threshold = float(self.get_parameter('nms_threshold').value)
        self.use_cuda = bool(self.get_parameter('use_cuda').value)
        self.publish_image_raw = bool(self.get_parameter('publish_image_raw').value)
        self.publish_detections_json = bool(self.get_parameter('publish_detections_json').value)

        # publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10) if self.publish_image_raw else None
        self.detections_pub = self.create_publisher(Detection2DArray, '/people/detections', 10)
        self.json_pub = self.create_publisher(String, '/people/detections_json', 10) if self.publish_detections_json else None

        self.bridge = CvBridge()

        # load classes
        self.class_names = []
        if os.path.exists(self.classes_path):
            with open(self.classes_path, 'r') as f:
                for line in f:
                    name = line.strip()
                    if name:
                        self.class_names.append(name)
        else:
            self.get_logger().warn("classes.txt not found at %s" % self.classes_path)

        # load model
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        try:
            self.net = cv2.dnn.readNet(self.model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read ONNX model: {e}")
            raise

        if self.use_cuda:
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                try:
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                except Exception:
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                self.get_logger().info("Using CUDA backend for OpenCV DNN")
            except Exception as e:
                self.get_logger().warn(f"Could not use CUDA backend: {e}. Falling back to CPU.")
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # open camera
        self.cap = cv2.VideoCapture(self.cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera index {self.cam_index}")
            raise RuntimeError("Camera open failed")

        self.get_logger().info("Perception camera node started. Camera opened.")

        # timer for main loop
        self.timer = self.create_timer(1.0 / max(1, self.fps), self.timer_callback)

    def preprocess(self, frame):
        ih, iw = frame.shape[:2]
        new_size = self.input_size
        scale = min(new_size/iw, new_size/ih)
        nw, nh = int(iw*scale), int(ih*scale)
        image_resized = cv2.resize(frame, (nw, nh))
        canvas = np.full((new_size, new_size, 3), 114, dtype=np.uint8)
        dx = (new_size - nw) // 2
        dy = (new_size - nh) // 2
        canvas[dy:dy+nh, dx:dx+nw, :] = image_resized
        blob = cv2.dnn.blobFromImage(canvas, 1/255.0, (new_size, new_size), swapRB=True, crop=False)
        return blob, scale, dx, dy, iw, ih

    def postprocess(self, outputs, scale, dx, dy, orig_w, orig_h):
        if isinstance(outputs, tuple) or isinstance(outputs, list):
            out = outputs[0]
        else:
            out = outputs
        out = np.squeeze(out)
        if out.ndim == 1:
            out = np.expand_dims(out, 0)
        detections = []
        if out.shape[1] < 6:
            self.get_logger().warn("Unexpected output shape from model: %s" % (str(out.shape),))
            return detections

        xywh = out[:, 0:4]
        confs = out[:, 4]
        class_scores = out[:, 5:]
        class_ids = np.argmax(class_scores, axis=1)
        class_confs = class_scores[np.arange(len(class_scores)), class_ids]
        scores = confs * class_confs

        mask = scores >= self.conf_threshold
        if not np.any(mask):
            return []

        xywh = xywh[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]

        boxes = []
        for (x,y,w,h) in xywh:
            x = (x - dx) / scale
            y = (y - dy) / scale
            w = w / scale
            h = h / scale
            x1, y1, x2, y2 = xywh2xyxy(x, y, w, h)
            x1 = max(0, min(orig_w, x1))
            y1 = max(0, min(orig_h, y1))
            x2 = max(0, min(orig_w, x2))
            y2 = max(0, min(orig_h, y2))
            boxes.append([x1, y1, x2 - x1, y2 - y1])

        boxes_xyxy = []
        for b in boxes:
            x, y, w, h = b
            boxes_xyxy.append([x, y, x+w, y+h])
        boxes_xyxy = np.array(boxes_xyxy, dtype=np.float32)

        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(),
            scores.tolist(),
            self.conf_threshold,
            self.nms_threshold
        )
        picks = []
        if len(indices) > 0:
            for idx in np.array(indices).flatten():
                picks.append(int(idx))

        results = []
        for i in picks:
            x, y, w, h = boxes[i]
            score = float(scores[i])
            cid = int(class_ids[i])
            results.append({
                "bbox": [float(x), float(y), float(w), float(h)],
                "score": score,
                "class_id": cid,
                "class_name": self.class_names[cid] if cid < len(self.class_names) else str(cid)
            })
        return results

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Camera frame not read")
            return

        t0 = time.time()

        if self.publish_image_raw and self.image_pub is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'camera_link'
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f"Failed to publish image: {e}")

        blob, scale, dx, dy, iw, ih = self.preprocess(frame)
        self.net.setInput(blob)
        try:
            outputs = self.net.forward()
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return

        detections = self.postprocess(outputs, scale, dx, dy, iw, ih)

        det_msg = Detection2DArray()
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.header.frame_id = 'camera_link'
        for det in detections:
            bbox = det['bbox']
            score = det['score']
            cid = det['class_id']
            cls_name = det['class_name']
            d = Detection2D()
            cx = float(bbox[0]) + float(bbox[2]) / 2.0
            cy = float(bbox[1]) + float(bbox[3]) / 2.0
            d.bbox.center.x = cx
            d.bbox.center.y = cy
            d.bbox.size_x = float(bbox[2])
            d.bbox.size_y = float(bbox[3])
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_name)
            hyp.hypothesis.score = float(score)
            d.results.append(hyp)
            det_msg.detections.append(d)

        self.detections_pub.publish(det_msg)

        if self.publish_detections_json and self.json_pub is not None:
            summary = []
            for det in detections:
                summary.append({
                    "class": det.get("class_name"),
                    "score": det.get("score"),
                    "bbox": det.get("bbox")
                })
            js = {"timestamp": time.time(), "detections": summary}
            sm = String()
            sm.data = json.dumps(js)
            self.json_pub.publish(sm)

        t1 = time.time()
        if int(time.time()) % 5 == 0:
            self.get_logger().info(f"Inference + publish latency: {(t1 - t0)*1000:.1f} ms, detections: {len(detections)}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down perception node")
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
