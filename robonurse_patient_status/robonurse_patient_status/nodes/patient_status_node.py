#!/usr/bin/env python3
"""patient_status_node.py

Subscribes to /people/detections (vision_msgs/Detection2DArray) and publishes /patient/status (std_msgs/String).
Implements:
  - confidence thresholding
  - short temporal aggregation (sliding window)
  - exponential smoothing to avoid flicker
  - priority-based conflict resolution
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import time
from collections import deque, defaultdict
import yaml

class PatientStatusNode(Node):
    def __init__(self):
        super().__init__('patient_status_node')

        # parameters
        self.declare_parameter('status_map', {})
        self.declare_parameter('class_conf_threshold', 0.3)
        self.declare_parameter('aggregate_window_sec', 2.0)
        self.declare_parameter('publish_min_interval', 0.5)
        self.declare_parameter('smoothing_alpha', 0.6)
        self.declare_parameter('priority_order', [])

        self.status_map = self.get_parameter('status_map').get_parameter_value().string_value or {}
        # When passing dict via yaml, rclpy gives it as string. Try to load from parameter server file if needed.
        if isinstance(self.status_map, str) and self.status_map.strip():
            try:
                self.status_map = yaml.safe_load(self.status_map)
            except Exception:
                self.status_map = {}
        self.class_conf_threshold = float(self.get_parameter('class_conf_threshold').value)
        self.aggregate_window = float(self.get_parameter('aggregate_window_sec').value)
        self.publish_min_interval = float(self.get_parameter('publish_min_interval').value)
        self.alpha = float(self.get_parameter('smoothing_alpha').value)
        self.priority_order = list(self.get_parameter('priority_order').value)

        # internal buffers
        self.detection_buffer = deque()  # elements: (timestamp, class_name, score)
        self.last_publish_time = 0.0
        self.smoothed_scores = defaultdict(float)  # map label -> smoothed score

        # publisher
        self.status_pub = self.create_publisher(String, '/patient/status', 10)

        # subscriber
        self.create_subscription(Detection2DArray, '/people/detections', self.detections_cb, 10)

        self.get_logger().info("patient_status_node started")

    def detections_cb(self, msg: Detection2DArray):
        now = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9
        # extract detections
        for det in msg.detections:
            if not det.results:
                continue
            # object class is stored in hypothesis.class_id (string)
            cls = det.results[0].hypothesis.class_id
            score = float(det.results[0].hypothesis.score)
            if score >= self.class_conf_threshold:
                self.detection_buffer.append((now, str(cls), score))
        # remove old
        cutoff = now - self.aggregate_window
        while self.detection_buffer and self.detection_buffer[0][0] < cutoff:
            self.detection_buffer.popleft()

        # aggregate counts/scores
        agg = {}
        for _, cls, score in self.detection_buffer:
            agg.setdefault(cls, []).append(score)

        avg_scores = {cls: sum(lst)/len(lst) for cls, lst in agg.items()}

        # map classes to statuses
        status_scores = {}
        for cls, avg in avg_scores.items():
            label = self.status_map.get(cls, None)
            if label:
                status_scores[label] = max(status_scores.get(label, 0.0), avg)

        # apply smoothing (EMA)
        for label, score in status_scores.items():
            prev = self.smoothed_scores.get(label, 0.0)
            new = self.alpha*score + (1.0 - self.alpha)*prev
            self.smoothed_scores[label] = new

        # decay labels not present (small decay)
        for label in list(self.smoothed_scores.keys()):
            if label not in status_scores:
                self.smoothed_scores[label] = (1.0 - self.alpha)*self.smoothed_scores[label]

        # select label to publish based on priority_order and highest smoothed score
        chosen = None
        # first check priority order
        for p in self.priority_order:
            if self.smoothed_scores.get(p, 0.0) > 0.01:
                chosen = p
                break
        if not chosen:
            # pick highest score
            if self.smoothed_scores:
                chosen = max(self.smoothed_scores.items(), key=lambda kv: kv[1])[0]

        # enforce min publish interval
        nowt = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1e9
        if chosen and (nowt - self.last_publish_time) >= self.publish_min_interval:
            msg = String()
            msg.data = str(chosen)
            self.status_pub.publish(msg)
            self.last_publish_time = nowt
            self.get_logger().info(f"Published patient status: {chosen}")

def main(args=None):
    rclpy.init(args=args)
    node = PatientStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down patient_status_node")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
