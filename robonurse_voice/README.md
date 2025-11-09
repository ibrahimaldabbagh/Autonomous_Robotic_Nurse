# robonurse_perception

RoboNurse perception ROS2 package.

Maintainer: Ibrahim Aldabbagh <eng.ibrahim.aldabbagh@gmail.com>

This package provides a single node `perception_camera_node` that:
- captures frames from a V4L2 camera (/dev/videoN),
- runs a YOLO ONNX model using OpenCV DNN (CUDA backend if available),
- publishes `/camera/image_raw` (sensor_msgs/Image),
- publishes `/people/detections` (vision_msgs/Detection2DArray),
- publishes `/people/detections_json` (std_msgs/String) with a JSON summary.

## Install / Use

1. Place your ONNX model at `models/robonurse_yolo.onnx` and set absolute paths in `config/camera_params.yaml`.
2. Build in a ROS2 Humble colcon workspace:
   ```
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch robonurse_perception perception.launch.py
   ```
3. Ensure OpenCV on Jetson has CUDA backend for best performance. See notes below.
