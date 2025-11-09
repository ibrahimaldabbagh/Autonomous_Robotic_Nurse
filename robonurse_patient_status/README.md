# robonurse_patient_status

Converts `/people/detections` (vision_msgs/Detection2DArray) into a concise `/patient/status` topic (std_msgs/String).

Features:
- Confidence filtering
- Short temporal aggregation and exponential smoothing to reduce flicker
- Priority rules for conflicting detections

Maintainer: Ibrahim Aldabbagh <eng.ibrahim.aldabbagh@gmail.com>
