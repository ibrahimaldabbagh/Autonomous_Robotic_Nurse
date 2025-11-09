# robonurse_interfaces/msg/Detection.msg
# Single detection from YOLO/other models

std_msgs/Header header

# Class of the detected object (e.g., "patient", "nurse", "medicine_box")
string label 

# Bounding box in normalized coordinates (0.0 to 1.0) or pixels
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Confidence score
float32 confidence

# Distance to the object in meters (from depth camera)
float32 distance_m