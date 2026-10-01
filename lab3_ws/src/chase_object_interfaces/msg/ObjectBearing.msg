# Original image timestamp and CAMERA OPTICAL frame (+x right, +y down, +z forward).
std_msgs/Header header
# False means no usable object. Numeric fields must be ignored in that case.
bool detected
# Horizontal angles in radians, positive LEFT of optical +z.
float64 bearing
float64 angle_min
float64 angle_max
# Blob fill ratio (0..1), a quality heuristic, NOT a calibrated probability.
float64 confidence
