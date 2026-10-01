# OLDEST contributing sensor timestamp; frame is the configured robot base frame.
std_msgs/Header header
bool detected
# Polar target location in base frame (+x forward, +y left), radians and metres.
float64 bearing
float64 distance
# Propagated camera quality heuristic, NOT a calibrated probability.
float64 confidence
