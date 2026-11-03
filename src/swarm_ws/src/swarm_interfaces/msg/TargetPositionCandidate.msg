# A LiDAR return projecting into the wheel camera's selected detection.
# This is a candidate, not a verified target center or an arrival indication.
std_msgs/Header header
builtin_interfaces/Time detection_stamp
string target_id
string class_id
bool valid
string reason
geometry_msgs/Point position
uint32 supporting_points
float32 range_spread_m
