# Original goal frame and generation time. valid_for limits acceptance/start from this stamp;
# it does not impose a deadline on an already started flight task.
std_msgs/Header header
uint32 task_id
string robot_id
geometry_msgs/Pose target
builtin_interfaces/Duration valid_for
