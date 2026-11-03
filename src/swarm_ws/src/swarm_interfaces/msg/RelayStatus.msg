uint8 IDLE=0
uint8 ACCEPTED=1
uint8 PLANNING=2
uint8 EXECUTING=3
uint8 ARRIVED=4
uint8 FAILED=5
uint8 CANCELED=6

std_msgs/Header header
uint32 task_id
string robot_id
uint8 state
string reason
bool pose_valid
geometry_msgs/PoseStamped current_pose
