uint8 LANDED_UNKNOWN=0
uint8 LANDED_ON_GROUND=1
uint8 LANDED_IN_AIR=2
uint8 LANDED_TAKEOFF=3
uint8 LANDED_LANDING=4

bool state_valid
bool connected
bool armed
string mode

bool position_valid
geometry_msgs/PoseStamped local_pose

bool landed_state_valid
uint8 landed_state

# Physical state can recover while DDS control still requires a ground reset.
# Defaults to false for the MAVROS backend.
bool control_inhibited
string control_message

# Backend result (MAVROS service reply or final PX4 DDS ACK).
# Acceptance is separate from actual mode/arming/landing state confirmation.
uint32 last_command_id
bool command_completed
bool command_accepted
string command_message
