# A message to represent the status of the navigation system

uint8 REQUEST=0
uint8 REJECT=1
uint8 ACCEPT=2   
uint8 FEEDBACK=3
uint8 FINISHED=4
uint8 FAILED=5
uint8 CANCEL=6
uint8 CANCELLED=7
uint8 ERROR=8
uint8 PAUSE=9
uint8 RESUME=10
uint8 PAUSED=11
uint8 RESUMED=12

uint8 type

# Message header
std_msgs/Header header

# Message sequence number
int64 seq

# A unique identifier for nav user
string user_id
string nav_current_user_id

# The current goal reuested if status == REQUEST
# Inform the current goal reuested otherwise
nav_msgs/Goals goals

# Message to provide additional information about the status
string status_message

# Feedback extra information ===================================================== 
# if status == FEEDBACK || FINISHED || PREEMPTED || FAILED || CANCELED

# Current robot position
geometry_msgs/PoseStamped current_pose

# Total time navigating towards this goal
builtin_interfaces/Duration navigation_time

# Estimated remaining time to reach the goal
builtin_interfaces/Duration estimated_time_remaining

# Amount of distance that the robot has covered since it received this goal
float32 distance_covered

# Estimated remaining distance to the goal
float32 distance_to_goal
# Feedback extra information ===================================================== 

