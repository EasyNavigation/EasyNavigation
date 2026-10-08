# State of the safety channel (safety PLC, safety scanner...), published on
# "easynav_safety_status" by the integrator. EasyNav does not stop the robot by itself: it stops
# commanding it while the safety channel stops it, and keeps within its safely limited speed.

std_msgs/Header header

# The safety channel is stopping, or has stopped, the robot.
bool protective_stop

# A safely limited speed (SLS) is in force: max_linear_vel and max_angular_vel apply (>= 0).
bool speed_limited
float64 max_linear_vel   # m/s, either direction
float64 max_angular_vel  # rad/s, either direction

# For diagnostics only.
string active_field
bool muting
