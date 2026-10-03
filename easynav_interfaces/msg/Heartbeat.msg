# Published by EasyNav from its real-time cycle: if it stops arriving, EasyNav stopped
# commanding the robot (a blocked cycle, a dead process).

uint8 RT_OK=0
uint8 RT_LATE=1
uint8 RT_ERROR=2

std_msgs/Header header

# +1 per heartbeat: a gap means lost heartbeats.
uint64 sequence

# RT_OK, RT_LATE (the last cycle started late) or RT_ERROR (too many late cycles in a row).
uint8 rt_status

# Cycles that started late since EasyNav was activated.
uint64 late_cycles

bool safety_mode

# SHA-256 of EasyNav's configuration.
string configuration_hash
