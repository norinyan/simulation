# One ordered volatile channel for all execution commands. No old task replay.
std_msgs/Header header
string task_id
uint64 segment_id
uint64 sequence
builtin_interfaces/Time valid_until
# BEGIN, HEARTBEAT, HOLD, STOP, LEG, WHEEL, WHEEL_GOAL, GLOBAL_WHEEL_GOAL,
# WHEEL_TRAJECTORY, FOOT_TRAJECTORY
string command
trajectory_msgs/MultiDOFJointTrajectory trajectory
geometry_msgs/Pose goal
bool path_tracking
# Current collision projection for tracking/braking safety, not a route decision.
float64[4] bounds
Box[] obstacles
Box[] supports
bool require_support
float64 safety_margin
