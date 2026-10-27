# Actual Gazebo observation; all coordinates and velocities are in header.frame_id.
std_msgs/Header header
string id
# true: actual entity observation; false + nonempty id: confirmed deletion.
# false + empty id: publisher heartbeat (does not clear active entities).
bool active
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Twist twist
