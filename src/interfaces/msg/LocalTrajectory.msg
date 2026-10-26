std_msgs/Header header
string mode
trajectory_msgs/MultiDOFJointTrajectoryPoint[] points

# mode:
#   "wheel" - 轮式运动
#   "foot"  - 足式运动

# points 中每个轨迹点包含：
#   transforms[0].translation.x   -> x
#   transforms[0].translation.y   -> y
#   transforms[0].translation.z   -> z
#   transforms[0].rotation.x      -> qx
#   transforms[0].rotation.y      -> qy
#   transforms[0].rotation.z      -> qz
#   transforms[0].rotation.w      -> qw
#   velocities[0].linear.x        -> vx
#   velocities[0].linear.y        -> vy
#   velocities[0].linear.z        -> vz
#   velocities[0].angular.x       -> wx
#   velocities[0].angular.y       -> wy
#   velocities[0].angular.z       -> wz
#   time_from_start               -> t