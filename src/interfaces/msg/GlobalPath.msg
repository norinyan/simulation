std_msgs/Header header
geometry_msgs/Pose[] points
string[] modes

# 每个路径点包含：
# position:      x, y, z
# orientation:   qx, qy, qz, qw

# modes[i] 表示 points[i] 到 points[i+1] 的运动模式
# wheel：轮式
# foot：足式
# modes 数量 = points 数量 - 1