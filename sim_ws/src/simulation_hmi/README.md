# 仿真控制台（第一版）

独立 Qt 面板，Gazebo 在另一个窗口显示。复用 sim_bringup 场景和机器人入口，不实现规划算法。

容器内构建并启动：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/sim_ws
colcon build --packages-up-to simulation_hmi --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch simulation_hmi hmi.launch.py
```

也可直接 `ros2 run simulation_hmi panel`。默认专用 ROS_DOMAIN_ID=91、Gazebo master=http://127.0.0.1:11355，避免接入已有命令行会话。更换参数：`ros2 launch simulation_hmi hmi.launch.py domain_id:=92 master_uri:=http://127.0.0.1:11356`。另开终端查询此面板的 topic 时，应先 `export ROS_DOMAIN_ID=91`。只支持本机容器内 Gazebo。

使用顺序：

1. 选场景与速度，启动仿真，等待模型列表和实际时钟就绪。
2. 选出生点与目标点，生成机器人，等待实际 odom 和执行器状态。绿色圈表示出生点，红色圈表示目标点。
3. 两级台阶 A→B、正常速度下可“仅规划”，响应成功后“开始执行”。规划期间机器人保持静止。
4. “停止运动”通过任务服务终止调度并发送 STOP；“关闭仿真”关闭整个会话。停止/完成后的任务不支持断点续跑，需移除并重新生成机器人。
5. 选择总览、俯视、内部透视、三层同屏，或点击第三人称跟随；“恢复所选视角”退出跟随。跟随采用后方 4.8 m、抬高 4.8 m 的宽视野（水平视场角 80°）；墙、门、窗仅在显示中淡化，避免房间墙体遮挡，退出跟随后恢复原材质透明度。

预设点集中在 config/presets.yaml：两级台阶 A/E/X/B，三层楼点位为：F1 大门内侧、楼梯口前、右上房间；F2 楼梯口前、右下房间、走廊最右端；F3 楼梯口前、走廊最右端、中间下方窗前。两场景共 13 个点。大门外没有楼板支撑，因此入口点放在门内约 0.6 m 的安全区。坐标为地面 x,y,z,yaw，机器人生成自动加轮式高度。三层放置启用真实几何碰撞和地面支撑检查；静态预检不保证动态行人不会与停放的机器人相遇，实际碰撞仍由执行器保护。

本版只允许原 A→E→X→B 任务；其他出生点/目标组合仅放置机器人和显示标记。三层同层/跨层导航、任意点选、算法切换和电梯控制均未实现。观察场景可用 5/10 倍，机器人闭环执行只开放正常速度。

状态来自 ROS 实际反馈；按钮点击成功不等于任务完成。运行配置、进程日志和规划数据在 ~/.local/state/simulation_hmi/<时间>/。面板只管理自己创建的进程组，不按进程名停止其他仿真；退出面板时异步等待 SIGINT→SIGTERM→SIGKILL 清理完成。

模块：app.py（界面与状态）、session.py（进程生命周期）、ros_bridge.py（ROS 反馈/服务）、presets.py（点位与静态预检）。后端新增 /planning/plan、/planning/execute、/planning/stop 三个 std_srvs/Trigger 服务，只在 interactive:=true 模式启用；原命令行自动任务保持原行为。
