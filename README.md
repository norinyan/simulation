# 轮足机器人 Gazebo 仿真

基于 ROS 2 Humble、Gazebo Classic 和 C++，执行轮足机器人 A→E→X→B 自主任务。采用运动学仿真，包含障碍物绕行、双台阶攀爬、轮足形态切换及执行反馈。

```text
docker/                      Dockerfile 与 Compose 环境
sim_ws/src/planning/         全局 A*、轮式 TEB、足式规划算法
sim_ws/src/sim_bringup/      场景、URDF、参数、统一 Launch
  src/planning_node.cpp     规划调用与自动任务调度
  src/robot_executor.cpp    Gazebo 运动学执行插件
sim_ws/results/             按运行批次保存的结果
```

## Docker 环境

宿主机使用当前 X11 桌面和 NVIDIA Container Toolkit：

```bash
cd /home/hfun/norin/lab/simulation
xhost +si:localuser:root
docker compose -f docker/compose.yaml up -d --build
docker compose -f docker/compose.yaml exec dev bash
```

首次运行或修改源码后，在容器内编译：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/sim_ws
colcon build --packages-select planning sim_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## 一键运行

在容器内执行：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/sim_ws
source install/setup.bash
ros2 launch sim_bringup sim.launch.py \
  planning:=true \
  local_planning:=true \
  foot_planning:=true \
  autonomous:=true \
  results_dir:=/workspace/sim_ws/results
```

`gui:=false` 可关闭图形界面。`autonomous` 默认为 `false`，保留原有单次规划、ROS Topic 和规划 CSV 输出。

## 算法与执行流程

1. 全局 A* 根据 `config/scene.yaml` 的关键点、边界和障碍物，生成 A→E、X→B 参考路径。
2. 轮式 TEB 根据 Gazebo 实际里程计循环规划，输出 v、w；临近终点减速，必要时细化 TEB 采样，并保持原有碰撞及运动约束。
3. 足式规划采用足步 A* 与五次多项式轨迹。插件消费机身、双足轨迹，通过逆运动学更新连杆位姿并发布实际反馈。

执行顺序：A→E 轮式行驶 → 停止并切换足式 → E→X 足式攀爬 → 切换轮式 → X→B 轮式行驶 → 停止并进入 `DONE`。模式切换、足式完成和目标到达均由执行反馈确认；规划不可行、碰撞风险或反馈/任务超时会停车并报告失败原因。

任务状态发布在 `/planning/task_status`；实际里程计、足端反馈及执行状态分别位于 `/robot/odom`、`/robot/foot_feedback`、`/robot/execution_status`。场景关键点的 Z 为地面高度，执行反馈的机身 Z 为 `base_link` 高度，足端位置为足部碰撞几何中心。

## 结果文件

上述命令将结果写入宿主机 `sim_ws/results/run_*/`：

- `task_result.csv`：最终状态、位置/高度/朝向误差、实际速度、仿真及墙钟耗时。
- `task_events.csv`、`actual_trajectory.csv`：任务阶段与实际机身、足端轨迹。
- `teb_execution.csv`：每次闭环规划的实际初始状态、优化轨迹和采样间隔。
- `path.csv`、`result.csv`：全局参考路径及规划结果。
- `local_trajectory.csv`、`local_result.csv`：原有单次轮式预测。
- `footstep_path.csv`、`foot_trajectory.csv`：足步及五次多项式规划轨迹。

不指定 `results_dir` 时，默认保存到容器内 `~/.local/state/simulation/global_planning/run_*/`。
