# 仿真框架平台

基于 ROS 2 + Gazebo，构建三层楼场景下的机器人三维路径规划仿真平台，用于规划算法的开发、测试与对比。

实验链路：

> 地图数据 → 全局路径规划 → 局部轨迹规划 → 理想运动学执行

## 技术方案

| 模块 | 技术 / 方法 |
| --- | --- |
| 仿真环境 | Gazebo Classic |
| 通信框架 | ROS 2 Humble |
| 开发语言 | C++、Python |
| 运行环境 | Docker |

## 流程与目录

### 流程：

```text
仿真启动：Gazebo 加载 World + URDF
                    │
three_floor.world   │
        │           │
        ▼           │
map_process.cpp     │
     （离线）       │
        │           │
        ▼           │
   scene.yaml       │
        │           │
        ├───────────────────────────┐
        │                           │
        ▼                           │
 global_node.cpp（A*）              │
   ▲                                │
   └── 起点、终点（预设/HMI）       │
        │                           │
        ▼                           │
   /global_path                     │
        │                           │
        ▼                           ▼
 local_node.cpp（TEB）◄── 地图、运动约束
        ▲
        │
        │ /robot/odom
        │
        │       /local_trajectory
        │               │
        │               ▼
        │          control.cpp
        │               │
        │               ▼
        │        Gazebo 机器人运动
        │               │
        └───────────────┤
                        │
                        ▼
                   到达目标判断
                        │
                        ▼
                     任务完成
```

### 目录：

```text
simulation/
└── sim_ws/
    └── src/
        ├── hmi/                          # 人机交互（后续实现）
        │
        ├── sim_bringup/                  # 仿真启动与数据准备
        │   ├── worlds/
        │   │   └── three_floor.world
        │   ├── config/
        │   │   └── scene.yaml            # 地图、机器人参数、预设点位
        │   ├── src/
        │   │   └── map_process.cpp       # 离线地图处理
        │
        ├── global_planning/              # 全局路径规划
        │   ├── include/
        │   │   └── astar.h
        │   └── src/
        │       ├── astar.cpp             # A* 算法
        │       └── global_node.cpp       # 全局规划 ROS 节点
        │
        ├── local_planning/               # 局部轨迹规划
        │   ├── include/
        │   │   └── teb.h
        │   └── src/
        │       ├── teb.cpp               # TEB 算法
        │       └── local_node.cpp        # 局部规划 ROS 节点
        │
        ├── control/                      # 运动学执行
        │   ├── config/
        │   │   └── control.yaml          # 控制参数
        │   ├── include/
        │   │   └── control.h
        │   ├── src/
        │   │   └── control.cpp           # 运动学控制插件
        │   └── urdf/
        │       └── robot.urdf            # 机器人模型
        │
        └── interfaces/                   # ROS 自定义消息（按需）
            └── msg/
                ├── GlobalPath.msg
                └── LocalTrajectory.msg
```

## 模块职责

- `sim_bringup`：仿真启动、World 加载、离线地图处理。
- `global_planning`：全局路径规划，输出 `/global_path`。
- `local_planning`：统一局部轨迹规划，输出 `/local_trajectory`。
- `control`：根据轨迹驱动机器人，发布 `/robot/odom`。
- `interfaces`：定义必要的 ROS 通信接口。
- `hmi`：起终点选择、任务操作与结果展示。

### Control 控制模块

随机器人模型加载的 Gazebo 插件，负责轨迹校验、轮式运动、足步运动（含上下台阶）、轮足切换及状态反馈。采用理想运动学执行，控制参数来自 `control.yaml`，关节限制来自 URDF。

```text
/local_trajectory → 校验与分段 → 必要时轮足切换 → 定时执行 → Gazebo
                                                            ↓
                                          /robot/odom、/joint_states
```

| 方向 | 话题与消息类型 | 主要数据 |
| --- | --- | --- |
| 输入 | `/local_trajectory` · `interfaces/msg/LocalTrajectory` | `header.frame_id`、`mode`、`points` |
| 输出 | `/robot/odom` · `nav_msgs/msg/Odometry` | 本体位姿与速度，坐标系为 `world` / `base_link` |
| 输出 | `/joint_states` · `sensor_msgs/msg/JointState` | 11 个关节的名称、位置与速度 |

轨迹约定：

- `mode` 为 `wheel` 或 `foot`，一条消息只包含一种模式。
- 每点的 `transforms[0]` 表示 `base_link` 在 `world` 下的位置与四元数；首点须衔接目标模式准备完成后的位姿。
- 首点 `time_from_start` 为零，后续严格递增；切换完成后开始计时，`header.stamp` 不参与调度。执行依据位姿和时间，当前不使用 `velocities/accelerations`。
- 平面段支持前后、转向与圆弧；台阶段须匹配配置中的完整单阶能力。逆解或关节超速异常会冻结控制，重新生成机器人后恢复。

## 实验流程

1. 配置docker环境
2. 在 Docker 环境中启动 ROS 2 和 Gazebo
3. 加载 World 与机器人模型
4. 启动全局规划
5. 启动局部规划
6. 机器人执行
