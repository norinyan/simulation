# 三层楼机器人规划仿真平台

基于 **ROS 2 Humble + Gazebo Classic** 的三维规划实验平台，支持跨楼层路径规划、局部滚动轨迹规划、动态行人避障及 HMI 可视化。当前接入 **PCT 全局规划 + SCAN 局部规划**，使用理想点模型执行轨迹。

## 构建

宿主机需安装 Docker Engine、Docker Compose（支持 `gpus: all`）、NVIDIA 驱动及 NVIDIA Container Toolkit；桌面启动需可用的 X11 显示环境。PCT 地图预处理使用 CUDA 12 / CuPy，需兼容的 NVIDIA GPU 与驱动。前端在宿主机构建，需 Node.js **20.19+ 或 22.12+** 及 npm；其余依赖由 Docker 镜像提供。

### 1. 构建前端和容器

在项目根目录执行：

```bash
# 前端产物通过项目挂载提供给容器；镜像内未安装 Node.js
npm --prefix src/hmi/frontend ci
npm --prefix src/hmi/frontend run build

# 允许容器内 root 用户访问宿主机 X11
xhost +si:localuser:root
docker compose -f docker/compose.yaml up -d --build
docker compose -f docker/compose.yaml exec dev bash
```

### 2. 构建 ROS 工作空间并生成地图

在容器内执行，项目已挂载到 `/workspace`：

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --packages-up-to sim_bringup
source install/setup.bash

/opt/pct/bin/python3 src/global_planning/pct/map_processing/prepare_map.py \
  --config src/global_planning/config/global_cfg.yaml
```

地图输出为 `build/pct_maps/three_floor_ideal.pctmap` 及同名 `.json` 元数据。首次运行必须生成；修改 World 或地图预处理参数后重新执行上述命令，未变化时自动复用缓存。

## 启动与操作

在已加载上述 ROS 环境的容器终端执行：

```bash
ros2 launch sim_bringup hmi.launch.py
```

该入口启动后端和桌面窗口；在界面中按顺序操作：

1. 选择起点、终点及是否开启“行人”（默认开启）。
2. 点击 **启动仿真**，等待机器人里程计及规划节点就绪。
3. 点击 **开始全局规划**，等待全局路径生成。
4. 点击 **启动任务**，观察局部滚动规划、避障与执行状态，直到任务完成。
5. 按需 **暂停/恢复仿真**；结束时点击 **停止仿真**。

修改起终点后需重新全局规划；切换行人开关需先停止仿真。HMI 会管理 Gazebo 和规划进程，无需另行启动 `sim.launch.py` 或 `planning.launch.py`。

也可只启动后端，在宿主机浏览器打开 <http://127.0.0.1:8000>：

```bash
# 容器内执行，与桌面启动方式二选一
bash src/hmi/run_hmi.sh
```

重新进入容器时，先执行 `source /opt/ros/humble/setup.bash` 和 `source /workspace/install/setup.bash`。停止容器可在宿主机项目根目录执行：

```bash
docker compose -f docker/compose.yaml down
```

## 架构与运行流程

```text
离线：three_floor.world → GPU 地图预处理 → .pctmap + .json
                                              │
在线：HMI 选择起终点 → PCT 全局规划 ←─────────────┘
                            │ /global_path（地面路径 + 运动模式）
                            ▼
                     SCAN 局部滚动规划 ← /dynamic_obstacles
                            │                 ↑
                            │           Gazebo 行人实际状态
                            ▼ /local_trajectory（定时本体轨迹）
                     理想执行器 / Gazebo
                            │ /robot/odom
                            └──────────→ 局部规划反馈 → 到达终点
```

HMI 通过 FastAPI / WebSocket 桥接 ROS，展示地图、路径、轨迹、机器人、行人、规划耗时和进程资源占用。全局规划读取离线地图，局部规划直接读取 World 碰撞几何，并结合里程计和动态障碍观测滚动生成轨迹。

## 目录与模块

```text
simulation_new/                   # ROS 工作空间根目录
├── docker/                       # Dockerfile、Compose 与 GPU/显示配置
├── doc/                          # 需求文档、使用手册
├── src/
│   ├── sim_bringup/              # 启动入口、三层楼场景、动态行人
│   │   ├── launch/               # hmi / sim / planning.launch.py
│   │   ├── worlds/               # three_floor.world
│   │   ├── config/               # dynamic_obstacles.yaml
│   │   └── scripts/              # dynamic_obstacles.py
│   ├── global_planning/          # 全局节点、配置、PCT 算法与地图预处理
│   │   ├── global_node.cpp
│   │   ├── config/
│   │   └── pct/                  # map_processing/、third_party/ 等
│   ├── local_planning/           # 局部节点、配置与 SCAN 算法
│   │   ├── local_node.cpp
│   │   ├── config/
│   │   └── scan/
│   ├── control/                  # Gazebo 理想轨迹执行插件、URDF 与配置
│   ├── interfaces/msg/           # GlobalPath、LocalTrajectory、DynamicObstacle
│   ├── hmi/                      # 独立 HMI，非 colcon 包
│   │   ├── backend/              # FastAPI、ROS 桥接及进程管理
│   │   ├── frontend/             # React + Three.js + Vite
│   │   └── desktop.py            # PySide6 桌面窗口
│   └── test/                     # 任务数据样例
└── build/、install/、log/         # 本地生成，不纳入 Git
```

| 模块 | 主要职责 |
| --- | --- |
| `sim_bringup` | 加载场景、生成机器人、启动规划节点；按机器人位置和运动方向触发行人，发布实际障碍观测。 |
| `global_planning` | PCT 多层地图搜索与地形分类，输出带 `wheel` / `foot` 分段标记的全局地面路径。 |
| `local_planning` | SCAN B 样条优化与滚动重规划，处理速度/加速度限制、静动态避障、等待与安全停车。 |
| `control` | 当前构建 `ideal_control.cpp`，按仿真时间校验、预约和插值执行轨迹，反馈本体里程计。 |
| `interfaces` | 定义全局路径、局部轨迹及动态障碍的 ROS 消息。 |
| `hmi` | 管理仿真和任务生命周期，选择点位、发起规划并展示实时结果。 |

## 配置与约定

| 配置文件 | 内容 |
| --- | --- |
| `src/global_planning/config/global_cfg.yaml` | 九个预设点位、全局算法、地图路径、通行约束及 PCT 参数。 |
| `src/local_planning/config/local_cfg.yaml` | SCAN 参数、规划范围与周期、运动限制、动态障碍预测参数。 |
| `src/control/config/ideal.yaml` | 统一参考高度（默认 `0.44 m`）、反馈周期及到达容差。 |
| `src/sim_bringup/config/dynamic_obstacles.yaml` | 各楼层行人的尺寸、路线、速度和触发区域。 |

- 坐标统一使用 `world`，单位为米和弧度；预设点和全局路径的 `z` 表示地面高度，局部轨迹与里程计的 `z` 表示本体高度。
- 局部轨迹的 `header.stamp` 是仿真时间下的预约生效时刻，`time_from_start` 是相对轨迹时间；执行器检查时效及衔接连续性。
- 默认配置路径基于容器 `/workspace`。HMI 使用配置临时副本，行人开关会覆盖局部规划配置中的 `dynamic_obstacles.enabled`，不修改源配置。
- HMI 默认使用 `ROS_DOMAIN_ID=186`、`GAZEBO_MASTER_URI=http://127.0.0.1:11386`；外部调试节点需使用同一环境，可分别通过 `HMI_ROS_DOMAIN_ID`、`HMI_GAZEBO_MASTER_URI` 覆盖。
- 当前机器人是理想点模型，方块仅作显示，执行器关闭重力与碰撞；避障由规划器计算。仓库中的旧 `control.cpp` 未参与当前构建，`wheel` / `foot` 是路径模式标记，不代表已验证真实轮足动力学。
