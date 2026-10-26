"""启动 Gazebo Classic，加载三层楼场景并生成轮腿机器人。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取各 ROS 包的安装资源目录，用于查找启动文件、场景和机器人模型。
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    control_share = Path(get_package_share_directory('control'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))

    # 启动 Gazebo：复用 gazebo_ros 包自带的启动文件，同时启动服务端和界面。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'gazebo.launch.py')
        ),
        launch_arguments={
            # 加载场景：让 Gazebo 在启动时读取三层楼世界文件。
            'world': str(bringup_share / 'worlds' / 'three_floor.world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    # 默认生成在 (0, -6.3, 0.32)，可指定位置与朝向进行独立测试。
    # spawn_entity.py 会等待 /spawn_entity 服务就绪后再生成，无需固定延时。
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wheel_leg_robot',
            '-file', str(control_share / 'urdf' / 'robot.urdf'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    # 将 Gazebo 和机器人生成节点交给 ROS 2 launch 启动。
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='-6.3'),
        DeclareLaunchArgument('z', default_value='0.32'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument(
            'control_config',
            default_value=str(control_share / 'config' / 'control.yaml'),
            description='轮足控制配置文件，使用 Gazebo 容器内的路径',
        ),
        SetEnvironmentVariable(
            'WHEEL_LEG_CONTROL_CONFIG', LaunchConfiguration('control_config')
        ),
        gazebo,
        spawn_robot,
    ])
