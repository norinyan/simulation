"""启动三层场景及理想方块机器人。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def spawn_robot(context):
    control_share = Path(get_package_share_directory('control'))
    config_file = LaunchConfiguration('ideal_config').perform(context)
    config = yaml.safe_load(Path(config_file).read_text())
    # 空 z 使用统一参考高度，可覆盖为其他楼层地面高度加参考高度。
    z = LaunchConfiguration('z').perform(context) or str(config['reference_height'])
    return [Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ideal_robot',
            '-file', str(control_share / 'urdf' / 'ideal_robot.urdf'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', z,
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    control_share = Path(get_package_share_directory('control'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments={
            'world': str(bringup_share / 'worlds' / 'three_floor.world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='-6.3'),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value='1.5707963267948966'),
        DeclareLaunchArgument('ideal_config', default_value=str(control_share / 'config' / 'ideal.yaml')),
        SetEnvironmentVariable('IDEAL_CONTROL_CONFIG', LaunchConfiguration('ideal_config')),
        gazebo,
        OpaqueFunction(function=spawn_robot),
    ])
