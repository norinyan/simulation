"""加载场景、运动学执行器与可选的自主规划任务。"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('sim_bringup'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    description = xacro.process_file(
        str(share / 'urdf' / 'robot.urdf.xacro'),
        mappings={'static_display': 'true',
                  'scene_file': str(share / 'config' / 'scene.yaml'),
                  'planning_file': str(share / 'config' / 'planning.yaml'),
                  'executor_library': str(Path(get_package_prefix('sim_bringup')) / 'lib' / 'librobot_executor.so')},
    ).toxml()

    # 由展开后的模型计算出生高度：轮架下降量 + 轮半径。
    # 本章零关节值，轮与轮架的 Z 偏移为零，下层地面高度为 0。
    robot = ET.fromstring(description)
    lift_z = float(robot.find("joint[@name='wheel_lift_joint']/origin").get('xyz').split()[2])
    radius = float(robot.find("link[@name='left_wheel']/collision/geometry/cylinder").get('radius'))
    spawn_z = radius - lift_z

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments={
            'world': str(share / 'worlds' / 'two_steps.world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'pause': 'false',
            'server_required': 'true',
        }.items(),
    )
    # 执行插件发布 joint_states 与实际 odom，机器人状态发布器负责关节 TF。
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(description, value_type=str),
            'use_sim_time': True,
        }],
    )
    # spawn_entity 自行等待描述与 /spawn_entity 服务，不使用固定启动延时。
    scene = yaml.safe_load((share / 'config' / 'scene.yaml').read_text())
    start = scene['scene']['waypoints']['A']
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_wheel_leg_robot',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-entity', 'wheel_leg_robot', '-topic', '/robot_description',
            '-x', str(start[0]), '-y', str(start[1]), '-z', str(start[2] + spawn_z), '-Y', str(start[3]),
            '-timeout', '60.0',
        ],
    )
    planning = Node(
        package='sim_bringup',
        executable='planning_node',
        name='planning_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('planning')),
        parameters=[
            str(share / 'config' / 'planning.yaml'),
            {
                'scene_file': str(share / 'config' / 'scene.yaml'),
                'output_dir': LaunchConfiguration('results_dir'),
                'use_sim_time': True,
                'local_enabled': ParameterValue(LaunchConfiguration('local_planning'), value_type=bool),
                'autonomous': ParameterValue(LaunchConfiguration('autonomous'), value_type=bool),
                'foot_enabled': ParameterValue(LaunchConfiguration('foot_planning'), value_type=bool),
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('autonomous', default_value='false', description='自动执行 A→E→X→B，需启用三类规划'),
        DeclareLaunchArgument('gui', default_value='true', description='启动 Gazebo 图形界面'),
        DeclareLaunchArgument('planning', default_value='false', description='启动全局 A* 参考路径规划'),
        DeclareLaunchArgument('local_planning', default_value='false', description='启用单次轮式 TEB 预测，需 planning:=true'),
        DeclareLaunchArgument('foot_planning', default_value='false', description='启用 E→X 足步与五次轨迹规划，需 planning:=true'),
        DeclareLaunchArgument(
            'results_dir',
            default_value=str(Path.home() / '.local/state/simulation/global_planning'),
            description='规划 CSV 输出根目录，每次运行创建独立子目录',
        ),
        gazebo,
        state_publisher,
        spawn,
        planning,
    ])
