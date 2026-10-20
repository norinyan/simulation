"""Load the robot into an existing two_steps Gazebo server, with optional planning."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('scene') != 'two_steps':
        raise ValueError('Robot configuration is only available for scene:=two_steps; three_floor currently supports observation only')
    flags = {name: value(name) == 'true' for name in ('planning', 'local_planning', 'foot_planning', 'autonomous')}
    if (flags['local_planning'] or flags['foot_planning']) and not flags['planning']:
        raise ValueError('local_planning and foot_planning require planning:=true')
    if flags['autonomous'] and not all(flags[name] for name in ('planning', 'local_planning', 'foot_planning')):
        raise ValueError('autonomous:=true requires planning:=true local_planning:=true foot_planning:=true')
    share = Path(get_package_share_directory('sim_bringup'))
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
    return [SetEnvironmentVariable('GAZEBO_MASTER_URI', value('master_uri')),
            state_publisher, spawn, planning]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='two_steps', choices=['two_steps', 'three_floor']),
        DeclareLaunchArgument('master_uri', default_value='http://127.0.0.1:11345'),
        DeclareLaunchArgument('planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('local_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('foot_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autonomous', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('results_dir', default_value=str(Path.home()/'.local/state/simulation/global_planning')),
        OpaqueFunction(function=setup),
    ])
