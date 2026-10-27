"""启动三层场景及理想方块机器人。"""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def start_gazebo(context):
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    world_file = bringup_share / 'worlds' / 'three_floor.world'
    actions = []
    gazebo_arguments = {'gui': LaunchConfiguration('gui')}
    if LaunchConfiguration('dynamic_obstacles').perform(context).lower() == 'true':
        # Entity-state services are a world plugin. Keep the source World
        # unchanged, and remove the runtime copy when this launch shuts down.
        world = ET.parse(world_file)
        plugin = ET.SubElement(world.find('world'), 'plugin',
                               name='dynamic_obstacle_state', filename='libgazebo_ros_state.so')
        ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/'
        ET.SubElement(plugin, 'update_rate').text = '20'
        with tempfile.NamedTemporaryFile(prefix='simulation-dynamic-', suffix='.world',
                                         delete=False) as runtime:
            world.write(runtime, encoding='utf-8', xml_declaration=True)
            world_file = Path(runtime.name)
        # gazebo_ros_init reads clock configuration through params_file.
        with tempfile.NamedTemporaryFile(mode='w', prefix='simulation-clock-', suffix='.yaml',
                                         delete=False) as parameters:
            yaml.safe_dump({'/**': {'ros__parameters': {'publish_rate': 100.0}}}, parameters)
            clock_file = Path(parameters.name)
        gazebo_arguments['params_file'] = str(clock_file)
        def cleanup(_context):
            world_file.unlink(missing_ok=True)
            clock_file.unlink(missing_ok=True)
            return []
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    gazebo_arguments['world'] = str(world_file)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
        launch_arguments=gazebo_arguments.items(),
    ))
    return actions


def spawn_robot(context):
    control_share = Path(get_package_share_directory('control'))
    config_file = LaunchConfiguration('ideal_config').perform(context)
    config = yaml.safe_load(Path(config_file).read_text())
    # 空 z 使用统一参考高度，可覆盖为其他楼层地面高度加参考高度。
    z = LaunchConfiguration('z').perform(context) or str(config['reference_height'])
    spawn = Node(
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
    )
    return [spawn]


def generate_launch_description():
    bringup_share = Path(get_package_share_directory('sim_bringup'))
    control_share = Path(get_package_share_directory('control'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='-6.3'),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value='1.5707963267948966'),
        DeclareLaunchArgument('ideal_config', default_value=str(control_share / 'config' / 'ideal.yaml')),
        DeclareLaunchArgument('dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('dynamic_obstacles_config',
                              default_value=str(bringup_share / 'config' / 'dynamic_obstacles.yaml')),
        SetEnvironmentVariable('IDEAL_CONTROL_CONFIG', LaunchConfiguration('ideal_config')),
        OpaqueFunction(function=start_gazebo),
        OpaqueFunction(function=spawn_robot),
        Node(package='sim_bringup', executable='dynamic_obstacles.py',
             condition=IfCondition(LaunchConfiguration('dynamic_obstacles')),
             parameters=[{'use_sim_time': True,
                          'config_file': LaunchConfiguration('dynamic_obstacles_config')}],
             output='screen'),
    ])
