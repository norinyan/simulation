"""Compose a scene and optional robot; default to observing the three-floor building."""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    robot = value('robot') == 'true'
    flags = {name: value(name) == 'true' for name in ('planning', 'local_planning', 'foot_planning', 'autonomous')}
    if robot and value('scene') != 'two_steps':
        raise ValueError('scene:=three_floor robot:=true is not configured; use scene:=two_steps robot:=true')
    if any(flags.values()) and not robot:
        raise ValueError('Robot planning/control requires robot:=true')
    if (flags['local_planning'] or flags['foot_planning']) and not flags['planning']:
        raise ValueError('local_planning and foot_planning require planning:=true')
    if flags['autonomous'] and not all(flags[name] for name in ('planning', 'local_planning', 'foot_planning')):
        raise ValueError('autonomous:=true requires planning:=true local_planning:=true foot_planning:=true')
    directory = Path(get_package_share_directory('sim_bringup'))/'launch'
    def include(name, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(directory/name)),
                                        launch_arguments={key: value(key) for key in arguments}.items())
    actions = [include('scene.launch.py', ('scene', 'gui', 'view', 'floor', 'real_time_factor',
                                          'snapshot_after', 'output_dir', 'master_uri'))]
    if robot:
        actions.append(include('robot.launch.py', ('scene', 'master_uri', 'planning', 'local_planning',
                                                  'foot_planning', 'autonomous', 'results_dir')))
    return actions


def generate_launch_description():
    output = Path.home()/'.local/state/simulation'/datetime.now().strftime('scene_%Y%m%d_%H%M%S_%f')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='three_floor', choices=['three_floor', 'two_steps']),
        DeclareLaunchArgument('robot', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('view', default_value='exterior', choices=['exterior', 'top', 'interior', 'split']),
        DeclareLaunchArgument('floor', default_value='0', choices=['0', '1', '2', '3']),
        DeclareLaunchArgument('real_time_factor', default_value='1'),
        DeclareLaunchArgument('snapshot_after', default_value='2'),
        DeclareLaunchArgument('output_dir', default_value=str(output)),
        DeclareLaunchArgument('master_uri', default_value='http://127.0.0.1:11345'),
        DeclareLaunchArgument('planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('local_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('foot_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autonomous', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('results_dir', default_value=str(Path.home()/'.local/state/simulation/global_planning')),
        OpaqueFunction(function=setup),
    ])
