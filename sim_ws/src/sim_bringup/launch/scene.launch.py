"""Start a selected Gazebo scene, its dynamic models, and optional visualization."""
from datetime import datetime
from pathlib import Path
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    scene, view, floor = value('scene'), value('view'), int(value('floor'))
    factor, snapshot = float(value('real_time_factor')), float(value('snapshot_after'))
    if not math.isfinite(factor) or not 1 <= factor <= 10:
        raise ValueError('real_time_factor must be between 1 and 10')
    if not math.isfinite(snapshot) or snapshot < 0:
        raise ValueError('snapshot_after must be a finite nonnegative number')
    if scene == 'two_steps' and (view not in ('exterior', 'top') or floor != 0):
        raise ValueError('two_steps supports exterior/top views with floor:=0')
    if scene == 'three_floor':
        if view in ('top', 'interior') and floor == 0:
            raise ValueError('top/interior views require floor:=1, 2, or 3')
        if view == 'split' and floor != 0:
            raise ValueError('view:=split displays all floors; use floor:=0')
    output = Path(value('output_dir')).expanduser()
    output.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('sim_bringup'))
    lib = Path(get_package_prefix('sim_bringup'))/'lib'
    tree = ET.parse(share/'worlds'/f'{scene}.world')
    physics = tree.getroot().find('world/physics')
    # Keep the old robot's integration step; the building observer uses 5 ms steps.
    step = .005 if scene == 'three_floor' else float(physics.findtext('max_step_size'))
    physics.find('max_step_size').text = str(step)
    physics.find('real_time_update_rate').text = str(factor/step)
    node = physics.find('real_time_factor')
    if node is None:
        node = ET.SubElement(physics, 'real_time_factor')
    node.text = str(factor)
    world = output/'runtime.world'
    tree.write(world, encoding='utf-8', xml_declaration=True)
    plugin_path = str(lib)+os.pathsep+os.environ.get('GAZEBO_PLUGIN_PATH', '')
    env = dict(os.environ, GAZEBO_MASTER_URI=value('master_uri'), GAZEBO_PLUGIN_PATH=plugin_path,
               GAZEBO_MODEL_DATABASE_URI='', SCENE_OUTPUT=str(output), SCENE_WORLD=scene,
               SCENE_VIEW=view, SCENE_FLOOR=str(floor), SCENE_SNAPSHOT_AFTER=str(snapshot))
    server = ExecuteProcess(cmd=['gzserver', '--verbose', str(world),
                                 '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], env=env)
    actions = [SetEnvironmentVariable('GAZEBO_MASTER_URI', value('master_uri')),
               SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),
               RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
                   EmitEvent(event=Shutdown(reason='Gazebo server exited'))])), server]
    if value('gui') == 'true':
        command = ['gzclient', '--verbose']
        command += ['--gui-client-plugin', str(lib/'libscene_visualization.so')]
        actions.append(TimerAction(period=2.0, actions=[ExecuteProcess(cmd=command, env=env)]))
    return actions


def generate_launch_description():
    output = Path.home()/'.local/state/simulation'/datetime.now().strftime('scene_%Y%m%d_%H%M%S_%f')
    return LaunchDescription([
        DeclareLaunchArgument('scene', default_value='three_floor', choices=['three_floor', 'two_steps']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('view', default_value='exterior', choices=['exterior', 'top', 'interior', 'split']),
        DeclareLaunchArgument('floor', default_value='0', choices=['0', '1', '2', '3']),
        DeclareLaunchArgument('real_time_factor', default_value='1'),
        DeclareLaunchArgument('snapshot_after', default_value='2'),
        DeclareLaunchArgument('output_dir', default_value=str(output)),
        DeclareLaunchArgument('master_uri', default_value='http://127.0.0.1:11345'),
        OpaqueFunction(function=setup),
    ])
