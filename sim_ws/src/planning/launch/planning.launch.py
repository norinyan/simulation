"""Start navigation against an existing scene and robot; do not spawn either."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    scene_file = Path(value('scene_file')).expanduser()
    if not value('scene_file') or not scene_file.is_file():
        raise ValueError('scene_file must name the shared scene configuration or task snapshot')
    execution = yaml.safe_load(scene_file.read_text())['robot']['execution']
    global_follow = value('wheel_execution') == 'global_path'
    return [Node(package='planning', executable='planning_node', output='screen', parameters=[
        value('planning_file'), dict(scene_file=str(scene_file), output_dir=value('results_dir'), use_sim_time=True,
            local_enabled=value('local_planning') == 'true' and not global_follow,
            global_path_execution=global_follow,
            foot_enabled=ParameterValue(LaunchConfiguration('foot_planning'), value_type=bool),
            autonomous=ParameterValue(LaunchConfiguration('autonomous'), value_type=bool),
            interactive=ParameterValue(LaunchConfiguration('interactive'), value_type=bool)),
        {'planners.'+key: value(key+'_planner') for key in ('global', 'wheel', 'foot')},
        {'task.'+key: val for key, val in execution.items()}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_file', default_value=''),
        DeclareLaunchArgument('planning_file', default_value=str(Path(get_package_share_directory('planning'))/'config/planning.yaml')),
        DeclareLaunchArgument('results_dir', default_value=str(Path.home()/'.local/state/simulation/planning')),
        DeclareLaunchArgument('wheel_execution', default_value='local', choices=['local', 'global_path'],
                              description='global_path follows checked A* edges without wheel local planning'),
        *[DeclareLaunchArgument(key, default_value=default, choices=['true', 'false']) for key, default in
          [('local_planning', 'true'), ('foot_planning', 'true'), ('autonomous', 'true'), ('interactive', 'false')]],
        *[DeclareLaunchArgument(key+'_planner', default_value=default) for key, default in
          [('global', 'astar'), ('wheel', 'teb'), ('foot', 'footstep')]],
        OpaqueFunction(function=setup)])
