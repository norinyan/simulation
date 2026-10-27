"""启动全局和局部规划节点；HMI 显式传入 auto_execute:=false。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    global_config = Path(get_package_share_directory('global_planning')) / 'config/global_cfg.yaml'
    local_config = Path(get_package_share_directory('local_planning')) / 'config/local_cfg.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('global_config', default_value=str(global_config)),
        DeclareLaunchArgument('local_config', default_value=str(local_config)),
        DeclareLaunchArgument('auto_execute', default_value='true'),
        Node(
            package='global_planning', executable='global_node', output='screen',
            parameters=[{'config_file': LaunchConfiguration('global_config')}],
        ),
        Node(
            package='local_planning', executable='local_node', output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('local_config'),
                'auto_execute': ParameterValue(LaunchConfiguration('auto_execute'), value_type=bool),
            }],
        ),
    ])
