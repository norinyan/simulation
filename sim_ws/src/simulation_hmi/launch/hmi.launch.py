"""Launch only the control panel; scenes are started explicitly from its buttons."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='91'),
        DeclareLaunchArgument('master_uri', default_value='http://127.0.0.1:11355'),
        Node(package='simulation_hmi', executable='panel', output='screen',
             arguments=['--domain-id', LaunchConfiguration('domain_id'),
                        '--master-uri', LaunchConfiguration('master_uri')]),
    ])
