"""Load a robot into an existing server; three_floor uses unified wheel/foot route segments."""

from pathlib import Path
import math
import itertools
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


def verify_wheel_geometry(robot, geometry):
    """Check the configured full footprint against the expanded zero-joint model."""
    identity = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    def origin(element):
        if element is None:
            return [0., 0., 0.], identity
        xyz = list(map(float, element.get('xyz', '0 0 0').split()))
        roll, pitch, yaw = map(float, element.get('rpy', '0 0 0').split())
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        return xyz, [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    def compose(a, b):
        pa, ra = a
        pb, rb = b
        return ([pa[i]+sum(ra[i][j]*pb[j] for j in range(3)) for i in range(3)],
                [[sum(ra[i][k]*rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)])
    poses = {'base_link': ([0., 0., 0.], identity)}
    pending = list(robot.findall('joint'))
    while pending:
        ready = [joint for joint in pending if joint.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('Cannot resolve robot collision frames')
        for joint in ready:
            poses[joint.find('child').get('link')] = compose(poses[joint.find('parent').get('link')], origin(joint.find('origin')))
            pending.remove(joint)
    corners = []
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            shape = collision.find('geometry')
            if shape.find('box') is not None:
                half = [float(x)/2 for x in shape.find('box').get('size').split()]
            elif shape.find('cylinder') is not None:
                cylinder = shape.find('cylinder')
                half = [float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2]
            else:
                raise ValueError('Unsupported robot collision geometry')
            frame = compose(poses[link.get('name')], origin(collision.find('origin')))
            for signs in itertools.product((-1, 1), repeat=3):
                corners.append(compose(frame, ([half[i]*signs[i] for i in range(3)], identity))[0])
    length, width = [2*max(abs(p[i]) for p in corners) for i in range(2)]
    if abs(length-geometry['wheel_mode_length']) > 1e-6 or abs(width-geometry['wheel_mode_width']) > 1e-6:
        raise ValueError('Planning footprint does not match expanded robot collisions')
    height = geometry['wheel_radius']+geometry['carrier_drop']
    if abs(min(p[2] for p in corners)+height) > 1e-6 or max(p[2] for p in corners) > geometry['body_size'][2]/2+1e-6:
        raise ValueError('Planning height does not cover expanded robot collisions')


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    flags = {name: value(name) == 'true' for name in ('planning', 'local_planning', 'foot_planning', 'autonomous')}
    if (flags['local_planning'] or flags['foot_planning']) and not flags['planning']:
        raise ValueError('local_planning and foot_planning require planning:=true')
    required = ('planning', 'local_planning', 'foot_planning') if value('scene') == 'two_steps' else ('planning', 'local_planning')
    if flags['autonomous'] and not all(flags[name] for name in required):
        raise ValueError('autonomous requires the selected scene planning modes')
    share = Path(get_package_share_directory('sim_bringup'))
    scene_file = Path(value('scene_file')).expanduser() if value('scene_file') else share/'config/scene.yaml'
    if value('scene') == 'three_floor' and not value('scene_file'):
        raise ValueError('three_floor placement requires a scene_file with live_geometry enabled')
    scene = yaml.safe_load(scene_file.read_text())
    if scene['scene']['name'] != value('scene'):
        raise ValueError('scene_file does not match the selected scene')
    if value('scene') == 'three_floor' and not scene['scene'].get('live_geometry', False):
        raise ValueError('three_floor placement must use live_geometry collision and support checks')
    start = scene['scene']['waypoints'][scene['scene']['task_route'][0]]
    # Preserve the planner task geometry while allowing explicitly selected spawn presets.
    if value('spawn_pose'):
        start = [float(v) for v in value('spawn_pose').split(',')]
        import math
        if len(start) != 4 or not all(math.isfinite(v) for v in start):
            raise ValueError('spawn_pose must contain finite x,y,ground_z,yaw values')
    if any(flags.values()) and start != scene['scene']['waypoints'][scene['scene']['task_route'][0]]:
        raise ValueError('Planning spawn must match the configured task start')
    description = xacro.process_file(
        str(share / 'urdf' / 'robot.urdf.xacro'),
        mappings={'static_display': 'true',
                  'scene_file': str(scene_file),
                  'planning_file': str(share / 'config' / 'planning.yaml'),
                  'executor_library': str(Path(get_package_prefix('sim_bringup')) / 'lib' / 'librobot_executor.so')},
    ).toxml()

    # 由展开后的模型计算出生高度：轮架下降量 + 轮半径。
    # 本章零关节值，轮与轮架的 Z 偏移为零，下层地面高度为 0。
    robot = ET.fromstring(description)
    verify_wheel_geometry(robot, scene['robot'])
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
                'scene_file': str(scene_file),
                'output_dir': LaunchConfiguration('results_dir'),
                'use_sim_time': True,
                'local_enabled': ParameterValue(LaunchConfiguration('local_planning'), value_type=bool),
                'interactive': ParameterValue(LaunchConfiguration('interactive'), value_type=bool),
                'autonomous': ParameterValue(LaunchConfiguration('autonomous'), value_type=bool),
                'planners.global': LaunchConfiguration('global_planner'),
                'planners.wheel': LaunchConfiguration('wheel_planner'),
                'planners.foot': LaunchConfiguration('foot_planner'),
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
        DeclareLaunchArgument('scene_file', default_value=''),
        DeclareLaunchArgument('spawn_pose', default_value='', description='Optional x,y,ground_z,yaw; does not change planner endpoints'),
        DeclareLaunchArgument('interactive', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('global_planner', default_value='astar'),
        DeclareLaunchArgument('wheel_planner', default_value='teb'),
        DeclareLaunchArgument('foot_planner', default_value='footstep'),
        DeclareLaunchArgument('planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('local_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('foot_planning', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autonomous', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('results_dir', default_value=str(Path.home()/'.local/state/simulation/global_planning')),
        OpaqueFunction(function=setup),
    ])
