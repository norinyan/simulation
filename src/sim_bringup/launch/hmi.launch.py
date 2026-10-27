"""Launch the HMI backend and its desktop window, without starting a task."""
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def project_root():
    if os.environ.get('HMI_PROJECT_ROOT'):
        return os.environ['HMI_PROJECT_ROOT']
    for parent in Path(get_package_prefix('sim_bringup')).parents:
        if (parent / 'src/hmi/backend/app.py').is_file():
            return str(parent)
    raise RuntimeError('找不到项目目录，请设置 HMI_PROJECT_ROOT')


def start(context):
    root = Path(LaunchConfiguration('project_root').perform(context)).resolve()
    port = LaunchConfiguration('port').perform(context)
    backend = ExecuteProcess(
        cmd=[sys.executable, str(root / 'src/hmi/backend/app.py')],
        additional_env={'HMI_PROJECT_ROOT': str(root), 'HMI_PORT': port},
        output='screen', sigterm_timeout='35', sigkill_timeout='5',
    )
    desktop = ExecuteProcess(
        cmd=[sys.executable, str(root / 'src/hmi/desktop.py'), '--url', f'http://127.0.0.1:{port}'],
        output='screen',
    )
    return [
        RegisterEventHandler(OnProcessExit(target_action=backend, on_exit=[
            EmitEvent(event=Shutdown(reason='HMI 后端已退出'))])),
        RegisterEventHandler(OnProcessExit(target_action=desktop, on_exit=[
            EmitEvent(event=Shutdown(reason='HMI 窗口已关闭'))])),
        backend, desktop,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('project_root', default_value=project_root()),
        DeclareLaunchArgument('port', default_value='8000'),
        OpaqueFunction(function=start),
    ])
