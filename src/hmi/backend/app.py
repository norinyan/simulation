"""Single-session HMI bridge. All movement and planning data come from ROS."""
import asyncio
import copy
import hashlib
from contextlib import asynccontextmanager
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlparse

import yaml
import psutil
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from maps import global_map, world_map, load_scene, scene_catalog

ROOT = Path(os.environ.get('HMI_PROJECT_ROOT', Path(__file__).resolve().parents[3])).resolve()
FRONTEND = ROOT / 'src/hmi/frontend/dist'
GLOBAL_CONFIG = ROOT / 'src/global_planning/config/global_cfg.yaml'
LOCAL_CONFIG = ROOT / 'src/local_planning/config/local_cfg.yaml'
IDEAL_CONFIG = ROOT / 'src/control/config/ideal.yaml'
DOMAIN = int(os.environ.get('HMI_ROS_DOMAIN_ID', '186'))
MASTER = os.environ.get('HMI_GAZEBO_MASTER_URI', 'http://127.0.0.1:11386')
os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)
os.environ['GAZEBO_MASTER_URI'] = MASTER

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from interfaces.msg import DynamicObstacle, GlobalPath, LocalTrajectory
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Selection(BaseModel):
    map_id: str = 'three-floor'
    start: str = 'F1-ENTRY'
    goal: str = 'F3-EAST'
    global_algorithm: str = 'pct'
    local_algorithm: str = 'scan'
    dynamic_obstacles: bool = True


def selection_dict(value):
    return value.model_dump() if hasattr(value, 'model_dump') else value.dict()


class Session:
    def __init__(self, loop):
        self.loop, self.lock = loop, threading.RLock()
        self.clients = set()
        self.obstacle_versions = {}
        self.state = {'simulation': {'state': 'stopped', 'paused': False, 'reason': ''},
                      'selection': selection_dict(Selection()), 'global_status': {'state': 'idle', 'reason': ''},
                      'local_status': {'state': 'idle', 'reason': '', 'current': 0, 'total': 0},
                      'global_path': None, 'local_trajectory': None, 'local_trajectories': [],
                      'dynamic_obstacles': {},
                      'odom': None, 'received_at': {},
                      'global_timing': None, 'local_timing': None,
                      'resources': {'global': None, 'local': None, 'stamp': time.time()}}
        self.processes, self.logs = {}, {}
        self.samples = {}
        self.directory = None
        self.run_scene_identity = None
        self.operation = asyncio.Lock()
        self.node = RosBridge(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def update(self, key, value, received=False):
        with self.lock:
            self.state[key] = value
            if received:
                self.state['received_at'][key] = time.time()
        self.emit({'type': key, 'data': value, 'received_at': time.time() if received else None})

    def emit(self, event):
        def enqueue():
            for queue in tuple(self.clients):
                if queue.full():
                    # A slow browser reconnects to an authoritative snapshot.
                    try:
                        queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                queue.put_nowait(event)
        self.loop.call_soon_threadsafe(enqueue)

    def simulation_state(self):
        with self.lock:
            return self.state['simulation']['state']

    def running(self):
        return self.simulation_state() == 'running'

    def busy(self):
        state = self.snapshot()
        return state['global_status']['state'] == 'planning' or state['local_status']['state'] in (
            'planning', 'executing', 'waiting', 'avoiding', 'stale')

    def require_running(self):
        if not self.running():
            raise HTTPException(409, '请先启动仿真并等待里程计就绪')

    def validate_selection(self, value):
        scene = selected_scene(value.map_id)
        if value.start not in scene['points'] or value.goal not in scene['points']:
            raise HTTPException(422, '起终点必须来自所选地图的预设点位')
        if value.dynamic_obstacles and not scene['features']['dynamic_obstacles']:
            raise HTTPException(422, '所选地图禁用行人')
        if value.global_algorithm != 'pct' or value.local_algorithm != 'scan':
            raise HTTPException(422, '目前仅接入 PCT / SCAN')
        if value.start == value.goal:
            raise HTTPException(422, '起点和终点不能相同')

    def require_same_scene(self, map_id):
        scene = selected_scene(map_id)
        if scene_identity(scene) != self.run_scene_identity:
            raise HTTPException(409, '地图产物已改变，请停止仿真后重新启动')

    def reset_planning(self):
        with self.lock:
            self.state['local_trajectories'] = []
        for key in ('global_path', 'local_trajectory'):
            self.update(key, None)
        self.update('global_status', {'state': 'idle', 'reason': ''})
        self.update('local_status', {'state': 'idle', 'reason': '', 'current': 0, 'total': 0})
        self.update('global_timing', None)
        self.update('local_timing', None)
        self.samples.clear()
        self.update('resources', {'global': None, 'local': None, 'stamp': time.time()})

    def resources(self):
        """Sample only the actual nodes in this Session's planning process group."""
        now = time.monotonic()
        result = {'global': None, 'local': None, 'stamp': time.time()}
        launch = self.processes.get('planning')
        alive = set()
        if launch and launch.poll() is None:
            for process in psutil.process_iter(['pid', 'name', 'create_time', 'cpu_times', 'memory_info']):
                name = process.info['name']
                key = {'global_node': 'global', 'local_node': 'local'}.get(name)
                if key is None:
                    continue
                try:
                    if os.getpgid(process.pid) != launch.pid:
                        continue
                    identity = (process.pid, process.info['create_time'])
                    alive.add(identity)
                    cpu = process.info['cpu_times']
                    seconds = cpu.user + cpu.system
                    rss = process.info['memory_info'].rss / (1024 * 1024)
                    previous = self.samples.get(identity)
                    percent = max(0., (seconds - previous['seconds']) / (now - previous['time']) * 100) if previous else None
                    peak_cpu = max(previous['peak_cpu'] if previous else 0., percent or 0.)
                    peak_rss = max(previous['peak_rss'] if previous else 0., rss)
                    self.samples[identity] = {'seconds': seconds, 'time': now,
                                              'peak_cpu': peak_cpu, 'peak_rss': peak_rss}
                    timing = self.snapshot()[f'{key}_timing']
                    result[key] = {'pid': process.pid, 'cpu_percent': percent, 'rss_mib': rss,
                                   'cpu_peak_percent': peak_cpu, 'rss_peak_mib': peak_rss,
                                   'planning_ms': timing['elapsed_ms'] if timing else None}
                except (psutil.NoSuchProcess, psutil.AccessDenied, ProcessLookupError):
                    continue
        self.samples = {key: value for key, value in self.samples.items() if key in alive}
        self.update('resources', result)

    def spawn_process(self, name, command):
        path = Path(self.directory.name) / f'{name}.log'
        log = path.open('w')
        self.logs[name] = log
        self.processes[name] = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                                env=dict(os.environ), start_new_session=True)

    async def stop_process(self, name):
        process = self.processes.pop(name, None)
        if process is not None:
            # Signal only the process group created by this Session.
            for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    await asyncio.to_thread(process.wait, timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
                # The launch parent may exit before its Gazebo child. Reap the
                # entire group we created, rather than leave a port-owning orphan.
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
        log = self.logs.pop(name, None)
        if log:
            log.close()

    def write_configs(self, selection, plan):
        config = yaml.safe_load(GLOBAL_CONFIG.read_text())
        scene = selected_scene(selection.map_id)
        config['points'] = scene['points']
        config['planning'].update(start=selection.start, goal=selection.goal, start_source='preset',
                                  algorithm=selection.global_algorithm, plan_on_startup=plan)
        # Moving a copy to /tmp must not change any source-relative map paths.
        for section, key in (('planning', 'map_file'), ('pct', 'world_file'), ('pct', 'ideal_config')):
            value = Path(config[section][key])
            if not value.is_absolute():
                config[section][key] = str((GLOBAL_CONFIG.parent / value).resolve())
        local = yaml.safe_load(LOCAL_CONFIG.read_text())
        if scene['manifest'] is not None:
            config['planning']['map_file'] = str(scene['files']['pctmap'])
            config['pct']['world_file'] = str(scene['files']['world'])
            local['map_file'] = str(scene['files']['npz'])
        local['algorithm'] = selection.local_algorithm
        dynamic = local.setdefault('dynamic_obstacles', {})
        dynamic['enabled'] = selection.dynamic_obstacles
        for key in ('map_file', 'ideal_config'):
            if local.get(key):
                value = Path(local[key])
                if not value.is_absolute():
                    local[key] = str((LOCAL_CONFIG.parent / value).resolve())
        destination = Path(self.directory.name)
        (destination / 'global.yaml').write_text(yaml.safe_dump(config, allow_unicode=True))
        (destination / 'local.yaml').write_text(yaml.safe_dump(local, allow_unicode=True))
        return destination / 'global.yaml', destination / 'local.yaml'

    def start_planners(self, selection, plan):
        global_file, local_file = self.write_configs(selection, plan)
        self.spawn_process('planning', ['ros2', 'launch', 'sim_bringup', 'planning.launch.py',
                                       f'global_config:={global_file}', f'local_config:={local_file}', 'auto_execute:=false'])

    async def wait_until(self, predicate, timeout, reason):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            for name, process in self.processes.items():
                if process.poll() is not None:
                    raise HTTPException(500, f'{name} 启动进程已退出，请检查仿真依赖')
            await asyncio.sleep(.05)
        raise HTTPException(504, reason)

    async def call(self, client, request, timeout=10):
        deadline = time.monotonic()+timeout
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise HTTPException(504, f'ROS 服务未就绪：{client.srv_name}')
            await asyncio.sleep(.05)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                client.remove_pending_request(future)
                raise HTTPException(504, f'ROS 服务调用超时：{client.srv_name}')
            await asyncio.sleep(.02)
        try:
            return future.result()
        except Exception as exc:
            raise HTTPException(500, str(exc)) from exc

    async def spawn_robot(self, selection):
        # Reset only the robot owned by this HMI, never external simulations.
        delete = DeleteEntity.Request(); delete.name = 'ideal_robot'
        await self.call(self.node.delete, delete)
        self.update('odom', None)
        request = SpawnEntity.Request(); request.name = 'ideal_robot'
        request.xml = (ROOT / 'src/control/urdf/ideal_robot.urdf').read_text()
        p = selected_scene(selection.map_id)['points'][selection.start]['pose']
        request.initial_pose.position.x, request.initial_pose.position.y = float(p[0]), float(p[1])
        request.initial_pose.position.z = float(p[2])+float(yaml.safe_load(IDEAL_CONFIG.read_text())['reference_height'])
        request.initial_pose.orientation.z, request.initial_pose.orientation.w = math.sin(p[3]/2), math.cos(p[3]/2)
        response = await self.call(self.node.spawn, request, timeout=20)
        if not response.success:
            raise HTTPException(500, response.status_message)
        await self.wait_until(lambda: self.snapshot()['odom'] is not None, 15, '新模型里程计未就绪')

    async def start(self, selection):
        self.validate_selection(selection)
        if self.snapshot()['simulation']['state'] not in ('stopped', 'failed'):
            raise HTTPException(409, '本次仿真已启动')
        if self.processes or self.directory:
            await self.stop()
        uri = urlparse(MASTER)
        with socket.socket() as probe:
            if probe.connect_ex((uri.hostname or '127.0.0.1', uri.port or 11386)) == 0:
                raise HTTPException(409, '专用 Gazebo 端口已被占用，未接管外部仿真')
        self.directory = tempfile.TemporaryDirectory(prefix='norin-hmi-')
        self.update('selection', selection_dict(selection))
        self.reset_planning()
        self.update('odom', None)
        self.update('dynamic_obstacles', {})
        self.obstacle_versions.clear()
        self.update('simulation', {'state': 'starting', 'paused': False, 'reason': ''})
        scene = selected_scene(selection.map_id)
        pose = scene['points'][selection.start]['pose']
        self.run_scene_identity = scene_identity(scene)
        height = yaml.safe_load(IDEAL_CONFIG.read_text())['reference_height']
        try:
            self.spawn_process('simulation', ['ros2', 'launch', 'sim_bringup', 'sim.launch.py', 'gui:=false',
                                               f'x:={pose[0]}', f'y:={pose[1]}', f'z:={pose[2]+height}',
                                               f'yaw:={pose[3]}', f'ideal_config:={IDEAL_CONFIG}',
                                               f'world:={scene["files"]["world"]}',
                                               f'dynamic_obstacles:={str(selection.dynamic_obstacles).lower()}'])
            await self.wait_until(lambda: self.snapshot()['odom'] is not None, 35, '等待模型里程计超时')
            self.start_planners(selection, False)
            await self.wait_until(lambda: self.node.start_task.service_is_ready(), 20, '等待局部规划节点超时')
            self.update('simulation', {'state': 'running', 'paused': False, 'reason': ''})
        except Exception as exc:
            await self.stop()
            self.update('simulation', {'state': 'failed', 'paused': False, 'reason': str(getattr(exc, 'detail', exc))})
            raise
        return self.snapshot()

    async def stop(self):
        self.update('simulation', {'state': 'stopping', 'paused': False, 'reason': ''})
        await self.stop_process('planning')
        await self.stop_process('simulation')
        self.reset_planning()
        self.update('odom', None)
        self.update('dynamic_obstacles', {})
        self.obstacle_versions.clear()
        if self.directory:
            self.directory.cleanup(); self.directory = None
        self.run_scene_identity = None
        self.update('simulation', {'state': 'stopped', 'paused': False, 'reason': ''})
        return self.snapshot()

    async def plan(self, selection):
        self.require_running(); self.validate_selection(selection)
        if selection.map_id != self.snapshot()['selection']['map_id']:
            raise HTTPException(409, '切换地图需先停止仿真')
        self.require_same_scene(selection.map_id)
        if selection.dynamic_obstacles != self.snapshot()['selection']['dynamic_obstacles']:
            raise HTTPException(409, '切换动态障碍需先停止并重新启动仿真')
        if self.busy():
            raise HTTPException(409, '规划或执行中不能修改任务')
        if self.snapshot()['simulation']['paused']:
            raise HTTPException(409, '请先恢复仿真再开始全局规划')
        await self.stop_process('planning')
        # Drain the previous publishers before invalidating their cached data.
        await asyncio.sleep(.1)
        self.reset_planning()
        self.update('selection', selection_dict(selection))
        await self.spawn_robot(selection)
        self.start_planners(selection, True)
        return self.snapshot()

    async def task(self):
        self.require_running()
        state = self.snapshot()
        self.validate_selection(Selection(**state['selection']))
        self.require_same_scene(state['selection']['map_id'])
        if state['simulation']['paused']:
            raise HTTPException(409, '请先恢复仿真')
        if not state['global_path'] or not state['global_path']['points'] or state['global_status']['state'] != 'ready':
            raise HTTPException(409, '尚无本次成功的全局路径')
        if state['local_status']['state'] != 'ready':
            raise HTTPException(409, '局部规划器未就绪或任务已经启动')
        if time.time()-state['received_at'].get('odom', 0) > 1.0:
            raise HTTPException(409, '里程计未连接或已过期')
        response = await self.call(self.node.start_task, Trigger.Request())
        if not response.success:
            raise HTTPException(409, response.message)
        return {'accepted': True, 'reason': response.message}

    async def pause(self, value):
        self.require_running()
        process = await asyncio.create_subprocess_exec('gz', 'world', '-p', '1' if value else '0',
                                                       env=dict(os.environ), stdout=asyncio.subprocess.PIPE,
                                                       stderr=asyncio.subprocess.PIPE)
        try:
            _, error = await asyncio.wait_for(process.communicate(), timeout=5)
        except asyncio.TimeoutError:
            process.kill(); await process.wait()
            raise HTTPException(504, 'Gazebo 暂停/恢复超时')
        if process.returncode:
            raise HTTPException(500, error.decode(errors='replace')[-500:])
        self.update('simulation', {'state': 'running', 'paused': value, 'reason': ''})
        return self.snapshot()

    async def monitor(self):
        while True:
            await asyncio.sleep(.5)
            if not hasattr(self, 'last_sample') or time.monotonic() - self.last_sample >= 1.:
                self.last_sample = time.monotonic()
                self.resources()
            if self.snapshot()['simulation']['state'] != 'running':
                continue
            dead = [name for name, process in self.processes.items() if process.poll() is not None]
            if dead:
                self.update('simulation', {'state': 'failed', 'paused': False, 'reason': f'进程退出：{", ".join(dead)}'})

    async def close(self):
        await self.stop()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.node.destroy_node()


class RosBridge(Node):
    def __init__(self, session):
        super().__init__('hmi_bridge')
        self.session, self.last_odom = session, 0.
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GlobalPath, '/global_path', lambda msg: self.receive('global_path', msg), retained)
        self.create_subscription(LocalTrajectory, '/local_trajectory', lambda msg: self.receive('local_trajectory', msg), 10)
        self.create_subscription(Odometry, '/robot/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(DynamicObstacle, '/dynamic_obstacles', self.obstacle, 128)
        self.create_subscription(String, '/global_planning/status', lambda msg: self.status('global_status', msg), retained)
        self.create_subscription(String, '/local_planning/status', lambda msg: self.status('local_status', msg), retained)
        self.create_subscription(String, '/global_planning/timing', lambda msg: self.timing('global_timing', msg), retained)
        self.create_subscription(String, '/local_planning/timing', lambda msg: self.timing('local_timing', msg), retained)
        self.start_task = self.create_client(Trigger, '/local_node/start_task')
        self.spawn = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete = self.create_client(DeleteEntity, '/delete_entity')

    def receive(self, key, msg):
        if self.session.simulation_state() in ('starting', 'running'):
            data = message_to_ordereddict(msg)
            if key == 'local_trajectory':
                with self.session.lock:
                    history = self.session.state['local_trajectories']
                    stamp = data['header']['stamp']
                    history = [item for item in history if item['header']['stamp'] != stamp]
                    self.session.state['local_trajectories'] = (history + [data])[-2:]
            self.session.update(key, data, received=True)

    def odom(self, msg):
        now = time.monotonic()
        if now-self.last_odom >= .05:
            self.last_odom = now
            self.receive('odom', msg)

    def obstacle(self, msg):
        if self.session.simulation_state() not in ('starting', 'running'):
            return
        if not msg.id or msg.header.frame_id != 'world':
            return
        data = message_to_ordereddict(msg)
        with self.session.lock:
            if not self.session.state['selection'].get('dynamic_obstacles'):
                return
            obstacles = dict(self.session.state['dynamic_obstacles'])
            at = lambda item: (item['header']['stamp']['sec'], item['header']['stamp']['nanosec'])
            timestamp = at(data)
            current = self.session.state.get('odom')
            now = (current['header']['stamp']['sec'] + current['header']['stamp']['nanosec']*1e-9
                   if current else timestamp[0]+timestamp[1]*1e-9)
            if timestamp[0]+timestamp[1]*1e-9 < now-1.0:
                return
            versions = self.session.obstacle_versions
            previous = versions.get(msg.id)
            if previous and (timestamp < previous[0] or
                             timestamp == previous[0] and (msg.active or not previous[1])):
                return
            versions[msg.id] = (timestamp, msg.active)
            if msg.active:
                obstacles[msg.id] = data
            else:
                obstacles.pop(msg.id, None)
            for key, (when, active) in list(versions.items()):
                if not active and now-(when[0]+when[1]*1e-9) > 1.0:
                    del versions[key]
        self.session.update('dynamic_obstacles', obstacles, received=True)

    def status(self, key, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict) and isinstance(data.get('state'), str):
            if self.session.simulation_state() in ('starting', 'running'):
                self.session.update(key, data, received=True)

    def timing(self, key, msg):
        try:
            data = json.loads(msg.data)
            elapsed = data['elapsed_ms']
            if isinstance(elapsed, bool) or not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed) or elapsed < 0:
                return
            if not isinstance(data['success'], bool):
                return
        except (ValueError, TypeError, KeyError):
            return
        if self.session.simulation_state() in ('starting', 'running'):
            self.session.update(key, {**data, 'received_at': time.time()}, received=True)


session = None


@asynccontextmanager
async def lifespan(app):
    global session
    # Uvicorn owns process signals and closes the ROS executor in lifespan.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    session = Session(asyncio.get_running_loop())
    watcher = asyncio.create_task(session.monitor())
    try:
        yield
    finally:
        watcher.cancel()
        await session.close()
        rclpy.try_shutdown()


app = FastAPI(title='Simulation HMI', lifespan=lifespan)


def selected_scene(map_id):
    try:
        return load_scene(map_id, GLOBAL_CONFIG)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise HTTPException(503, f'所选地图不可用：{exc}') from exc


def scene_identity(scene):
    if scene['manifest'] is not None:
        # Validation above checks every artifact against this manifest. Pin the
        # whole generation while Gazebo and planners retain loaded resources.
        return (scene['id'], hashlib.sha256(scene['manifest'].read_bytes()).hexdigest())
    return (scene['id'], GLOBAL_CONFIG.read_bytes(), tuple(
        (key, str(path), path.stat().st_size, path.stat().st_mtime_ns)
        for key, path in sorted(scene['files'].items())))


def read_config(map_id=None):
    selection = session.snapshot()['selection']
    scene = selected_scene(map_id or selection['map_id'])
    try:
        maps = scene_catalog(GLOBAL_CONFIG)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise HTTPException(503, f'地图目录不可用：{exc}') from exc
    return {'points': [{'id': key, 'label': item.get('label', key), 'pose': item['pose']}
                       for key, item in scene['points'].items()],
            'map_id': scene['id'], 'maps': maps,
            'algorithms': {'global': [{'id': item, 'available': item == 'pct'} for item in ('pct', 'mesh', 'gb')],
                           'local': [{'id': item, 'available': item == 'scan'} for item in ('scan', 'teb', 'ego', 'mppi')]},
            'reference_height': yaml.safe_load(IDEAL_CONFIG.read_text())['reference_height'],
            'selection': selection, 'topic_hz': 20}


@app.get('/api/config')
async def config(map_id: str = None):
    return await asyncio.to_thread(read_config, map_id)


class MapSelection(BaseModel):
    map_id: str


@app.post('/api/map/select')
async def select_map(value: MapSelection):
    async with session.operation:
        if session.simulation_state() != 'stopped':
            raise HTTPException(409, '停止仿真后才能切换地图')
        scene = selected_scene(value.map_id)
        selection = Selection(map_id=scene['id'], start=scene['start'], goal=scene['goal'],
                              dynamic_obstacles=scene['features']['dynamic_obstacles'])
        session.validate_selection(selection)
        session.reset_planning()
        session.update('odom', None)
        session.update('dynamic_obstacles', {})
        session.obstacle_versions.clear()
        session.update('selection', selection_dict(selection))
        return session.snapshot()


map_cache = {}


def map_data(kind, map_id=None):
    scene = selected_scene(map_id or session.snapshot()['selection']['map_id'])
    source = scene['files']['pctmap' if kind == 'global' else 'world']
    try:
        key = (scene['id'], kind)
        marker = (str(source), source.stat().st_mtime_ns, scene.get('geometry_fingerprint'))
        if map_cache.get(key, {}).get('marker') != marker:
            map_cache[key] = {'marker': marker, 'data': global_map(source) if kind == 'global' else world_map(source, scene)}
        return map_cache[key]['data']
    except (OSError, ValueError, struct_error) as exc:
        raise HTTPException(503, f'实际地图不可用：{exc}') from exc


from struct import error as struct_error


@app.get('/api/map/world')
async def world(map_id: str = None):
    return await asyncio.to_thread(map_data, 'world', map_id)


@app.get('/api/map/global')
async def offline(map_id: str = None):
    return await asyncio.to_thread(map_data, 'global', map_id)


@app.get('/api/map/mesh')
async def mesh(map_id: str):
    scene = await asyncio.to_thread(selected_scene, map_id)
    if 'mesh' not in scene['files']:
        raise HTTPException(404, '所选地图没有网格资源')
    return FileResponse(scene['files']['mesh'], media_type='model/vnd.collada+xml')


@app.post('/api/simulation/start')
async def start(value: Selection = Selection()):
    async with session.operation:
        return await session.start(value)


@app.post('/api/simulation/stop')
async def stop():
    async with session.operation:
        return await session.stop()


@app.post('/api/simulation/pause')
async def pause():
    async with session.operation:
        return await session.pause(True)


@app.post('/api/simulation/resume')
async def resume():
    async with session.operation:
        return await session.pause(False)


@app.post('/api/planning/global')
async def plan(value: Selection):
    async with session.operation:
        return await session.plan(value)


@app.post('/api/task/start')
async def task():
    async with session.operation:
        return await session.task()


@app.websocket('/ws')
async def websocket(socket: WebSocket):
    await socket.accept()
    queue = asyncio.Queue(maxsize=128)
    session.clients.add(queue)
    try:
        await socket.send_json({'type': 'snapshot', 'data': session.snapshot()})
        # Receive disconnects even when ROS is stopped and there are no events.
        async def receive():
            while True:
                await socket.receive_text()
        reader = asyncio.create_task(receive())
        while True:
            pending = asyncio.create_task(queue.get())
            done, _ = await asyncio.wait((reader, pending), return_when=asyncio.FIRST_COMPLETED)
            if reader in done:
                pending.cancel()
                await reader
                break
            await socket.send_json(pending.result())
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        session.clients.discard(queue)
        if 'reader' in locals():
            reader.cancel()


if FRONTEND.is_dir():
    app.mount('/assets', StaticFiles(directory=FRONTEND / 'assets'), name='assets')


@app.get('/')
async def index():
    if not (FRONTEND / 'index.html').is_file():
        raise HTTPException(503, '请先构建 src/hmi/frontend')
    return FileResponse(FRONTEND / 'index.html')


if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host=os.environ.get('HMI_HOST', '0.0.0.0'), port=int(os.environ.get('HMI_PORT', '8000')))
