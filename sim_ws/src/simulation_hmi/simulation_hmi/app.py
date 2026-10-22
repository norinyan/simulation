"""Local simulation control panel. Launch composition stays in sim_bringup."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import re
import socket
import signal
import sys
import time
from urllib.parse import urlparse

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (QApplication, QComboBox, QFormLayout, QGroupBox,
                            QHBoxLayout, QLabel, QMainWindow, QPushButton,
                            QPlainTextEdit, QVBoxLayout, QWidget)
from ament_index_python.packages import get_package_share_directory
import rclpy
from gazebo_msgs.srv import GetModelList, DeleteEntity

from .presets import load_presets, task_supported, atomic_yaml, placement_scene, check_static_placement
from .ros_bridge import RosBridge
from .session import SessionProcess


class ControlPanel(QMainWindow):
    def __init__(self, master_uri, headless=False):
        super().__init__()
        self.master_uri, self.headless = master_uri, headless
        self.setWindowTitle('机器人仿真控制台')
        self.resize(660, 850)
        self.share = Path(get_package_share_directory('sim_bringup'))
        self.presets = load_presets(Path(get_package_share_directory('simulation_hmi'))/'config/presets.yaml')
        self.sessions = {}
        self.running = self.ready = self.spawned = self.spawning = self.stopping = False
        self.removing = self.closing = self.plan_ready = self.task_active = self.task_used = False
        self.busy = False
        self.generation = 0
        self.stop_requested = False
        self.task_cancelled = False
        self.task_id = 0
        self.stop_acknowledged = False
        self.session_dir = None
        self.last_world_query = 0
        self.querying = False
        self.last_status = {}
        self.scene_started = self.robot_started = 0
        self.actual_pose = None
        self.build_ui()
        self.ros = RosBridge(self)
        self.ros.status.connect(self.status_received)
        self.ros.pose.connect(self.pose_received)
        self.ros.clock.connect(lambda t: self.clock_label.setText(f'仿真时间：{t:.1f} s'))
        self.scene_combo.currentIndexChanged.connect(self.scene_selected)
        self.start_point.currentIndexChanged.connect(self.selection_changed)
        self.goal_point.currentIndexChanged.connect(self.selection_changed)
        self.view.currentIndexChanged.connect(self.write_view)
        self.floor.currentIndexChanged.connect(self.write_view)
        self.start_button.clicked.connect(self.start_scene)
        self.close_button.clicked.connect(self.close_scene)
        self.spawn_button.clicked.connect(self.spawn_robot)
        self.remove_button.clicked.connect(self.remove_robot)
        self.execute_button.clicked.connect(self.start_task)
        self.stop_button.clicked.connect(self.stop_robot)
        self.apply_view.clicked.connect(self.write_view)
        self.follow_button.clicked.connect(self.follow)
        self.scene_selected()
        self.timer = QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.message('先选择场景并启动仿真；Gazebo 画面在独立窗口显示。')

    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        title = QLabel('机器人仿真控制台')
        title.setStyleSheet('font-size:22px;font-weight:600;padding:8px 0;')
        layout.addWidget(title)
        scene_box = QGroupBox('1 · 仿真场景')
        form = QFormLayout(scene_box)
        self.scene_combo = QComboBox()
        for name, scene in self.presets.items():
            self.scene_combo.addItem(scene['label'], name)
        self.scene_combo.setCurrentIndex(self.scene_combo.findData('three_floor'))
        self.factor = QComboBox()
        for label, value in [('正常速度 · 1 倍', 1), ('加速观察 · 5 倍', 5), ('加速观察 · 10 倍', 10)]:
            self.factor.addItem(label, value)
        form.addRow('场景', self.scene_combo)
        form.addRow('仿真速度', self.factor)
        self.start_button, self.close_button = QPushButton('启动仿真'), QPushButton('关闭仿真')
        buttons = QHBoxLayout()
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.close_button)
        form.addRow(buttons)
        layout.addWidget(scene_box)
        robot_box = QGroupBox('2 · 机器人与预设点位')
        form = QFormLayout(robot_box)
        self.start_point, self.goal_point = QComboBox(), QComboBox()
        self.coordinates = QLabel()
        self.coordinates.setWordWrap(True)
        self.route_note = QLabel()
        self.route_note.setWordWrap(True)
        form.addRow('出生点', self.start_point)
        form.addRow('目标点', self.goal_point)
        form.addRow(self.coordinates)
        self.spawn_button, self.remove_button = QPushButton('生成机器人'), QPushButton('移除机器人 / 重选出生点')
        buttons = QHBoxLayout()
        buttons.addWidget(self.spawn_button)
        buttons.addWidget(self.remove_button)
        form.addRow(buttons)
        form.addRow(self.route_note)
        layout.addWidget(robot_box)
        task_box = QGroupBox('3 · 规划与执行')
        form = QVBoxLayout(task_box)
        algorithms = QFormLayout()
        self.algorithms = {}
        for key, label, title, value in [('global', '全局规划', 'A*', 'astar'), ('wheel', '轮式局部', 'TEB', 'teb'), ('foot', '足式局部', 'Footstep', 'footstep')]:
            combo = QComboBox();combo.addItem(title, value)
            self.algorithms[key] = combo;algorithms.addRow(label, combo)
        form.addLayout(algorithms)
        buttons = QHBoxLayout()
        self.execute_button, self.stop_button = (QPushButton(x) for x in ('开始任务', '停止任务'))
        for button in (self.execute_button, self.stop_button):
            buttons.addWidget(button)
        form.addLayout(buttons)
        layout.addWidget(task_box)
        view_box = QGroupBox('4 · 观察视角')
        form = QFormLayout(view_box)
        self.view, self.floor = QComboBox(), QComboBox()
        for label, value in [('总览', 'exterior'), ('俯视', 'top'), ('内部透视', 'interior'), ('三层同屏', 'split')]:
            self.view.addItem(label, value)
        for label, value in [('全部', 0), ('F1', 1), ('F2', 2), ('F3', 3)]:
            self.floor.addItem(label, value)
        form.addRow('视图', self.view)
        form.addRow('楼层', self.floor)
        self.apply_view, self.follow_button = QPushButton('恢复所选视角'), QPushButton('跟随机器人')
        buttons = QHBoxLayout()
        buttons.addWidget(self.apply_view)
        buttons.addWidget(self.follow_button)
        form.addRow(buttons)
        layout.addWidget(view_box)
        self.state_label, self.clock_label, self.pose_label = QLabel(), QLabel('仿真时间：—'), QLabel('机器人位置：—')
        for widget in (self.state_label, self.clock_label, self.pose_label):
            widget.setWordWrap(True)
            layout.addWidget(widget)
        self.messages = QPlainTextEdit()
        self.messages.setReadOnly(True)
        self.messages.setMaximumBlockCount(500)
        self.messages.setMinimumHeight(140)
        layout.addWidget(self.messages, 1)
        self.setStyleSheet('QGroupBox{margin-top:8px;font-weight:600;} QComboBox,QPushButton{min-height:28px;}')

    def message(self, text):
        text = re.sub(r'\x1b\[[0-9;]*m', '', str(text))
        self.messages.appendPlainText(datetime.now().strftime('%H:%M:%S')+'  '+text)

    def scene(self):
        return self.scene_combo.currentData()

    def selected_pose(self, combo):
        return self.presets[self.scene()]['points'][combo.currentData()]['pose']

    def supported(self):
        return task_supported(self.presets[self.scene()], self.start_point.currentData(), self.goal_point.currentData())

    def scene_selected(self):
        if self.running:
            return
        scene = self.presets[self.scene()]
        for combo, default in ((self.start_point, 'default_start'), (self.goal_point, 'default_goal')):
            combo.blockSignals(True)
            combo.clear()
            for key, point in scene['points'].items():
                combo.addItem(point['label'], key)
            combo.setCurrentIndex(combo.findData(scene[default]))
            combo.blockSignals(False)
        self.factor.setCurrentIndex(0)
        self.view.setCurrentIndex(0)
        self.floor.setCurrentIndex(0)
        for i in range(self.view.count()):
            self.view.model().item(i).setEnabled(self.scene() == 'three_floor' or i < 2)
        self.floor.setEnabled(self.scene() == 'three_floor')
        self.selection_changed()

    def selection_changed(self):
        if self.start_point.currentData() is None or self.goal_point.currentData() is None:
            return
        start, goal = self.selected_pose(self.start_point), self.selected_pose(self.goal_point)
        self.coordinates.setText(f'出生地面坐标：{tuple(start[:3])} m   朝向：{start[3]:.2f} rad\n目标地面坐标：{tuple(goal[:3])} m')
        self.route_note.setText(('三层楼全部不同点对；轮式／足式由全局路线决定。' if self.scene() == 'three_floor'
                                  else '可执行任务：A → E → X → B。') if self.supported() else '请选择支持的不同起终点。')
        if self.spawned and not self.busy and not self.task_active:
            self.write_task()
        self.plan_ready = False
        self.write_view()
        self.update_buttons()

    def launch(self, key, command):
        process = SessionProcess(key, self.session_dir, {'GAZEBO_MASTER_URI': self.master_uri}, self)
        self.sessions[key] = process
        process.message.connect(self.message)
        process.ended.connect(lambda code, key=key, process=process: self.process_ended(key, process, code))
        process.start(command)

    def start_scene(self):
        if self.running or self.stopping:
            return
        address = urlparse(self.master_uri)
        try:
            with socket.socket() as probe:
                probe.bind((address.hostname, address.port))
            # Refuse to attach to any preexisting Gazebo ROS session in this domain.
            if self.ros.world.service_is_ready():
                raise ValueError('当前 ROS 通信域已有 Gazebo 服务，请更换 HMI domain_id')
        except (OSError, ValueError) as error:
            self.message(f'不能启动：端口或通信域已占用。{error}')
            return
        self.session_dir = Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state')))/'simulation_hmi'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.session_dir.mkdir(parents=True)
        self.running, self.ready = True, False
        self.scene_started = time.monotonic()
        self.last_status.clear()
        self.ros.last_clock = self.ros.last_odom = 0
        self.write_view()
        self.launch('scene', ['ros2', 'launch', 'sim_bringup', 'scene.launch.py', 'scene:='+self.scene(),
                              'view:=exterior', 'floor:=0', 'master_uri:='+self.master_uri,
                              'gui:='+str(not self.headless).lower(),
                              'real_time_factor:='+str(self.factor.currentData()), 'output_dir:='+str(self.session_dir)])
        self.message(f'正在启动{self.presets[self.scene()]["label"]}。本次日志：{self.session_dir}')
        self.update_buttons()

    def write_view(self, *_args, follow=False):
        view, floor = self.view.currentData(), self.floor.currentData()
        if self.scene() == 'two_steps':
            floor = 0
        elif view == 'split':
            floor = 0
        elif view in ('top', 'interior') and floor == 0:
            floor = 1
        if floor != self.floor.currentData():
            self.floor.blockSignals(True)
            self.floor.setCurrentIndex(self.floor.findData(floor))
            self.floor.blockSignals(False)
        self.floor.setEnabled(self.scene() == 'three_floor' and view != 'split')
        if not self.session_dir or not self.running:
            return
        atomic_yaml(self.session_dir/'viewer.yaml', dict(
            view=view, floor=floor, follow=follow, robot_generation=self.generation,
            start=self.selected_pose(self.start_point), goal=self.selected_pose(self.goal_point)))

    def follow(self):
        if self.spawned:
            self.view.blockSignals(True)
            self.view.setCurrentIndex(self.view.findData('exterior'))
            self.view.blockSignals(False)
            if self.scene() == 'three_floor' and self.actual_pose:
                level = min(3, max(1, round((self.actual_pose.pose.pose.position.z-.32)/3.2)+1))
                self.floor.blockSignals(True)
                self.floor.setCurrentIndex(self.floor.findData(level))
                self.floor.blockSignals(False)
            self.write_view(follow=True)
            self.message('已请求第三人称跟随视角；恢复所选视角可退出跟随。')

    def spawn_robot(self):
        if not self.ready or self.spawned or self.spawning:
            return
        try:
            pose = self.selected_pose(self.start_point)
            check_static_placement(self.share/'worlds'/f'{self.scene()}.world', pose)
            self.scene_file = self.session_dir/'robot_scene.yaml'
            self.write_task()
        except (OSError, ValueError) as error:
            self.message('生成被拒绝：'+str(error))
            return
        self.spawning, self.robot_started = True, time.monotonic()
        self.ros.last_odom = 0
        self.last_status.pop('/robot/execution_status', None)
        self.launch('robot', ['ros2', 'launch', 'sim_bringup', 'robot.launch.py', 'scene:='+self.scene(),
                              'master_uri:='+self.master_uri, 'scene_file:='+str(self.scene_file)])
        self.message('正在生成机器人，等待实际里程计与执行器状态。')
        self.update_buttons()

    def write_task(self):
        placement_scene(self.share/'config/scene.yaml', self.scene(), self.selected_pose(self.start_point), self.scene_file,
                        goal=self.selected_pose(self.goal_point), world_file=self.session_dir/'runtime.world')

    def factor_supported(self):
        return self.factor.currentData() <= self.presets[self.scene()].get('max_navigation_factor', 1)

    def start_planner(self):
        self.task_text = '地图初始化中'
        self.launch('planner', ['ros2', 'run', 'sim_bringup', 'planning_node', '--ros-args',
                                '--params-file', str(self.share/'config/planning.yaml'),
                                '-p', 'scene_file:='+str(self.scene_file),
                                '-p', 'output_dir:='+str(self.session_dir/'planning'),
                                '-p', 'use_sim_time:=true', '-p', 'local_enabled:=true',
                                '-p', 'foot_enabled:=true',
                                *sum((['-p', 'planners.'+key+':='+combo.currentData()] for key, combo in self.algorithms.items()), []), '-p', 'autonomous:=false', '-p', 'interactive:=true'])

    def task_callback(self, callback):
        task_id = self.task_id
        return lambda response, error: callback(response, error) if task_id == self.task_id else None

    def start_task(self):
        if not self.spawned or not self.supported() or self.busy or self.task_active:
            return
        if not self.factor_supported():
            self.message('当前场景不支持此闭环倍率，请降低倍率后重新启动。');return
        self.task_id += 1
        self.stop_requested = self.task_cancelled = False
        self.busy, self.plan_ready = True, False
        self.task_text = '冻结算法配置'
        self.task_started_wall = time.monotonic()
        self.task_algorithms = {key: combo.currentData() for key, combo in self.algorithms.items()}
        self.write_task()
        self.ros.select_algorithms(self.task_algorithms, self.task_callback(self.algorithms_selected))
        self.update_buttons()

    def algorithms_selected(self, response, error):
        if self.task_cancelled:
            return
        if not response or not response.result.successful:
            self.busy = False;self.task_text = '算法选择失败'
            self.message(error or response.result.reason);self.update_buttons();return
        self.task_text = '全局规划中'
        self.message('开始任务：'+str(self.task_algorithms))
        self.ros.command('plan', self.task_callback(self.plan_response), timeout=65)
        self.update_buttons()

    def plan_response(self, response, error):
        self.busy = False
        self.plan_ready = bool(response and response.success and not self.task_cancelled)
        if not self.task_cancelled:
            self.task_text = '规划就绪' if self.plan_ready else '规划未就绪'
        if not self.plan_ready:
            self.task_active = False
        self.message(response.message if response else error)
        self.update_buttons()
        if self.plan_ready:
            self.execute()

    def execute(self):
        if not self.plan_ready or self.busy or self.stop_requested:
            return
        self.busy = True
        self.ros.command('execute', self.task_callback(self.execute_response))
        self.update_buttons()

    def execute_response(self, response, error):
        self.busy = False
        if response and response.success:
            self.task_active = self.task_used = True
            self.plan_ready = False
            self.task_text = '执行请求已接受'
        else:
            self.task_active = False
        self.message(response.message if response else error)
        self.update_buttons()

    def stop_robot(self):
        self.task_id += 1
        self.task_cancelled = True
        self.ros.stop_robot()
        self.plan_ready = False
        self.task_used = True
        self.stop_requested = True
        self.stop_acknowledged = False
        if 'planner' in self.sessions and self.ros.clients['stop'].service_is_ready():
            self.ros.command('stop', self.task_callback(self.stop_response))
        self.message('已发送停止请求，等待实际速度为零；足式安全姿态保留，重新开始须满足实际轮式起点状态。')
        self.update_buttons()

    def stop_response(self, response, error):
        self.stop_acknowledged = bool(response and response.success)
        self.message(response.message if response else error)
        self.complete_stop()
        if not response or not response.success:
            # A failed task service must not leave a scheduler sending motion commands.
            if 'planner' in self.sessions:
                self.sessions['planner'].stop()
            self.ros.stop_robot()
            QTimer.singleShot(300, self.ros.stop_robot)

    def remove_robot(self):
        if self.task_active or self.removing or not self.spawned:
            return
        self.removing = True
        self.ros.stop_robot()
        self.ros.request(self.ros.delete, DeleteEntity.Request(name='wheel_leg_robot'), self.removed)
        self.update_buttons()

    def removed(self, response, error):
        self.removing = False
        if not response or not response.success:
            self.message('移除失败：'+(error or response.status_message))
            return
        for name in ('planner', 'robot'):
            if name in self.sessions:
                self.sessions[name].stop()
        self.spawned = self.spawning = self.plan_ready = self.task_used = self.stop_requested = False
        self.actual_pose = None
        self.last_status.clear()
        self.generation += 1
        self.write_view()
        self.message('机器人已移除，等待节点退出后可重新选点生成。')
        self.update_buttons()

    def status_received(self, topic, text):
        if not self.running or (not self.spawning and not self.spawned):
            return
        if self.last_status.get(topic) != text:
            self.last_status[topic] = text
            self.message(text)
        if topic == '/planning/task_status':
            state = text.split(';')[0]
            labels = {'GLOBAL_PLANNING':'全局规划中', 'FOOT_PLANNING':'足式规划中', 'DEPLOY':'展腿中', 'FOLD':'收腿中', 'DONE':'到达并停车', 'FAILED':'任务失败', 'STOPPED':'任务已停止', 'WAIT_OBSTACLE':'等待障碍物', 'WAIT_EXECUTOR':'等待执行器'}
            self.task_text = labels.get(state, '足式执行中' if 'phase=foot_execution' in text else '轮式规划与执行' if 'phase=wheel_execution' in text else state)
            self.task_active = not text.startswith(('DONE', 'FAILED', 'STOPPED'))
            if not self.task_active:
                self.busy = False
                if hasattr(self, 'task_started_wall'):
                    self.message(f'任务墙钟耗时：{time.monotonic()-self.task_started_wall:.2f} s')
                if text.startswith('DONE') and self.scene() == 'three_floor':
                    self.start_point.setCurrentIndex(self.start_point.findData(self.goal_point.currentData()))
        if topic == '/robot/execution_status' and text.startswith('FAILED'):
            self.task_text = '执行器保护停止'
            self.plan_ready = False
            self.task_used = True
        self.update_buttons()

    def pose_received(self, msg):
        if not self.running or (not self.spawning and not self.spawned):
            return
        self.actual_pose = msg
        p = msg.pose.pose.position
        self.pose_label.setText(f'机器人位置：({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m  速度：{msg.twist.twist.linear.x:.3f} m/s')
        if self.spawning and '/robot/execution_status' in self.last_status:
            self.spawning, self.spawned = False, True
            self.task_used = self.last_status['/robot/execution_status'].startswith('FAILED')
            self.generation += 1
            self.write_view()
            self.message('已收到机器人实际反馈。')
            if not self.task_used and (self.scene() == 'three_floor' or self.start_point.currentData() == 'A'):
                self.start_planner()
        self.complete_stop()
        self.update_buttons()

    def complete_stop(self):
        if not self.stop_requested or not self.stop_acknowledged or self.actual_pose is None:
            return
        velocity = self.actual_pose.twist.twist
        if max(abs(getattr(axis, key)) for axis in (velocity.linear, velocity.angular) for key in ('x', 'y', 'z')) < .001:
            self.busy = self.task_active = self.stop_requested = False
            self.task_text = '已停止（服务确认，实际速度为零）'
            self.update_buttons()

    def tick(self):
        now = time.monotonic()
        if self.running and not self.stopping:
            if not self.ready and now-self.scene_started > 40:
                self.message('场景启动超时，正在清理本次进程。')
                self.close_scene()
            elif self.ready and now-self.ros.last_clock > 5:
                self.message('仿真时钟反馈丢失，正在关闭本次会话。')
                self.close_scene()
            elif not self.querying and now-self.last_world_query > 1 and self.ros.world.service_is_ready():
                self.querying = True
                self.last_world_query = now
                self.ros.request(self.ros.world, GetModelList.Request(), self.world_response, timeout=4)
            if self.spawning and now-self.robot_started > 25:
                self.spawning = False
                self.message('机器人生成超时；请关闭仿真后重试。')
        self.update_buttons()
        if self.closing and not self.sessions:
            self.close()

    def world_response(self, response, error):
        self.querying = False
        if self.stopping or not self.running:
            return
        if not response or not response.success or time.monotonic()-self.ros.last_clock > 3:
            return
        expected = 'f1_wall_001' if self.scene() == 'three_floor' else 'lower_ground'
        names = response.model_names
        # The two_steps slab's actual model name is obtained from the installed World.
        if self.scene() == 'two_steps':
            import xml.etree.ElementTree as ET
            expected = ET.parse(self.share/'worlds/two_steps.world').getroot().find('world/model').get('name')
        if expected not in names:
            self.message('当前 ROS 服务的场景与选择不符，拒绝继续。')
            self.close_scene()
            return
        if not self.ready:
            self.ready = True
            self.message(f'场景已就绪：服务端返回 {len(names)} 个模型，仿真时钟正常。')
            self.update_buttons()

    def close_scene(self):
        if not self.running or self.stopping:
            return
        self.stopping = True
        self.ros.stop_robot()
        if self.task_active and self.ros.clients['stop'].service_is_ready():
            self.ros.command('stop', lambda *_: None)
        # Stop the planner first; no new motion commands may outlive the server.
        for key in ('planner', 'robot', 'scene'):
            if key in self.sessions:
                self.sessions[key].stop()
        self.message('正在关闭本次仿真，等待所属进程退出。')
        self.update_buttons()

    def process_ended(self, key, process, code):
        if self.sessions.get(key) is process:
            del self.sessions[key]
        self.message(f'{key} 进程已退出（{code}）。')
        if key in ('planner', 'robot') and self.running and not self.stopping and self.spawned:
            self.ros.stop_robot()
            self.plan_ready = self.busy = self.task_active = False
            self.task_used = True
            self.task_text = '节点已退出，请移除并重新生成机器人'
        if key == 'scene' and not self.stopping:
            self.close_scene()
        if not self.sessions and self.stopping:
            self.running = self.ready = self.stopping = self.spawned = self.spawning = False
            self.task_active = self.task_used = self.plan_ready = self.busy = False
            self.last_status.clear()
            self.actual_pose = None
            self.task_text = '待命'
            self.pose_label.setText('机器人位置：—')
            self.clock_label.setText('仿真时间：—')
            self.message('会话已关闭，可以选择其他场景。')
        self.update_buttons()

    def update_buttons(self):
        self.scene_combo.setEnabled(not self.running)
        self.factor.setEnabled(not self.running)
        robot_nodes = 'robot' in self.sessions or 'planner' in self.sessions
        self.start_point.setEnabled(not self.spawning and not self.task_active and not self.busy and (not robot_nodes or self.scene() == 'three_floor'))
        self.goal_point.setEnabled(not self.task_active and not self.busy)
        self.start_button.setEnabled(not self.running and not self.sessions)
        self.close_button.setEnabled(self.running and not self.stopping)
        self.spawn_button.setEnabled(self.ready and not self.stopping and not self.spawned and not self.spawning and not robot_nodes)
        self.remove_button.setEnabled(self.spawned and not self.task_active and not self.busy and not self.removing and not self.stopping)
        planner_ready = hasattr(self, 'ros') and self.ros.clients['plan'].service_is_ready()
        if planner_ready and getattr(self, 'task_text', '') == '地图初始化中':
            self.task_text = '地图就绪'
        self.execute_button.setEnabled(self.spawned and self.supported() and planner_ready and not self.task_active and not self.busy and self.factor_supported() and not self.stopping)
        for combo in self.algorithms.values():
            combo.setEnabled(not self.task_active and not self.busy)
        self.stop_button.setEnabled(self.spawned and not self.stopping)
        self.follow_button.setEnabled(self.spawned and not self.stopping)
        self.apply_view.setEnabled(self.ready and not self.stopping)
        scene = '关闭中' if self.stopping else ('运行中' if self.ready else ('启动中' if self.running else '未启动'))
        robot = '移除中' if self.removing else ('已生成' if self.spawned else ('生成中' if self.spawning else '未生成'))
        elapsed = f' · {time.monotonic()-self.task_started_wall:.1f} s' if (self.busy or self.task_active) and hasattr(self, 'task_started_wall') else ''
        self.state_label.setText(f'仿真：{scene}  ｜  机器人：{robot}  ｜  任务：{getattr(self, "task_text", "待命")}{elapsed}')

    def closeEvent(self, event):
        if self.sessions:
            self.closing = True
            self.close_scene()
            event.ignore()
        else:
            self.timer.stop()
            self.ros.shutdown()
            event.accept()


def main(args=None):
    parser = argparse.ArgumentParser(description='Local simulation control panel')
    parser.add_argument('--domain-id', type=int, default=91)
    parser.add_argument('--master-uri', default='http://127.0.0.1:11355')
    parser.add_argument('--headless', action='store_true', help='Do not open Gazebo GUI (automation only)')
    options, ros_args = parser.parse_known_args(args)
    if not 0 <= options.domain_id <= 232:
        parser.error('domain-id must be between 0 and 232')
    address = urlparse(options.master_uri)
    if address.hostname not in ('127.0.0.1', 'localhost') or not address.port:
        parser.error('master-uri must name a local TCP port')
    os.environ['ROS_DOMAIN_ID'] = str(options.domain_id)
    os.environ['GAZEBO_MASTER_URI'] = options.master_uri
    rclpy.init(args=ros_args)
    application = QApplication([sys.argv[0]])
    window = ControlPanel(options.master_uri, options.headless)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    try:
        code = application.exec_()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return code
