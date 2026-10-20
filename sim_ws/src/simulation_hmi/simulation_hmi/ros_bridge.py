"""ROS callbacks are pumped by Qt, so widgets are never touched from ROS threads."""
import time
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.parameter import Parameter
from gazebo_msgs.srv import GetModelList, DeleteEntity
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RosBridge(QObject):
    status = pyqtSignal(str, str)
    pose = pyqtSignal(object)
    clock = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.node = Node('simulation_hmi', parameter_overrides=[Parameter('use_sim_time', value=True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = []
        for topic in ('/robot/execution_status', '/planning/status', '/planning/local_status',
                      '/planning/foot_status', '/planning/task_status'):
            self.subscriptions.append(self.node.create_subscription(
                String, topic, lambda msg, topic=topic: self.status.emit(topic, msg.data), qos))
        self.subscriptions.append(self.node.create_subscription(Odometry, '/robot/odom', self.odom, 10))
        self.subscriptions.append(self.node.create_subscription(Clock, '/clock', self.time, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.mode = self.node.create_publisher(String, '/robot/mode_command', 10)
        self.world = self.node.create_client(GetModelList, '/get_model_list')
        self.delete = self.node.create_client(DeleteEntity, '/delete_entity')
        self.clients = {name: self.node.create_client(Trigger, '/planning/'+name)
                        for name in ('plan', 'execute', 'stop')}
        self.pending = []
        self.last_clock = self.last_odom = 0
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.spin)
        self.timer.start()

    def spin(self):
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        for item in list(self.pending):
            future, end, callback = item
            if future.done():
                self.pending.remove(item)
                try:
                    response = future.result()
                except Exception as error:
                    callback(None, str(error))
                else:
                    callback(response, '')
            elif now > end:
                self.pending.remove(item)
                future.cancel()
                callback(None, '服务响应超时')

    def request(self, client, request, callback, timeout=12):
        if not client.service_is_ready():
            callback(None, '服务尚未就绪')
            return
        self.pending.append((client.call_async(request), time.monotonic()+timeout, callback))

    def command(self, name, callback):
        self.request(self.clients[name], Trigger.Request(), callback, 30 if name == 'plan' else 8)

    def stop_robot(self):
        self.mode.publish(String(data='STOP'))

    def odom(self, msg):
        self.last_odom = time.monotonic()
        self.pose.emit(msg)

    def time(self, msg):
        self.last_clock = time.monotonic()
        self.clock.emit(msg.clock.sec+msg.clock.nanosec/1e9)

    def shutdown(self):
        self.timer.stop()
        for future, _, _ in self.pending:
            future.cancel()
        self.pending.clear()
        self.node.destroy_node()
