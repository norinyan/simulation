"""Coordinate one accepted planning request; route and mode decisions stay in planning."""
import re
from PyQt5.QtCore import QObject, pyqtSignal


def task_identifier(text):
    match = re.search(r'(?:^|[;\s])task_id=([^;\s]+)', text)
    return match.group(1) if match else None


class TaskCoordinator(QObject):
    phase = pyqtSignal(str)
    message = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        self.bridge = bridge
        self.generation = 0
        self.task_id = None
        self.state = 'idle'
        self.early_status = {}
        bridge.status.connect(self.observe)

    def callback(self, handler):
        generation = self.generation
        return lambda response, error: handler(response, error) if generation == self.generation else None

    def start(self, start, goal, algorithms):
        self.cancel()
        self.state = 'configuring'
        self.phase.emit('提交任务')
        self.bridge.set_task(start, goal, algorithms, self.callback(self.configured))

    def cancel(self):
        self.generation += 1
        self.state = 'idle'
        self.task_id = None
        self.early_status.clear()

    def fail(self, reason):
        self.state = 'idle'
        self.failed.emit(reason)

    def configured(self, response, error):
        if not response or not response.success:
            self.fail(error or response.message)
            return
        self.state = 'requesting'
        self.phase.emit('请求全局规划')
        self.bridge.command('plan', self.callback(self.accepted))

    def accepted(self, response, error):
        if not response or not response.success:
            self.fail(error or response.message)
            return
        self.task_id = task_identifier(response.message)
        if not self.task_id:
            self.bridge.stop_robot()
            self.fail('规划响应缺少任务标识，已请求停止')
            return
        self.state = 'planning'
        self.phase.emit('全局规划中')
        self.message.emit(response.message)
        text = self.early_status.pop(self.task_id, None)
        self.early_status.clear()
        if text:
            self.observe('/planning/task_status', text)

    def observe(self, topic, text):
        if topic != '/planning/task_status':
            return
        identifier = task_identifier(text)
        if not identifier:
            return
        if self.state == 'requesting':
            self.early_status[identifier] = text
            return
        if identifier != self.task_id:
            return
        state = text.split(';', 1)[0]
        if state == 'PLAN_READY' and self.state == 'planning':
            self.state = 'executing_request'
            self.phase.emit('请求执行')
            self.bridge.command('execute', self.callback(self.executed))
        elif state in ('DONE', 'FAILED', 'STOPPED'):
            self.state = 'idle'
            if state != 'DONE':
                self.failed.emit(text)

    def executed(self, response, error):
        if self.state != 'executing_request':
            return  # A terminal status may arrive before its service response.
        if not response or not response.success:
            self.fail(error or response.message)
            return
        self.state = 'executing'
        self.phase.emit('执行请求已接受')
        self.message.emit(response.message)
