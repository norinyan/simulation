"""Own exactly the process groups created by this panel; never kill by executable name."""
import os
from pathlib import Path
import signal
import time

from PyQt5.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, pyqtSignal


class SessionProcess(QObject):
    message = pyqtSignal(str)
    ended = pyqtSignal(int)

    def __init__(self, name, directory, environment, parent=None):
        super().__init__(parent)
        self.name = name
        self.group = 0
        self.deadline = None
        self.phase = 0
        self.done = False
        self.pending = ''
        self.log = (Path(directory)/f'{name}.log').open('a', buffering=1)
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        env = QProcessEnvironment.systemEnvironment()
        for key, value in environment.items():
            env.insert(key, str(value))
        self.process.setProcessEnvironment(env)
        self.process.readyReadStandardOutput.connect(self.read)
        self.process.started.connect(lambda: setattr(self, 'group', int(self.process.processId())))
        self.process.finished.connect(self.exited)
        self.process.errorOccurred.connect(self.error)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.reap)
        self.timer.start()

    def start(self, command):
        # setsid gives this session a private process group, including launch children.
        self.process.start('setsid', ['--wait', *command])

    def read(self):
        text = bytes(self.process.readAllStandardOutput()).decode(errors='replace')
        self.log.write(text)
        self.pending += text
        lines = self.pending.split('\n')
        self.pending = lines.pop()
        for line in lines:
            if any(term in line for term in ('[ERROR]', 'FAILED', 'Successfully spawned', 'Caught exception')):
                self.message.emit(f'{self.name}: {line[-800:]}')

    def error(self, _):
        self.message.emit(f'{self.name}: {self.process.errorString()}')
        if self.process.state() == QProcess.NotRunning and not self.group:
            self.exited(-1)

    def signal(self, number):
        if not self.group or self.group == os.getpgrp():
            return
        try:
            os.killpg(self.group, number)
        except ProcessLookupError:
            pass

    def stop(self):
        if self.done or self.deadline is not None:
            return
        self.signal(signal.SIGINT)
        self.phase = 0
        self.deadline = time.monotonic()+3

    def exited(self, code, *_):
        if self.done:
            return
        self.code = code
        self.read()
        # A launch parent exiting does not prove its Gazebo children exited.
        if self.deadline is None:
            self.stop()

    def reap(self):
        if self.deadline is None:
            return
        alive = False
        if self.group:
            # Ignore zombies: they have no resources and are reaped by the container init.
            for entry in Path('/proc').iterdir():
                if not entry.name.isdigit():
                    continue
                try:
                    stat = (entry/'stat').read_text().rsplit(')', 1)[1].split()
                    if int(stat[2]) == self.group and stat[0] != 'Z':
                        alive = True
                        break
                except (OSError, ValueError, IndexError):
                    continue
        if not alive and self.process.state() == QProcess.NotRunning:
            self.done = True
            self.timer.stop()
            self.log.close()
            self.ended.emit(getattr(self, 'code', 0))
            return
        if time.monotonic() >= self.deadline:
            self.signal(signal.SIGTERM if self.phase == 0 else signal.SIGKILL)
            self.phase += 1
            self.deadline = time.monotonic()+2
