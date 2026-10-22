"""Regression for cancelled Qt task callbacks and stop/feedback ordering."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from types import SimpleNamespace
import pytest


def test_cancelled_callbacks_and_stop_during_planning(monkeypatch, tmp_path):
    Qt = pytest.importorskip('PyQt5.QtCore').Qt
    QApplication = pytest.importorskip('PyQt5.QtWidgets').QApplication
    QTest = pytest.importorskip('PyQt5.QtTest').QTest
    import rclpy
    from nav_msgs.msg import Odometry
    from simulation_hmi.app import ControlPanel
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    panel = ControlPanel('http://127.0.0.1:11991', headless=True)
    panel.timer.stop()
    panel.ros.timer.stop()
    panel.running = panel.ready = panel.spawned = True
    panel.session_dir = tmp_path
    panel.sessions['planner'] = SimpleNamespace(stop=lambda: None)
    monkeypatch.setattr(panel, 'write_task', lambda: None)
    monkeypatch.setattr(panel, 'write_view', lambda *args, **kwargs: None)
    panel.ros.clients = {key: SimpleNamespace(service_is_ready=lambda: True)
                        for key in ('plan', 'stop', 'execute')}
    algorithms, commands = [], []
    monkeypatch.setattr(panel.ros, 'select_algorithms', lambda selection, callback: algorithms.append(callback))
    monkeypatch.setattr(panel.ros, 'command', lambda name, callback, **kwargs: commands.append((name, callback)))
    monkeypatch.setattr(panel.ros, 'stop_robot', lambda: None)
    ok = SimpleNamespace(success=True, message='accepted')
    selected = SimpleNamespace(result=SimpleNamespace(successful=True, reason=''))
    # Stop while the parameter reply is pending; ACK then actual stopped feedback.
    panel.update_buttons()
    QTest.mouseClick(panel.execute_button, Qt.LeftButton)
    old_selection = algorithms[-1]
    QTest.mouseClick(panel.stop_button, Qt.LeftButton)
    commands[-1][1](ok, '')
    assert panel.busy
    panel.pose_received(Odometry())
    assert not panel.busy and not panel.task_active
    QTest.mouseClick(panel.execute_button, Qt.LeftButton)
    old_selection(selected, '')
    assert [name for name, _ in commands] == ['stop']
    algorithms[-1](selected, '')
    assert commands[-1][0] == 'plan'
    old_plan = commands[-1][1]
    # Stop during global planning; actual feedback arrives before the stop ACK.
    panel.actual_pose = None
    QTest.mouseClick(panel.stop_button, Qt.LeftButton)
    panel.pose_received(Odometry())
    assert panel.busy
    commands[-1][1](ok, '')
    assert not panel.busy and not panel.task_active
    old_plan(ok, '')
    assert 'execute' not in [name for name, _ in commands]
    panel.sessions.clear()
    panel.running = panel.spawned = False
    panel.ros.shutdown()
    panel.deleteLater()
    app.processEvents()
    rclpy.shutdown()
