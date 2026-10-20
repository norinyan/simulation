"""Verify the lifecycle boundary that prevents orphaned Gazebo processes."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5.QtWidgets import QApplication
from PyQt5.QtTest import QTest
from simulation_hmi.session import SessionProcess


def test_stop_owns_children_but_leaves_unrelated_process(tmp_path):
    application = QApplication.instance() or QApplication([])
    external = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True)
    pid_file = tmp_path/'child.pid'
    code = ("import subprocess,sys,time; "
            "p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)']); "
            f"open({str(pid_file)!r},'w').write(str(p.pid)); time.sleep(30)")
    owned = SessionProcess('lifecycle', tmp_path, {})
    try:
        owned.start([sys.executable, '-c', code])
        deadline = time.monotonic()+4
        while not pid_file.exists() and time.monotonic()<deadline:
            application.processEvents()
            QTest.qWait(20)
        assert pid_file.exists()
        child = int(pid_file.read_text())
        owned.stop()
        deadline = time.monotonic()+8
        while not owned.done and time.monotonic()<deadline:
            application.processEvents()
            QTest.qWait(20)
        assert owned.done
        stat = Path(f'/proc/{child}/stat')
        assert not stat.exists() or stat.read_text().rsplit(')',1)[1].split()[0]=='Z'
        assert external.poll() is None
    finally:
        owned.signal(signal.SIGKILL)
        external.terminate()
        external.wait(timeout=3)
