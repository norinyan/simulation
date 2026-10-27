#!/usr/bin/env python3
"""Display the existing HMI in a Qt window; the backend remains a separate process."""
import argparse
import os
import signal
import sys
import time
from urllib.error import URLError
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8000')
    args = parser.parse_args()
    deadline = time.monotonic() + 30
    while True:
        try:
            with urlopen(args.url + '/api/config', timeout=.5) as response:
                if response.status == 200:
                    break
        except (URLError, TimeoutError):
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError('HMI 后端未就绪，未打开窗口')
        time.sleep(.2)

    # Chromium does not support its sandbox as root. Keep this compatibility
    # setting local to the desktop process in the existing development container.
    if os.geteuid() == 0:
        os.environ.setdefault('QTWEBENGINE_DISABLE_SANDBOX', '1')

    from PySide6.QtCore import QTimer, QUrl
    from PySide6.QtWidgets import QApplication, QMainWindow
    from PySide6.QtWebEngineCore import QWebEngineSettings
    from PySide6.QtWebEngineWidgets import QWebEngineView

    app = QApplication(sys.argv[:1])
    window = QMainWindow()
    window.setWindowTitle('机器人仿真平台')
    window.resize(1600, 1050)
    view = QWebEngineView(window)
    view.settings().setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)
    window.setCentralWidget(view)
    view.setUrl(QUrl(args.url))
    window.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
