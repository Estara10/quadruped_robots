#!/usr/bin/python3 -s
"""Read-only Go2 charts and front-camera preview; no ROS or motion publisher."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "go2_live_view_mpl"))

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib import font_manager, rcParams
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

LEGS = ("FR", "FL", "RR", "RL")
JOINTS = ("hip", "thigh", "calf")
NAMES = [f"{leg}_{joint}" for leg in LEGS for joint in JOINTS]


def number(value, digits=2):
    return f"{value:.{digits}f}" if value is not None and math.isfinite(value) else "--"


class LiveView(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("Go2 · 只读状态曲线与前置相机")
        self.samples = deque(maxlen=300)
        self.camera_frame = None
        self.camera_code = None
        self.errors = {}
        self.lock = threading.Lock()
        self.processes = {}
        self.threads = []
        self.log_files = []
        self.closed = False
        self.last_camera_time = None
        self.log_dir = Path(tempfile.mkdtemp(prefix="go2_live_view_"))

        fonts = {font.name for font in font_manager.fontManager.ttflist}
        for font in ("Noto Sans CJK SC", "WenQuanYi Micro Hei", "Noto Sans CJK JP"):
            if font in fonts:
                rcParams["font.family"] = font
                break
        rcParams["axes.unicode_minus"] = False
        rcParams["font.size"] = 8
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        title = QtWidgets.QLabel("只读监视 · 最近30秒 · 状态采样10 Hz / 绘图5 Hz · 不发送运动命令")
        title.setStyleSheet("font-size: 16px; font-weight: bold; padding: 5px")
        layout.addWidget(title)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("关节曲线选择腿："))
        self.leg = QtWidgets.QComboBox()
        self.leg.addItems(["FR 右前", "FL 左前", "RR 右后", "RL 左后"])
        controls.addWidget(self.leg)
        controls.addStretch()
        end = QtWidgets.QPushButton("结束只读监视")
        end.clicked.connect(self.close)
        controls.addWidget(end)
        layout.addLayout(controls)
        self.state_status = QtWidgets.QLabel("等待状态消息…")
        self.state_status.setWordWrap(True)
        layout.addWidget(self.state_status)

        body = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(body, 1)
        self.figure = Figure(figsize=(10, 8))
        self.canvas = FigureCanvasQTAgg(self.figure)
        body.addWidget(self.canvas)
        self.axes = self.figure.subplots(3, 3).flatten()
        self.specs = [
            ("low", "q", "关节角 / rad", JOINTS),
            ("low", "dq", "关节角速度 / rad/s", JOINTS),
            ("low", "tau", "估计关节力矩 / Nm", JOINTS),
            ("low", "temperature", "电机温度 / °C", JOINTS),
            ("low", "gyro", "IMU角速度 / SDK原值", ("x", "y", "z")),
            ("low", "accel", "IMU加速度 / m/s²", ("x", "y", "z")),
            ("sport", "position", "位置 / SDK原值（z不是机身高度）", ("x", "y", "z")),
            ("sport", "velocity", "线速度 / SDK原值", ("x", "y", "z")),
            ("low", "foot_force", "足端力 / SDK原值", LEGS),
        ]
        self.lines = []
        for ax, (_, _, title_text, labels) in zip(self.axes, self.specs):
            self.lines.append([ax.plot([], [], label=label, linewidth=1.1)[0] for label in labels])
            ax.set_title(title_text, fontsize=9)
            ax.set_xlabel("时间 / s")
            ax.grid(alpha=0.25)
            ax.legend(loc="upper right", fontsize=7)
        self.figure.subplots_adjust(left=0.065, right=0.98, top=0.95, bottom=0.07, wspace=0.3, hspace=0.55)

        side = QtWidgets.QWidget()
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.addWidget(QtWidgets.QLabel("12关节最新样本（单位按接口约定，尚未标定）"))
        self.table = QtWidgets.QTableWidget(12, 4)
        self.table.setHorizontalHeaderLabels(["q / rad", "dq / rad/s", "估计τ / Nm", "温度 / °C"])
        self.table.setVerticalHeaderLabels(NAMES)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(23)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setMaximumHeight(330)
        self.table.setMinimumHeight(320)
        for row in range(12):
            for col in range(4):
                self.table.setItem(row, col, QtWidgets.QTableWidgetItem("--"))
        side_layout.addWidget(self.table)
        self.camera_label = QtWidgets.QLabel("正在请求Go2前置相机…")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 230)
        self.camera_label.setStyleSheet("background: #111827; color: white")
        side_layout.addWidget(self.camera_label, 1)
        self.camera_status = QtWidgets.QLabel("彩色图像预览；不是深度或避障距离。")
        self.camera_status.setWordWrap(True)
        side_layout.addWidget(self.camera_status)
        body.addWidget(side)
        body.setSizes([1020, 480])
        geometry = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1580, int(geometry.width() * 0.96)), min(950, int(geometry.height() * 0.94)))

        env = os.environ.copy()
        env["LD_LIBRARY_PATH"] = "/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib"
        try:
            for name, executable in (("state", "go2_readonly_state"), ("camera", "go2_readonly_camera")):
                log = open(self.log_dir / f"{name}.stderr.txt", "wb")
                self.log_files.append(log)
                command = [str(args.build_dir / executable), "--domain", str(args.domain), "--interface", args.interface]
                if name == "state":
                    command.append("--json")
                self.processes[name] = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=log, env=env)
                target = self.read_state if name == "state" else self.read_camera
                thread = threading.Thread(target=target, daemon=True)
                self.threads.append(thread)
                thread.start()
        except Exception:
            self.shutdown()
            raise
        print(f"只读窗口日志：{self.log_dir}", flush=True)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

    def read_state(self):
        try:
            for line in self.processes["state"].stdout:
                packet = json.loads(line)
                with self.lock:
                    self.samples.append((time.monotonic(), packet))
        except (ValueError, OSError) as error:
            with self.lock:
                self.errors["state"] = str(error)

    def read_camera(self):
        try:
            stream = self.processes["camera"].stdout
            while header := stream.readline():
                metadata = json.loads(header)
                size = metadata["bytes"]
                if size < 0:
                    raise ValueError("相机数据长度为负")
                data = stream.read(size)
                if len(data) != size:
                    raise ValueError("相机图像读取不完整")
                with self.lock:
                    self.camera_code = metadata["code"]
                    if metadata["code"] == 0 and data:
                        self.camera_frame = (time.monotonic(), data)
        except (ValueError, OSError) as error:
            with self.lock:
                self.errors["camera"] = str(error)

    def refresh(self):
        now = time.monotonic()
        with self.lock:
            samples = list(self.samples)
            camera = self.camera_frame
            code = self.camera_code
            errors = dict(self.errors)
        for name, process in self.processes.items():
            if process.poll() is not None:
                errors[name] = f"读取进程退出，代码{process.returncode}；见 {self.log_dir / (name + '.stderr.txt')}"
        if samples:
            last_time, latest = samples[-1]
            parts = []
            for key, label in (("low", "LowState"), ("sport", "SportModeState")):
                state = latest[key]
                if state is None:
                    parts.append(f"{label} 尚未收到")
                else:
                    age = state["age"] + now - last_time
                    parts.append(f"{label} {number(state['hz'], 1)} Hz / DDS接收龄 {state['age'] * 1000:.0f} ms / 显示样本龄 {age * 1000:.0f} ms" + (" 状态过期" if state["age"] > 0.1 else ""))
            if latest["sport"]:
                parts.append(f"原始mode={latest['sport']['mode']} / error={latest['sport']['error_code']}")
            self.state_status.setText("   |   ".join(parts) + (f"\n{errors['state']}" if "state" in errors else ""))
            origin = samples[0][0]
            times = np.array([received - origin for received, _ in samples])
            leg = self.leg.currentIndex()
            for index, (ax, spec, lines) in enumerate(zip(self.axes, self.specs, self.lines)):
                source, field, title_text, labels = spec
                values = []
                for _, packet in samples:
                    state = packet[source]
                    if state is None or state["age"] > 0.1:
                        values.append([np.nan] * len(labels))
                    else:
                        vector = state[field]
                        values.append(vector[leg * 3:leg * 3 + 3] if index < 4 else vector)
                matrix = np.asarray(values, dtype=float)
                for col, line in enumerate(lines):
                    line.set_data(times, matrix[:, col])
                ax.set_title((f"{LEGS[leg]} " if index < 4 else "") + title_text, fontsize=9)
                ax.set_xlim(0, max(1, times[-1]))
                ax.relim()
                ax.autoscale_view(scalex=False)
                if field == "velocity":
                    bottom, top = ax.get_ylim()
                    ax.set_ylim(min(-0.05, bottom), max(0.05, top))
            low = latest["low"]
            if low:
                for col, field in enumerate(("q", "dq", "tau", "temperature")):
                    for row, value in enumerate(low[field]):
                        self.table.item(row, col).setText(number(value, 0 if col == 3 else 3))
            self.canvas.draw_idle()
        elif "state" in errors:
            self.state_status.setText(errors["state"])
        if camera:
            received, jpeg = camera
            if received != self.last_camera_time:
                image = QtGui.QImage.fromData(jpeg)
                if image.isNull():
                    errors["camera"] = "相机返回的数据无法解码为图像"
                else:
                    pixmap = QtGui.QPixmap.fromImage(image)
                    self.camera_label.setPixmap(pixmap.scaled(self.camera_label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
                self.last_camera_time = received
            self.camera_status.setText(f"前置彩色图像 · 距最近取图成功 {now - received:.2f} s\n不是深度/距离输入。" + (f"\nSDK返回码：{code}" if code else ""))
        elif code is not None:
            self.camera_status.setText(f"尚未获得图像；SDK返回码 {code}。不会自动开启或切换机器人服务。")
        if "camera" in errors:
            self.camera_status.setText(errors["camera"])

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        for process in self.processes.values():
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for name, process in self.processes.items():
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                print(f"{name}退出超时，强制结束本窗口的读取进程", file=sys.stderr)
                process.kill()
                process.wait()
            print(f"{name} reader exit={process.returncode}", flush=True)
        for thread in self.threads:
            thread.join(timeout=1)
        for process in self.processes.values():
            process.stdout.close()
        for log in self.log_files:
            log.close()

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--build-dir", type=Path, default=Path("/tmp/go2_readonly_state_build"))
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = LiveView(args)
    app.aboutToQuit.connect(window.shutdown)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.show()
    sys.exit(app.exec_())
