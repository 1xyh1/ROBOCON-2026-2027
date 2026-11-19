"""Native Qt desktop SLAM test console; the ROS backend runs in another process."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import sys
import time
from zoneinfo import ZoneInfo

from PyQt5 import QtCore, QtGui, QtWidgets
import yaml
from robocon_core.slam_bench import load_bench_view


LABELS = {"PASS": "通过", "FAIL": "未通过", "RUNNING": "采样中", "INTERRUPTED": "已中断",
          "NEEDS_EVIDENCE": "待证据", "INCOMPLETE": "未完成"}
COLORS = {"PASS": "#15836a", "FAIL": "#c34c42", "RUNNING": "#2675ba", "INTERRUPTED": "#a26b25",
          "NEEDS_EVIDENCE": "#a26b25"}


def number(value, scale=1., suffix="", digits=2):
    return "—" if value is None else f"{value * scale:.{digits}f}{suffix}"


class SpatialView(QtWidgets.QWidget):
    def __init__(self, cloud=False):
        super().__init__()
        self.cloud_view = cloud
        self.points, self.path = [], []
        self.yaw, self.pitch, self.zoom = -.6, .65, 1.
        self.last_mouse = None
        self.setMinimumSize(300, 240)

    def wheelEvent(self, event):
        self.zoom = max(.1, min(12., self.zoom * (1.15 if event.angleDelta().y() > 0 else 1 / 1.15)))
        self.update()

    def mousePressEvent(self, event):
        self.last_mouse = event.pos()

    def mouseMoveEvent(self, event):
        if self.cloud_view and self.last_mouse is not None and event.buttons():
            delta = event.pos() - self.last_mouse
            self.yaw += delta.x() * .008
            self.pitch = max(.05, min(1.5, self.pitch + delta.y() * .008))
            self.last_mouse = event.pos()
            self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#111f30"))
        painter.setPen(QtGui.QColor("#8dabc8"))
        title = "点云累积预览 · livo_odom · 拖动旋转 / 滚轮缩放" if self.cloud_view else "融合轨迹 XY · 单位 m · 非独立真值"
        painter.drawText(16, 25, title)
        points = self.points if self.cloud_view else self.path
        if not points:
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, "等待真实数据，不显示模拟轨迹")
            return
        if self.cloud_view:
            cy, sy = math.cos(self.yaw), math.sin(self.yaw)
            projected = [(p[0] * cy - p[1] * sy,
                          (p[0] * sy + p[1] * cy) * math.sin(self.pitch) - p[2] * math.cos(self.pitch), p[2])
                         for p in points]
        else:
            projected = [(p[0], -p[1], 0.) for p in points]
        xs, ys = [p[0] for p in projected], [p[1] for p in projected]
        center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
        span = max(max(xs) - min(xs), max(ys) - min(ys), .05)
        scale = min(self.width() - 70, self.height() - 80) / span * .85 * self.zoom

        def mapped(p):
            return QtCore.QPointF(self.width() / 2 + (p[0] - center[0]) * scale,
                                 (self.height() + 25) / 2 + (p[1] - center[1]) * scale)

        painter.setPen(QtGui.QPen(QtGui.QColor("#24364c"), 1))
        for fraction in (.2, .4, .6, .8):
            painter.drawLine(int(self.width() * fraction), 40, int(self.width() * fraction), self.height() - 25)
            painter.drawLine(15, int(self.height() * fraction), self.width() - 15, int(self.height() * fraction))
        if self.cloud_view:
            zmin, zmax = min(p[2] for p in projected), max(p[2] for p in projected)
            palette = ["#326ca8", "#258da7", "#35b3a3", "#a6ce78", "#ead36c", "#e78b5d"]
            groups = [[] for _ in palette]
            for point in projected:
                index = min(5, int((point[2] - zmin) / max(.001, zmax - zmin) * 5.99))
                groups[index].append(mapped(point))
            for color, group in zip(palette, groups):
                painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
                painter.drawPoints(QtGui.QPolygonF(group))
        else:
            painter.setPen(QtGui.QPen(QtGui.QColor("#58c9d2"), 2))
            painter.drawPolyline(QtGui.QPolygonF([mapped(point) for point in projected]))
            for point, color in ((projected[0], "#75d898"), (projected[-1], "#ffd56e")):
                painter.setBrush(QtGui.QColor(color))
                painter.drawEllipse(mapped(point), 4, 4)
        painter.setPen(QtGui.QColor("#9ab3cb"))
        painter.drawText(16, self.height() - 10, f"预览 {len(points)} 点 / 视野约 {span / self.zoom:.3f} m")


class BenchWindow(QtWidgets.QMainWindow):
    def __init__(self, repo, standard_path, session=None):
        super().__init__()
        self.repo, self.standard_path = repo, standard_path
        self.standard = yaml.safe_load(standard_path.read_text())
        self.session, self.state, self.image_mtime = session, {}, 0.
        self.document_path = None
        self.reviewing = session is not None
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_log)
        self.process.finished.connect(self.finished)
        self.setWindowTitle("ROBOCON · SLAM 测试台")
        self.resize(1480, 930)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        title = QtWidgets.QLabel("ROBOCON  /  SLAM 测试台    MID-360 · D435i")
        title.setObjectName("title")
        layout.addWidget(title)
        self.banner = QtWidgets.QLabel("未启动 · 仅测试视觉与定位；不会发送任何底盘或机械臂运动指令")
        self.banner.setObjectName("banner")
        layout.addWidget(self.banner)
        middle = QtWidgets.QHBoxLayout()
        layout.addLayout(middle, 1)
        controls = QtWidgets.QFrame()
        controls.setFixedWidth(265)
        form = QtWidgets.QVBoxLayout(controls)
        form.addWidget(QtWidgets.QLabel("测试编号 / " + self.standard["id"]))
        self.case = QtWidgets.QComboBox()
        for key, value in self.standard["cases"].items():
            self.case.addItem(key + "  " + value["name"], key)
        self.case.setCurrentIndex(1)
        self.case.currentIndexChanged.connect(self.select_case)
        form.addWidget(self.case)
        self.protocol = QtWidgets.QLabel()
        self.protocol.setWordWrap(True)
        form.addWidget(self.protocol)
        self.stationary = QtWidgets.QCheckBox("SB01：设备已固定，测试期间不触碰")
        form.addWidget(self.stationary)
        self.source = QtWidgets.QComboBox()
        self.source.addItem("设备实时测试（域 178）", "live")
        self.source.addItem("完整录包回放（域 179）", "replay")
        form.addWidget(self.source)
        form.addWidget(QtWidgets.QLabel("采样时长（秒，预热另计）"))
        self.duration = QtWidgets.QSpinBox()
        self.duration.setRange(5, 1800)
        form.addWidget(self.duration)
        form.addWidget(QtWidgets.QLabel("配置目录"))
        self.config = QtWidgets.QLineEdit(str(repo / "artifacts/fusion-20261017-replay-config-v3"))
        form.addWidget(self.config)
        choose = QtWidgets.QPushButton("选择配置目录")
        choose.clicked.connect(lambda: self.choose_directory(self.config))
        form.addWidget(choose)
        form.addWidget(QtWidgets.QLabel("回放数据目录（含 bag/）"))
        self.capture = QtWidgets.QLineEdit(str(repo / "artifacts/fusion-20261017-bag-v2"))
        form.addWidget(self.capture)
        choose_bag = QtWidgets.QPushButton("选择录包目录")
        choose_bag.clicked.connect(lambda: self.choose_directory(self.capture))
        form.addWidget(choose_bag)
        self.reference = QtWidgets.QCheckBox("同时运行 FAST-LIO 对照\n（增加负载；坐标系未对齐）")
        form.addWidget(self.reference)
        self.start_button = QtWidgets.QPushButton("开始测试")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start)
        form.addWidget(self.start_button)
        stop_button = QtWidgets.QPushButton("中断并保存报告")
        stop_button.clicked.connect(self.stop)
        form.addWidget(stop_button)
        fixture = QtWidgets.QPushButton("SB02：确认已回到同一夹具")
        fixture.clicked.connect(self.confirm_fixture)
        form.addWidget(fixture)
        open_button = QtWidgets.QPushButton("打开已有测试报告")
        open_button.clicked.connect(self.open_report)
        form.addWidget(open_button)
        export = QtWidgets.QPushButton("导出当前测试证据 ZIP")
        export.clicked.connect(self.export)
        form.addWidget(export)
        form.addStretch()
        warning = QtWidgets.QLabel("精度未认证\n静态抖动 ≠ 定位误差\n初值外参 ≠ 已完成标定\n重复回放 ≠ 独立实场验收")
        warning.setObjectName("warning")
        warning.setWordWrap(True)
        form.addWidget(warning)
        middle.addWidget(controls)
        content = QtWidgets.QVBoxLayout()
        middle.addLayout(content, 1)
        cards = QtWidgets.QHBoxLayout()
        self.cards = []
        for name in ("里程计 Hz", "消息龄 P99", "最大间断", "水平 P95", "航向 P95", "测试时长"):
            label = QtWidgets.QLabel(name + "\n—")
            label.setObjectName("card")
            label.setAlignment(QtCore.Qt.AlignCenter)
            cards.addWidget(label)
            self.cards.append(label)
        content.addLayout(cards)
        grid = QtWidgets.QGridLayout()
        self.trajectory = SpatialView()
        self.cloud = SpatialView(cloud=True)
        grid.addWidget(self.trajectory, 0, 0)
        grid.addWidget(self.cloud, 0, 1)
        self.image = QtWidgets.QLabel("视觉跟踪画面\n等待融合节点真实图像")
        self.image.setAlignment(QtCore.Qt.AlignCenter)
        self.image.setStyleSheet("background:#111f30;color:#9ab3cb;")
        self.image.setMinimumSize(300, 220)
        grid.addWidget(self.image, 1, 0)
        self.table = QtWidgets.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["检查项", "实测", "门限", "结果"])
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        grid.addWidget(self.table, 1, 1)
        content.addLayout(grid, 1)
        self.notes = QtWidgets.QLabel("点云为 10 cm 体素降采样的显示预览；不是测量地图。所有测试通过也不自动授予运动权限。")
        self.notes.setWordWrap(True)
        content.addWidget(self.notes)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(95)
        self.log.setMaximumBlockCount(150)
        layout.addWidget(self.log)
        self.setStyleSheet("""
          QMainWindow,QWidget {background:#edf2f6;color:#22334a;font-size:13px;}
          QLabel#title {background:#13283e;color:#ecf5ff;font-size:22px;font-weight:600;padding:15px;}
          QLabel#banner {background:#fff0cc;color:#7e5919;padding:10px;font-weight:600;}
          QLabel#card {background:white;border:1px solid #d7e1e8;border-radius:7px;padding:12px;font-size:15px;}
          QLabel#warning {background:#fff0cc;color:#7e5919;padding:12px;}
          QPushButton {background:white;border:1px solid #cedae4;border-radius:4px;padding:8px;}
          QPushButton#primary {background:#197c78;color:white;font-weight:600;}
          QLineEdit,QComboBox,QSpinBox,QTableWidget,QPlainTextEdit {background:white;padding:4px;}
        """)
        self.select_case()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)
        self.refresh()

    def select_case(self):
        case = self.standard["cases"][self.case.currentData()]
        self.protocol.setText(case["protocol"])
        self.duration.setValue(case["duration_s"])

    def choose_directory(self, edit):
        value = QtWidgets.QFileDialog.getExistingDirectory(self, "选择目录", edit.text())
        if value:
            edit.setText(value)

    def start(self):
        if self.process.state() != QtCore.QProcess.NotRunning:
            return
        if self.case.currentData() == "SB01" and not self.stationary.isChecked():
            QtWidgets.QMessageBox.warning(self, "工况未确认", "静态测试需先固定整套设备，并确认测试期间不触碰；不确定时请先选 SB00。")
            return
        if not (Path(self.config.text()) / "evidence.json").is_file():
            QtWidgets.QMessageBox.warning(self, "缺少配置", "请选择含 livo.yaml、camera.json、evidence.json 的配置目录。")
            return
        session_id = self.case.currentData() + "_" + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d-%H%M%S-%f")
        self.session = self.repo / "artifacts/slam-bench" / session_id
        self.document_path = None
        self.reviewing = False
        self.state = {}
        self.session.parent.mkdir(parents=True, exist_ok=True)
        command = ["-m", "robocon_fusion.bench_session", "--repo", str(self.repo), "--config-dir", self.config.text(),
                   "--standard", str(self.standard_path), "--output", str(self.session),
                   "--case", self.case.currentData(), "--duration", str(self.duration.value()),
                   "--source", self.source.currentData()]
        if self.source.currentData() == "replay":
            command.extend(["--capture", self.capture.text()])
        if self.reference.isChecked():
            command.append("--reference-lio")
        if self.stationary.isChecked():
            command.append("--stationary-confirmed")
        self.log.clear()
        self.image.clear()
        self.image_mtime = 0.
        self.process.setWorkingDirectory(str(self.repo))
        self.process.start(sys.executable, command)
        self.start_button.setEnabled(False)

    def stop(self):
        if not self.reviewing and self.session and self.session.is_dir() and self.state.get("phase") not in ("finished", "failed"):
            (self.session / "stop.request").touch()

    def confirm_fixture(self):
        if self.session and self.state.get("identity", {}).get("case_id") == "SB02":
            answer = QtWidgets.QMessageBox.question(self, "现场证据确认", "整套传感器是否确实回到起始的同一定位夹具和朝向？")
            if answer == QtWidgets.QMessageBox.Yes:
                (self.session / "return_fixture.request").write_text(datetime.now().isoformat())

    def read_log(self):
        text = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self.log.appendPlainText(text[-16000:])

    def finished(self, code, _status):
        self.start_button.setEnabled(True)
        self.refresh()
        self.log.appendPlainText(f"测试进程退出 {code}；证据：{self.session}")
        if self.session and self.session.is_dir():
            QtCore.QTimer.singleShot(300, lambda: self.grab().save(str(self.session / "desktop.png")))

    def open_report(self):
        if self.process.state() != QtCore.QProcess.NotRunning:
            QtWidgets.QMessageBox.warning(self, "测试仍在运行", "请先中断或等待当前测试结束，再切换报告。")
            return
        value, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择测试报告或 state.json", str(self.repo / "artifacts"), "JSON (*.json)")
        if value:
            selected = Path(value)
            try:
                state = load_bench_view(selected)
            except (OSError, ValueError, KeyError) as exc:
                QtWidgets.QMessageBox.warning(self, "报告读取失败", str(exc))
                return
            self.document_path = selected
            self.session = selected.parent
            self.reviewing = True
            self.state = state
            self.image_mtime = 0.
            self.image.clear()
            self.image.setText("历史报告：没有保存的图像预览")
            case_index = self.case.findData(state.get("identity", {}).get("case_id"))
            if case_index >= 0:
                self.case.setCurrentIndex(case_index)
            self.refresh()

    def export(self):
        if not self.session or not self.session.is_dir():
            return
        if not self.reviewing and self.state.get("phase") not in ("finished", "failed"):
            QtWidgets.QMessageBox.warning(self, "测试仍在运行", "测试结束后再导出完整证据。")
            return
        value, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出证据", str(self.session) + ".zip", "ZIP (*.zip)")
        if value:
            if Path(value).resolve().is_relative_to(self.session.resolve()):
                QtWidgets.QMessageBox.warning(self, "导出目录", "ZIP 请保存到测试目录外，避免打包自身。")
                return
            shutil.make_archive(str(Path(value).with_suffix("")), "zip", self.session)
            self.log.appendPlainText("已导出 " + value)

    def refresh(self):
        if not self.session:
            return
        path = self.document_path or self.session / "state.json"
        if not path.exists() and self.reviewing:
            path = self.session / "report.json"
        if not path.exists():
            return
        try:
            state = load_bench_view(path)
        except (OSError, ValueError, KeyError) as exc:
            self.banner.setText("报告读取失败：" + str(exc))
            return
        self.state = state
        identity, metrics = state.get("identity", {}), state.get("metrics", {})
        verdict = state.get("verdict", {})
        phase = state.get("phase", "")
        active = not self.reviewing and phase not in ("finished", "failed")
        self.start_button.setEnabled(not active and self.process.state() == QtCore.QProcess.NotRunning)
        status = verdict.get("status", "RUNNING")
        label = LABELS.get(status, status)
        calibrated = not identity.get("calibration", {}).get("unverified", ["unknown"])
        banner = f"{identity.get('case_id', '')} · {identity.get('source', '')} · {phase} · {label}"
        if self.reviewing:
            banner = "历史报告（非实时） | " + banner
        if identity.get("algorithm"):
            banner += " | " + identity["algorithm"]
        banner += "  |  精度未认证" + (" · 标定未验收" if not calibrated else "")
        if state.get("error"):
            banner += "  |  " + state["error"]
        review_path = self.session / "review.json"
        if review_path.exists():
            review = json.loads(review_path.read_text(encoding="utf-8"))
            banner += "  |  " + review.get("note", "工况复核：仅作诊断")
        if active and phase != "preflight" and time.time_ns() - state.get("heartbeat_utc_ns", 0) > 3_000_000_000:
            banner += "  |  监控心跳已过期，禁止当作实时数据"
        self.banner.setText(banner)
        streams, static = metrics.get("streams", {}), metrics.get("static", {})
        odom = streams.get("odom", {})
        titles = ["里程计 Hz", "消息龄 P99", "最大间断", "水平 P95", "航向 P95", "测试时长"]
        values = [number(odom.get("hz")), number(odom.get("age_p99_s"), 1000, " ms", 0),
                  number(odom.get("max_receive_gap_s"), 1000, " ms", 0),
                  number(static.get("horizontal_p95_m"), 1000, " mm"),
                  number(static.get("yaw_p95_deg"), 1, "°"), number(metrics.get("elapsed_s"), 1, " s", 0)]
        if state.get("review_scope"):
            motion = metrics["estimated_motion"]
            titles[2:5] = ["测量最大间断", "最大估计半径", "估计末位移"]
            values[3:5] = [number(motion.get("max_radius_m"), suffix=" m"),
                           number(motion.get("end_displacement_m"), suffix=" m")]
        for widget, title, value in zip(self.cards, titles, values):
            widget.setText(title + "\n" + value)
        self.trajectory.path = metrics.get("trajectory", [])
        self.cloud.points = state.get("cloud", [])
        self.trajectory.update()
        self.cloud.update()
        checks = verdict.get("checks", [])
        self.table.setRowCount(len(checks))
        for row, check in enumerate(checks):
            measured = check["value"]
            measured = f"{measured:.4f}" if isinstance(measured, float) else str(measured)
            for column, text in enumerate((check["name"], measured, str(check["limit"]), LABELS.get(check["status"], check["status"]))):
                item = QtWidgets.QTableWidgetItem(text)
                if column == 3:
                    item.setForeground(QtGui.QColor(COLORS.get(check["status"], "#22334a")))
                self.table.setItem(row, column, item)
        image = self.session / "image.jpg"
        if image.exists() and image.stat().st_mtime != self.image_mtime:
            self.image_mtime = image.stat().st_mtime
            pixmap = QtGui.QPixmap(str(image))
            self.image.setPixmap(pixmap.scaled(self.image.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        if self.reviewing:
            self.notes.setText("历史报告预览；保留当时标准和判定。估计轨迹、半径和末位移均不是独立测量误差。")
        elif state.get("image_age_s", 0.) > 1:
            self.notes.setText("视觉画面已过期；数值和轨迹是否继续推进请看数据门控。旧画面不是新数据。")
        else:
            self.notes.setText("点云为 10 cm 体素显示预览，不是测量地图；静态抖动不等于绝对误差。" +
                               ("  硬件告警：" + str(len(state["hardware_errors"])) if state.get("hardware_errors") else ""))

    def closeEvent(self, event):
        if self.process.state() != QtCore.QProcess.NotRunning:
            self.stop()
            event.ignore()
            QtCore.QTimer.singleShot(1500, self.close)
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--session", type=Path)
    parser.add_argument("--screenshot", type=Path, help="Render existing real evidence, then exit")
    options = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    standard_path = Path(get_package_share_directory("robocon_fusion")) / "config/bench_standard_v1.yaml"
    application = QtWidgets.QApplication(sys.argv[:1])
    application.setFont(QtGui.QFont("Noto Sans CJK SC", 10))
    window = BenchWindow(options.repo.resolve(), standard_path, options.session)
    window.show()
    if options.screenshot:
        def snapshot():
            window.refresh()
            window.grab().save(str(options.screenshot))
            application.quit()
        QtCore.QTimer.singleShot(1200, snapshot)
    return application.exec_()
