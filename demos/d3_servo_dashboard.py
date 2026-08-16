#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D3 舵机调试上位机（PySide6 / Qt）
=================================

工业上位机/示教器主流技术栈（Qt）实现的机械臂舵机调试面板：

  - 实时状态：6 舵机 位置/电压/温度/错误位/限位 定时刷新（QTimer 事件循环）
  - 单关节控制：滑块 + 数值框写 Goal_Position，范围锁定在限位内
  - 使能/失能力矩、急停、控制模式开关（安全设计）

架构要点（面试/README 话术）：
  - 信号槽（signal/slot）：滑块 valueChanged → 写舵机；按钮 clicked → 连接/急停
  - 事件循环 + 定时器：QTimer 周期轮询串口刷新 UI，单线程内串行访问串口
  - UI 状态与串口状态解耦：控制模式门控 + 每关节使能门控

用法：
  python d3_servo_dashboard.py                 # 启动面板，下拉选择 COM 口连接
  python d3_servo_dashboard.py --smoke         # 冒烟自检（不连硬件，1.5s 后自动退出）
  python d3_servo_dashboard.py --port COM22    # 指定端口并自动连接
"""

import argparse
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from my_arm_control.protocol import ADDR, DEFAULT_BAUDRATE, FeetechSerialBus, ProtocolError, counts_to_angle, decode_sign_magnitude, encode_sign_magnitude

POLL_MS = 100          # 状态刷新周期（10Hz）
WRITE_MIN_INTERVAL = 0.03  # 滑块连续写 Goal 的最小间隔（秒），防刷爆串口


class ServoDashboard(QMainWindow):
    """舵机调试上位机主窗口。"""

    def __init__(self, port: str | None = None):
        super().__init__()
        self.setWindowTitle("SO-ARM101 舵机调试上位机 (D3 / PySide6)")
        self.resize(860, 640)

        self.bus: FeetechSerialBus | None = None
        self.control_enabled = False   # 控制模式门控（默认关闭）
        self.servo_rows: dict[int, dict] = {}   # sid -> {chk, slider, spin, deg, state}
        self._last_write: dict[int, float] = {}

        self._build_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._poll_once)
        self.timer.start(POLL_MS)

        if port:
            self.port_combo.setCurrentText(port)
            self._connect()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # 连接区
        top = QHBoxLayout()
        top.addWidget(QLabel("端口:"))
        self.port_combo = QComboBox()
        self._refresh_ports()
        top.addWidget(self.port_combo)
        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self._connect)
        top.addWidget(self.btn_connect)
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.clicked.connect(self._disconnect)
        top.addWidget(self.btn_disconnect)
        top.addStretch(1)
        self.chk_control = QCheckBox("控制模式")
        self.chk_control.setToolTip("开启后滑块才允许写 Goal_Position（安全门控）")
        self.chk_control.stateChanged.connect(self._on_control_mode)
        top.addWidget(self.chk_control)
        self.btn_estop = QPushButton("急停")
        self.btn_estop.setStyleSheet("background:#d33;color:#fff;font-weight:bold;padding:4px 16px;")
        self.btn_estop.clicked.connect(self._estop)
        top.addWidget(self.btn_estop)
        root.addLayout(top)

        # 实时状态表
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["ID", "型号", "位置(码)", "角度(°)", "电压(V)", "温度(°C)", "错误"])
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table)

        # 关节控制区
        box = QGroupBox("关节控制（滑块 = Goal_Position，范围锁定限位）")
        self.joint_layout = QVBoxLayout(box)
        self.hint = QLabel("提示：勾选[控制模式] + 每关节[使能]后，拖动滑块即写入目标位置。")
        self.hint.setStyleSheet("color:#888;")
        self.joint_layout.addWidget(self.hint)
        root.addWidget(box)

        # 日志区
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(100)
        root.addWidget(self.log_view)

    def _refresh_ports(self):
        from serial.tools import list_ports
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device}  ({p.description})", p.device)

    def _log(self, msg: str):
        self.log_view.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")

    # ----------------------------------------------------------- 连接管理
    def _connect(self):
        if self.bus is not None:
            self._log("已连接，请先断开")
            return
        port = self.port_combo.currentData() or self.port_combo.currentText()
        try:
            bus = FeetechSerialBus(port=port, baudrate=DEFAULT_BAUDRATE)
            models = bus.scan()
        except Exception as e:
            self._log(f"连接失败: {e}")
            return
        if not models:
            bus.close()
            self._log("总线无应答：请检查 12V 电源与 BusLinker")
            return
        self.bus = bus
        self._log(f"已连接 {port}，发现 {len(models)} 舵机: {sorted(models)}")

        # 为每个舵机建立状态行 + 控制行
        self.servo_rows.clear()
        self.table.setRowCount(0)
        for sid in sorted(models):
            self._add_servo_row(sid, models[sid])
        self.chk_control.setChecked(False)  # 连接后默认关闭控制

    def _disconnect(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None
        self.servo_rows.clear()
        self.table.setRowCount(0)
        self._log("已断开")

    def _add_servo_row(self, sid: int, model: int):
        # 读限位（决定滑块范围）
        try:
            lo = self.bus.read_u16(sid, ADDR["min_position_limit"][0])
            hi = self.bus.read_u16(sid, ADDR["max_position_limit"][0])
        except ProtocolError as e:
            lo, hi = 0, 4095
            self._log(f"ID={sid} 读限位失败: {e}")

        # 状态表行
        r = self.table.rowCount()
        self.table.insertRow(r)
        for col, txt in enumerate([str(sid), str(model), "-", "-", "-", "-", "-"]):
            self.table.setItem(r, col, QTableWidgetItem(txt))

        # 控制行：使能 | ID | 滑块 | 数值框 | 角度
        row = QHBoxLayout()
        chk = QCheckBox("使能")
        chk.stateChanged.connect(lambda state, s=sid: self._on_torque(s, state))
        row.addWidget(chk)
        row.addWidget(QLabel(f"ID={sid}"))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(lo)  # 占位，轮询时刷新
        slider.valueChanged.connect(lambda v, s=sid: self._on_slider(s, v))
        slider.sliderReleased.connect(lambda s=sid: self._write_goal(s))
        row.addWidget(slider, 1)
        spin = QSpinBox()
        spin.setRange(lo, hi)
        spin.valueChanged.connect(lambda v, s=sid: self._on_spin(s, v))
        row.addWidget(spin)
        deg = QLabel("-°")
        deg.setMinimumWidth(70)
        row.addWidget(deg)
        self.joint_layout.addLayout(row)

        self.servo_rows[sid] = {
            "chk": chk, "slider": slider, "spin": spin, "deg": deg,
            "lo": lo, "hi": hi, "model": model,
            "state": {},   # 轮询填充: present/volt/temp/err
        }

    # ----------------------------------------------------------- 事件槽
    def _on_control_mode(self, state: int):
        self.control_enabled = state == Qt.CheckState.Checked.value
        self._log(f"控制模式 {'开启' if self.control_enabled else '关闭'}")

    def _on_torque(self, sid: int, state: int):
        """使能/失能力矩。"""
        if self.bus is None:
            return
        val = 1 if state == Qt.CheckState.Checked.value else 0
        try:
            self.bus.write_u8(sid, ADDR["torque_enable"][0], val)
            self._log(f"ID={sid} 力矩 {'使能' if val else '失能'}")
        except ProtocolError as e:
            self._log(f"ID={sid} 写使能失败: {e}")

    def _on_slider(self, sid: int, value: int):
        """滑块变化 → 同步数值框/角度，并在控制模式下写入 Goal。"""
        row = self.servo_rows.get(sid)
        if row is None:
            return
        row["spin"].blockSignals(True)
        row["spin"].setValue(value)
        row["spin"].blockSignals(False)
        row["deg"].setText(f"{counts_to_angle(value):.1f}°")
        if self.control_enabled and row["chk"].isChecked():
            self._write_goal(sid)

    def _on_spin(self, sid: int, value: int):
        row = self.servo_rows.get(sid)
        if row is None:
            return
        row["slider"].blockSignals(True)
        row["slider"].setValue(value)
        row["slider"].blockSignals(False)

    def _write_goal(self, sid: int):
        """写 Goal_Position（带 30ms 节流，防连续事件刷爆串口）。"""
        if self.bus is None or not self.control_enabled:
            return
        now = time.monotonic()
        if now - self._last_write.get(sid, 0.0) < WRITE_MIN_INTERVAL:
            return
        self._last_write[sid] = now
        row = self.servo_rows.get(sid)
        if row is None:
            return
        try:
            self.bus.write_u16(sid, ADDR["goal_position"][0], encode_sign_magnitude(row["slider"].value()))
        except ProtocolError as e:
            self._log(f"ID={sid} 写 Goal 失败: {e}")

    def _estop(self):
        """急停：全部失能 + 关闭控制模式。"""
        self.chk_control.setChecked(False)
        if self.bus is not None:
            for sid in self.servo_rows:
                try:
                    self.bus.write_u8(sid, ADDR["torque_enable"][0], 0)
                except ProtocolError:
                    pass
        self._log("⚠ 急停：所有舵机失能")
        self.setStyleSheet("")  # 复位样式（若之前标红）

    # ----------------------------------------------------------- 轮询刷新
    def _poll_once(self):
        if self.bus is None:
            return
        for sid, row in self.servo_rows.items():
            st = {}
            try:
                st["present"] = decode_sign_magnitude(self.bus.read_u16(sid, ADDR["present_position"][0]))
                st["volt"] = self.bus.read_u8(sid, ADDR["present_voltage"][0]) / 10.0
                st["temp"] = self.bus.read_u8(sid, ADDR["present_temperature"][0])
            except ProtocolError as e:
                st["err"] = str(e)
            row["state"] = st
            self._update_table_row(sid, row, st)

            # 滑块同步实际位置（仅在用户未拖动时）
            if not row["slider"].isSliderDown() and "present" in st:
                row["slider"].blockSignals(True)
                row["slider"].setValue(st["present"])
                row["slider"].blockSignals(False)
                row["deg"].setText(f"{counts_to_angle(st['present']):.1f}°")

    def _update_table_row(self, sid: int, row: dict, st: dict):
        # 用 sid 定位行：按 ID 列匹配
        for r in range(self.table.rowCount()):
            if self.table.item(r, 0) and int(self.table.item(r, 0).text()) == sid:
                if "err" in st:
                    self.table.setItem(r, 6, QTableWidgetItem("通信错误"))
                    return
                self.table.setItem(r, 2, QTableWidgetItem(str(st["present"])))
                self.table.setItem(r, 3, QTableWidgetItem(f"{counts_to_angle(st['present']):.1f}"))
                self.table.setItem(r, 4, QTableWidgetItem(f"{st['volt']:.1f}"))
                self.table.setItem(r, 5, QTableWidgetItem(str(st["temp"])))
                self.table.setItem(r, 6, QTableWidgetItem("正常"))
                self.table.item(r, 6).setForeground(QColor("#0a0"))
                return


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D3 舵机调试上位机（PySide6/Qt）")
    p.add_argument("--port", default=None, help="串口（如 COM22），指定则自动连接")
    p.add_argument("--smoke", action="store_true", help="冒烟自检：构建窗口后 1.5s 自动退出（不连硬件）")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    app = QApplication(sys.argv)
    win = ServoDashboard(port=args.port)
    win.show()
    if args.smoke:
        QTimer.singleShot(1500, app.quit)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
