#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 抓取闭环离线单测（无需硬件）：mock 总线/运动控制，验证状态机与容错。

场景：
  1. 夹爪咬住物体（present 停在关闭目标前 250 码）→ 判成功
  2. 夹爪关到底（空抓）→ 判失败并重试
  3. 画面无目标 → 检测失败计数

运行：
  python test_grasp.py
  pytest test_grasp.py
"""

import json
import sys
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from my_arm_control.calibration import TableCalibration  # noqa: E402
from my_arm_control.detect import ColorTargetDetector  # noqa: E402
from my_arm_control.grasp import GraspController  # noqa: E402
from my_arm_control.kinematics import SO101Kinematics, load_calibration  # noqa: E402

CONFIG = json.loads(
    (Path(__file__).resolve().parents[1] / "config" / "d4_config.json").read_text(encoding="utf-8")
)
CALIB = {
    "shoulder_pan": {"mid": 2095.0, "min": 814.0, "max": 3376.0},
    "shoulder_lift": {"mid": 2036.5, "min": 841.0, "max": 3232.0},
    "elbow_flex": {"mid": 1969.5, "min": 866.0, "max": 3073.0},
    "wrist_flex": {"mid": 2029.0, "min": 905.0, "max": 3153.0},
    "wrist_roll": {"mid": 2047.5, "min": 0.0, "max": 4095.0},
    "gripper": {"mid": 1993.0, "min": 1560.0, "max": 2426.0},
}


class FakeBus:
    """present=goal；gripper_stall>0 时夹爪 present 停在目标前 N 码并置过载位 0x20（模拟咬住物体/堵转）；
    gripper_frozen=True 时夹爪 present 恒为张开位且无过载位（模拟夹爪故障，回归误报场景）。"""

    def __init__(self, gripper_stall: int = 0, gripper_frozen: bool = False):
        self.goal: dict[int, int] = {}
        self.stall = gripper_stall
        self.frozen = gripper_frozen

    def write_u16(self, servo_id, addr, value):  # noqa: ANN001
        if addr == 42:
            self.goal[servo_id] = value

    def read_u16(self, servo_id, addr):  # noqa: ANN001
        if addr == 56:
            if self.frozen and servo_id == 6:
                return 2426  # 夹爪堵死在张开位（离 close_code 很远）
            g = self.goal.get(servo_id, 2000)
            return g - self.stall if self.stall > 0 and servo_id == 6 else g
        return 0

    def read_u16_soft(self, servo_id, addr):  # noqa: ANN001
        """容错读：返回 (值, 错误位)。夹爪堵转（stall）时置过载位 0x20。"""
        if addr == 56:
            if self.frozen and servo_id == 6:
                return 2426, 0
            g = self.goal.get(servo_id, 2000)
            if self.stall > 0 and servo_id == 6:
                return g - self.stall, 0x20
            return g, 0
        return 0, 0

    def write_u8(self, *a):  # noqa: ANN001
        pass


class FakeArm:
    def __init__(self, bus):  # noqa: ANN001
        self.bus = bus
        self.servo_ids = [1, 2, 3, 4, 5, 6]

    def read_positions(self) -> dict[int, int]:
        return {i: 2000 for i in self.servo_ids}

    def get_limits(self) -> dict[int, tuple[int, int]]:
        return {i: (800, 3400) for i in self.servo_ids}

    def enable_torque(self):  # noqa: ANN201
        pass

    def move_to(self, targets, **kw):  # noqa: ANN001
        for sid, v in targets.items():
            self.bus.goal[sid] = int(v)
        return {}


def _make_ctrl(stall: int, frozen: bool = False) -> GraspController:
    kin = SO101Kinematics(CONFIG, CALIB)
    kin.safe_z_min = None  # mock 场景不受 z_safe_min 防撞护栏限制
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255], area_range=(100, 50000))
    cal = TableCalibration()
    for bx, by, u, v in [(0.12, -0.05, 200, 300), (0.20, -0.05, 600, 300),
                         (0.20, 0.05, 600, 180), (0.12, 0.05, 200, 180), (0.16, 0.0, 400, 240)]:
        cal.add_point((u, v), (bx, by))
    cal.solve()
    bus = FakeBus(stall, frozen)
    ctrl = GraspController(CONFIG, kin, cal, det, bus, FakeArm(bus))
    ctrl._camera_read = lambda: np.zeros((480, 640, 3), np.uint8)
    return ctrl


def _target_frame() -> np.ndarray:
    frame = np.zeros((480, 640, 3), np.uint8)
    cv2.rectangle(frame, (380, 220), (420, 260), (0, 0, 255), -1)  # 中心 (400,240)
    return frame


def test_grasp_success_when_jaw_stalled():
    """夹爪咬住（堵转 250 码）→ 单轮成功。"""
    ctrl = _make_ctrl(stall=250)
    ok = ctrl.grasp_once(_target_frame(), log=lambda s: None)
    assert ok and ctrl.stats["success"] == 1 and ctrl.stats["fail"] == 0


def test_grasp_fail_when_empty():
    """夹爪关到底（空抓）→ 失败并计入 grasp_fail。"""
    ctrl = _make_ctrl(stall=0)
    ok = ctrl.grasp_once(_target_frame(), log=lambda s: None)
    assert not ok and ctrl.stats["fail"] == 1
    assert ctrl.stats["fail_reasons"].get("grasp_fail", 0) == 1


def test_grasp_detect_fail_when_no_target():
    """画面无目标 → 检测失败，不移动。"""
    ctrl = _make_ctrl(stall=250)
    ctrl.detect_timeout = 1.0
    ctrl.max_retries = 1
    ok = ctrl.grasp_once(np.zeros((480, 640, 3), np.uint8), log=lambda s: None)
    assert not ok and ctrl.stats["fail_reasons"].get("detect_fail", 0) == 1


def test_grasp_fail_when_gripper_frozen_open():
    """夹爪没动（堵死在张开位，离 close_code 很远）→ 不得误报成功（旧逻辑 bug 回归）。"""
    ctrl = _make_ctrl(stall=0, frozen=True)
    ok = ctrl.grasp_once(_target_frame(), log=lambda s: None)
    assert not ok and ctrl.stats["fail"] == 1
    assert ctrl.stats["fail_reasons"].get("grasp_fail", 0) == 1


def test_run_trials_stats():
    """run_trials 汇总统计与成功率计算（相机帧含目标）。"""
    ctrl = _make_ctrl(stall=250)
    ctrl._camera_read = _target_frame
    result = ctrl.run_trials(n=3, log=lambda s: None)
    assert result["trials"] == 3 and result["success"] == 3
    assert abs(result["success_rate"] - 100.0) < 1e-6


if __name__ == "__main__":
    import traceback

    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n汇总: {passed} PASS / {len(fns) - passed} FAIL / {len(fns)} 总检查项")
    sys.exit(0 if passed == len(fns) else 1)
