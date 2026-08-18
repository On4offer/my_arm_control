#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 运动学离线单测（无需硬件）：FK/IK 往返、可达性、限位、角度换算。

运行：
  python test_kinematics.py
  pytest test_kinematics.py
"""

import json
import math
import sys
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.kinematics import (
    KinematicsError,
    PathUnsafeError,
    SO101Kinematics,
    code_to_deg,
    deg_to_code,
    load_calibration,
)

# 用真实从臂校准（COM24）测试（离线，只读文件）
CALIB = {
    "shoulder_pan": {"mid": 2095.0, "min": 814.0, "max": 3376.0},
    "shoulder_lift": {"mid": 2036.5, "min": 841.0, "max": 3232.0},
    "elbow_flex": {"mid": 1969.5, "min": 866.0, "max": 3073.0},
    "wrist_flex": {"mid": 2029.0, "min": 905.0, "max": 3153.0},
    "wrist_roll": {"mid": 2047.5, "min": 0.0, "max": 4095.0},
    "gripper": {"mid": 1993.0, "min": 1560.0, "max": 2426.0},
}
CONFIG = {
    "kinematics": {
        "shoulder_x": 0.0388, "shoulder_y": 0.0, "shoulder_h": 0.0624,
        "link1": 0.1776, "link2": 0.1350, "wrist_to_tip": 0.0980,
        "sign": {"shoulder_pan": 1.0, "shoulder_lift": 1.0, "elbow_flex": 1.0, "wrist_flex": 1.0},
        "offset_deg": {"shoulder_pan": 0.0, "shoulder_lift": 19.3, "elbow_flex": 68.5, "wrist_flex": 5.0},
        "elbow_branch": "up",
    }
}
# 桌面工作区内的典型抓取目标（基座系，米）
REACHABLE = [(0.15, 0.05, 0.08), (0.20, -0.05, 0.02), (0.10, 0.0, 0.005), (0.24, 0.08, 0.03), (0.05, 0.05, 0.005)]


def _kin() -> SO101Kinematics:
    return SO101Kinematics(CONFIG, CALIB)


def test_deg_code_roundtrip():
    """角度↔码值往返（整数量化误差 < 0.1°）。"""
    kin = _kin()
    for j in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]:
        for deg in [-90.0, -30.0, 0.0, 30.0, 90.0]:
            rt = kin.code_to_deg(j, kin.deg_to_code(j, deg))
            assert abs(rt - deg) < 0.1, (j, deg, rt)


def test_deg_code_mid_is_zero():
    """量程中点码值 → 0°（LeRobot DEGREES 约定；mid 可为小数，量化误差 <0.1°）。"""
    kin = _kin()
    for j, c in CALIB.items():
        assert abs(kin.code_to_deg(j, int(round(c["mid"])))) < 0.1
        assert abs(kin.deg_to_code(j, 0.0) - c["mid"]) <= 1.0


def test_ik_fk_roundtrip():
    """垂直抓取 IK → FK 还原指尖位置（误差 < 1mm）。"""
    kin = _kin()
    for x, y, z in REACHABLE:
        joints = kin.ik_vertical(x, y, z)
        assert set(joints) == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"}
        for j, v in joints.items():
            lo, hi = kin.limits_deg[j]
            assert lo - 1e-6 <= v <= hi + 1e-6, f"{j} 超限: {v}"
        tip = kin.fk({**joints, "wrist_roll": 0.0, "gripper": 0.0})
        err = math.dist(tip, (x, y, z))
        assert err < 1e-3, f"({x},{y},{z}) 误差 {err * 1000:.2f}mm"


def test_ik_gripper_vertical():
    """垂直抓取姿态：φ2+φ3+φ4 = 180°（夹爪竖直朝下）。"""
    kin = _kin()
    for x, y, z in REACHABLE:
        joints = kin.ik_vertical(x, y, z)
        phi2 = kin._phi2(joints["shoulder_lift"])
        phi3 = kin._phi3(joints["elbow_flex"])
        phi4 = kin._phi4(joints["wrist_flex"])
        assert abs(phi2 + phi3 + phi4 - math.pi) < 1e-9


def test_ik_unreachable_raises():
    """超出臂展 / 与肩轴重合 → 抛 KinematicsError。"""
    kin = _kin()
    for target in [(1.0, 0.0, 0.05), (0.0, 0.0, 0.6), (-1.0, -1.0, 0.1), (0.30, 0.30, 0.05)]:
        try:
            kin.ik_vertical(*target)
            assert False, f"应报不可达: {target}"
        except KinematicsError:
            pass


def test_fk_zero_config():
    """零位（量程中点）FK 与模型一致（URDF new_calib 推导的 offset 默认值）：
    指尖 ≈ (0.33, 0, 0.23)m（肩轴前伸、前臂水平、夹爪朝前）。"""
    kin = _kin()
    tip = kin.fk({"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                  "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 0.0})
    assert abs(tip[0] - 0.33) < 0.02 and abs(tip[1]) < 0.02 and abs(tip[2] - 0.23) < 0.02, tip


def test_clamp_joint():
    kin = _kin()
    assert kin.clamp_joint("shoulder_pan", -1000.0) == kin.limits_deg["shoulder_pan"][0]
    assert kin.clamp_joint("shoulder_pan", 1000.0) == kin.limits_deg["shoulder_pan"][1]
    assert kin.clamp_joint("shoulder_pan", 10.0) == 10.0


def test_home_pose():
    kin = _kin()
    home = kin.home_pose()
    assert home["shoulder_pan"] == 0.0 and home["shoulder_lift"] == 0.0
    assert set(home) == set(CALIB)


def test_path_safe_allows_high_path():
    """路径级护栏：两端点都高、插值不触底的路径放行。"""
    kin = _kin()
    cur = kin.ik_vertical(0.10, 0.0, 0.14)
    tgt = kin.ik_vertical(0.16, 0.0, 0.14)
    cur["wrist_roll"] = tgt["wrist_roll"] = 0.0
    kin.check_path_safe(cur, tgt, z_min=0.075)  # 不抛异常即通过


def test_path_safe_rejects_dip():
    """路径级护栏：端点 z 都高于下限、但插值中途下坠 → 抛 PathUnsafeError（拒绝执行）。

    用测试 config（sign 全 +1）数值搜索出的确定性用例：lift=-20°/elbow=130°/
    wrist=90°（端点指尖 z=0.149，高位）线性插值到 (0.15, 0, 0.10) 的垂直位姿，
    中途指尖最低 z≈0.061，跌破 0.075 → 必须拒绝，防止"起点高但路径下坠"撞桌。
    """
    kin = _kin()
    cur = {"shoulder_pan": 0.0, "shoulder_lift": -20.0, "elbow_flex": 130.0,
           "wrist_flex": 90.0, "wrist_roll": 0.0}
    tgt = kin.ik_vertical(0.15, 0.0, 0.10)
    tgt["wrist_roll"] = 0.0
    assert kin.fk(cur)[2] > 0.14  # 端点确实在高位
    try:
        kin.check_path_safe(cur, tgt, z_min=0.075)
        assert False, "应抛 PathUnsafeError（路径下坠）"
    except PathUnsafeError:
        pass


def test_path_safe_skips_when_zmin_none():
    """z_min=None（探高受控模式）→ 跳过路径校验，不抛异常。"""
    kin = _kin()
    cur = {"shoulder_pan": 0.0, "shoulder_lift": -70.0, "elbow_flex": 120.0,
           "wrist_flex": -50.0, "wrist_roll": 0.0}
    tgt = kin.ik_vertical(0.15, 0.0, 0.10)
    tgt["wrist_roll"] = 0.0
    kin.check_path_safe(cur, tgt, z_min=None)


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
