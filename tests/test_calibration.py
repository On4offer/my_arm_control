#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 手眼标定（平面单应）离线单测（无需硬件）：合成对应点求解与重投影。

运行：
  python test_calibration.py
  pytest test_calibration.py
"""

import json
import sys
import tempfile
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from my_arm_control.calibration import CalibrationError, TableCalibration, grid_xy  # noqa: E402


def _synthetic_calibration(n: int = 9, noise_px: float = 0.0) -> TableCalibration:
    """构造"真实"单应（基座→像素），生成 n 个对应点，可选加像素噪声。"""
    H_true = np.array([
        [800.0, 120.0, 320.0],
        [-50.0, 900.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    cal = TableCalibration()
    rng = np.random.default_rng(42)
    for _ in range(n):
        bx, by = rng.uniform(0.08, 0.25), rng.uniform(-0.08, 0.08)
        p = H_true @ np.array([bx, by, 1.0])
        u, v = p[0] / p[2], p[1] / p[2]
        u += rng.normal(0, noise_px)
        v += rng.normal(0, noise_px)
        cal.add_point((u, v), (bx, by))
    return cal


def test_solve_exact():
    """无噪声：求解后重投影误差 ≈ 0（<0.01mm / 1e-3px，含数值精度）。"""
    cal = _synthetic_calibration(noise_px=0.0)
    err = cal.solve()
    assert err["rms_mm"] < 0.01
    assert err["rms_px"] < 1e-3


def test_solve_with_noise():
    """有像素噪声：RMS 误差有界（< 5mm）。"""
    cal = _synthetic_calibration(noise_px=2.0)
    err = cal.solve()
    assert err["rms_mm"] < 5.0


def test_pixel_to_base_roundtrip():
    """像素→基座→像素 往返。"""
    cal = _synthetic_calibration()
    cal.solve()
    bx, by = 0.15, 0.03
    u, v = cal.base_to_pixel(bx, by)
    bx2, by2 = cal.pixel_to_base(u, v)
    assert abs(bx - bx2) < 1e-3 and abs(by - by2) < 1e-3


def test_insufficient_points():
    """点数 <4 → 抛 CalibrationError。"""
    cal = TableCalibration()
    cal.add_point((100, 100), (0.1, 0.0))
    cal.add_point((200, 100), (0.2, 0.0))
    cal.add_point((100, 200), (0.1, 0.1))
    try:
        cal.solve()
        assert False, "应报点数不足"
    except CalibrationError:
        pass


def test_save_load():
    """保存/加载往返一致。"""
    cal = _synthetic_calibration()
    cal.solve()
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "calib.json"
        cal.save(path)
        loaded = TableCalibration.load(path)
        u, v = 320.0, 240.0
        bx1, by1 = cal.pixel_to_base(u, v)
        bx2, by2 = loaded.pixel_to_base(u, v)
        assert abs(bx1 - bx2) < 1e-6 and abs(by1 - by2) < 1e-6
        assert loaded.n_points() == cal.n_points()


def test_grid_xy():
    """网格生成：3x3 共 9 点、边界正确。"""
    pts = grid_xy(0.10, 0.24, 3, -0.08, 0.08, 3)
    assert len(pts) == 9
    assert (0.10, -0.08) == pts[0]
    assert (0.24, 0.08) == pts[-1]


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
