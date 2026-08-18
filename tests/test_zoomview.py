#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 标定工具：缩放/平移视图 + 滚轮方向 离线单测（无需硬件/无 GUI）。

运行：
  pytest tests/test_zoomview.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "demos"))

import numpy as np  # noqa: E402

from d4_calibrate_table import ZoomPanView, _wheel_direction  # noqa: E402

W, H = 640, 480


def test_window_to_orig_identity_at_scale1():
    """scale=1、视图在中心时：窗口坐标 = 原始坐标（与未缩放时行为一致）。"""
    v = ZoomPanView(W, H)
    for (wx, wy) in [(0, 0), (320, 240), (639, 479)]:
        ox, oy = v.window_to_orig(wx, wy)
        assert abs(ox - wx) < 1e-6 and abs(oy - wy) < 1e-6


def test_zoom_keeps_cursor_point_fixed():
    """缩放前后，光标所在位置对应的原始坐标必须保持不变。"""
    v = ZoomPanView(W, H)
    cursor = (500, 300)
    before = v.window_to_orig(*cursor)
    v.zoom_at(*cursor, 2.0)
    after = v.window_to_orig(*cursor)
    assert abs(before[0] - after[0]) < 1e-6
    assert abs(before[1] - after[1]) < 1e-6
    assert v.scale == 2.0


def test_roundtrip_orig_window():
    """原始坐标 → 窗口坐标 → 原始坐标 往返一致。"""
    v = ZoomPanView(W, H)
    v.zoom_at(400, 200, 3.0)
    v.pan_by(-50, 30)
    for (ox, oy) in [(100, 100), (320, 240), (500, 400)]:
        wx, wy = v.orig_to_window(ox, oy)
        rx, ry = v.window_to_orig(wx, wy)
        assert abs(rx - ox) < 1e-6 and abs(ry - oy) < 1e-6


def test_view_stays_in_bounds():
    """疯狂平移/缩放后，视口中心必须仍在图像内（避免越界裁剪/黑边）。"""
    v = ZoomPanView(W, H)
    for _ in range(50):
        v.pan_by(1000, 1000)
    v.zoom_at(0, 0, 1.0 / 10.0)  # 缩小到下限
    v.zoom_at(0, 0, 100.0)  # 放大到上限
    half_w = W / (2.0 * v.scale)
    half_h = H / (2.0 * v.scale)
    assert half_w <= v.cx <= W - half_w
    assert half_h <= v.cy <= H - half_h


def test_scale_clamped():
    v = ZoomPanView(W, H)
    v.zoom_at(320, 240, 1000.0)
    assert v.scale == v.max_scale
    v.zoom_at(320, 240, 1e-9)
    assert v.scale == v.min_scale


def test_render_shapes():
    """render 输出必须与窗口等尺寸；scale=1 时是原图。"""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    v = ZoomPanView(W, H)
    out1 = v.render(frame)
    assert out1.shape == (H, W, 3)
    assert np.array_equal(out1, frame)
    v.zoom_at(320, 240, 3.0)
    out2 = v.render(frame)
    assert out2.shape == (H, W, 3)
    v.pan_by(-1000, 1000)  # 边界情况也要能正常渲染
    out3 = v.render(frame)
    assert out3.shape == (H, W, 3)


def test_wheel_direction_win32():
    """Win32 后端：上滚=CTRLKEY(8)，下滚=SHIFTKEY(16)。"""
    assert _wheel_direction(8) == 1      # 上滚
    assert _wheel_direction(16) == -1    # 下滚
    assert _wheel_direction(8 | 1) == 1  # 上滚+左键按住


def test_wheel_direction_qt():
    """Qt/Cocoa 后端：delta 编码在高 16 位。"""
    assert _wheel_direction(120 << 16) == 1        # 上滚 +120
    assert _wheel_direction((0x10000 - 120) << 16) == -1  # 下滚 -120（低16位0）
    assert _wheel_direction(0) == 0                # 无法判断


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
