#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 目标检测离线单测（无需硬件）：合成图像颜色分割。

运行：
  python test_detect.py
  pytest test_detect.py
"""

import sys
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from my_arm_control.detect import ColorTargetDetector  # noqa: E402


def _synthetic_frame(rects: list[tuple[tuple[int, int, int, int], tuple[int, int, int]]],
                     size=(320, 240), bg=(240, 240, 240)) -> np.ndarray:
    """生成合成 BGR 画面：rects = [(x, y, w, h, BGR)]。"""
    frame = np.full((size[1], size[0], 3), bg, np.uint8)
    for (x, y, w, h), color in rects:
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, -1)
    return frame


def test_detect_single_red():
    """单个红色方块 → 检测到中心与面积。"""
    frame = _synthetic_frame([((100, 80, 60, 60), (0, 0, 255))])
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255], area_range=(100, 50000))
    target = det.detect_one(frame)
    assert target is not None
    assert abs(target.x - 130) <= 3 and abs(target.y - 110) <= 3
    assert target.area > 3000  # 60*60=3600


def test_detect_wrap_around_hsv():
    """红色在 HSV 0/179 环绕：hsv=[170,10,...] 应同时命中高/低 H。"""
    frame = _synthetic_frame([((50, 50, 40, 40), (0, 0, 255))])
    det = ColorTargetDetector(hsv=[170, 10, 80, 255, 60, 255], area_range=(100, 50000))
    assert det.detect_one(frame) is not None


def test_detect_none_when_missing():
    """无目标颜色 → 返回 None。"""
    frame = _synthetic_frame([((100, 80, 60, 60), (0, 255, 0))])  # 绿色
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255], area_range=(100, 50000))
    assert det.detect_one(frame) is None


def test_detect_area_filter():
    """面积过小（噪点）被过滤。"""
    frame = _synthetic_frame([((150, 100, 3, 3), (0, 0, 255))])  # 9 px
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255], area_range=(100, 50000))
    assert det.detect_one(frame) is None


def test_detect_picks_largest():
    """多目标 → 返回面积最大者（max_targets=1）。"""
    frame = _synthetic_frame([
        ((30, 30, 30, 30), (0, 0, 255)),   # 900 px
        ((160, 120, 80, 80), (0, 0, 255)),  # 6400 px
    ])
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255], area_range=(100, 50000))
    target = det.detect_one(frame)
    assert target is not None and abs(target.x - 200) <= 3 and abs(target.y - 160) <= 3


def test_detect_mask_shape():
    """mask 输出单通道二值图。"""
    frame = _synthetic_frame([((100, 80, 60, 60), (0, 0, 255))])
    det = ColorTargetDetector(hsv=[0, 10, 80, 255, 60, 255])
    mask = det.mask(frame)
    assert mask.shape == frame.shape[:2]
    assert mask.dtype == np.uint8


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
