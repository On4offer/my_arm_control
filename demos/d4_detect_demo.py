#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 Demo：目标检测调试（实时画面 + HSV 滑杆调参）
==================================================

用途：找到目标的 HSV 范围（颜色分割），供 d4_config.json 的 target_detect.hsv 使用。

操作：
  鼠标拖动 6 个滑杆（H/S/V 的 min/max）；画面实时显示检测框与目标中心；
  按 s 打印当前 HSV 与检测结果；按 q 退出。

用法：
  python d4_detect_demo.py
  python d4_detect_demo.py --camera 0
"""

import argparse
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from d4_common import load_d4_config, make_camera, make_detector  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：目标检测调试（HSV 滑杆）")
    p.add_argument("--camera", type=int, default=None, help="相机索引（默认取 d4_config.json）")
    p.add_argument("--no-undistort", action="store_true", help="不去畸变（默认若已有内参则去畸变）")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config()
    if args.camera is not None:
        config["hardware"]["camera_index"] = args.camera
    cam = make_camera(config)
    det = make_detector(config)
    hsv = list(config["target_detect"]["hsv"])

    win = "d4_detect"
    cv2.namedWindow(win)
    cv2.createTrackbar("H_min", win, hsv[0], 179, lambda v: None)
    cv2.createTrackbar("H_max", win, hsv[1], 179, lambda v: None)
    cv2.createTrackbar("S_min", win, hsv[2], 255, lambda v: None)
    cv2.createTrackbar("S_max", win, hsv[3], 255, lambda v: None)
    cv2.createTrackbar("V_min", win, hsv[4], 255, lambda v: None)
    cv2.createTrackbar("V_max", win, hsv[5], 255, lambda v: None)
    print("拖动滑杆调 HSV；s=打印当前值；q=退出")

    while True:
        frame = cam.read_undistorted() if not args.no_undistort else cam.read()
        cur = [
            cv2.getTrackbarPos("H_min", win), cv2.getTrackbarPos("H_max", win),
            cv2.getTrackbarPos("S_min", win), cv2.getTrackbarPos("S_max", win),
            cv2.getTrackbarPos("V_min", win), cv2.getTrackbarPos("V_max", win),
        ]
        det.hsv_min = np.array([cur[0], cur[2], cur[4]], np.uint8)
        det.hsv_max = np.array([cur[1], cur[3], cur[5]], np.uint8)
        targets = det.detect(frame)
        display = det.draw(frame, targets)
        if targets:
            t = targets[0]
            cv2.putText(display, f"center=({t.x},{t.y}) area={t.area:.0f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"HSV=[{cur[0]},{cur[1]},{cur[2]},{cur[3]},{cur[4]},{cur[5]}]", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow(win, display)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            print(f"HSV={cur}  targets={[(t.x, t.y, int(t.area)) for t in targets]}")
        elif key == ord("q"):
            break
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
