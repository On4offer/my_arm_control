#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 相机选择器：可视化确认哪台是工作台固定相机，并写入 d4_config.json
=====================================================================

背景：Windows 上 USB 相机换插口后 OpenCV index 会变（0/1/2...），且无法从
index 区分相机身份。本工具逐个显示可用相机画面，让你亲眼确认并保存。

操作：
  按数字键 0/1/2... 切换查看对应相机画面（画面左上角显示 index）
  看到【工作台固定相机】画面 → 按 s 保存该 index 到 d4_config.json
  按 q 退出

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_camera_select.py
"""

import argparse
import json
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from d4_common import D4_CONFIG, load_d4_config  # noqa: E402
from my_arm_control.vision import CameraView, scan_cameras  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：相机选择器（确认工作台固定相机并写配置）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="配置文件路径")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    load_d4_config(args.config)  # 校验配置可读
    cams = scan_cameras()
    if not cams:
        print("未检测到任何相机")
        return 1
    print("可用相机:", [c["index"] for c in cams])
    print("操作：按数字键 0/1/2... 切换画面；看到【工作台固定相机】按 s 保存；q 退出")

    cap = None
    current = cams[0]["index"]  # 默认打开第一个可用相机
    saved = None
    win = "d4_camera_select"
    cv2.namedWindow(win)

    while True:
        if current != saved:
            if cap is not None:
                cap.release()
            cap = CameraView(current)
            saved = current

        frame = cap.read()
        display = frame.copy()
        cv2.putText(display, f"index {current}  ({frame.shape[1]}x{frame.shape[0]})  "
                             f"是工作台相机就按 s 保存",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(win, display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            cfg_path = Path(args.config)
            cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
            cfg["hardware"]["camera_index"] = current
            cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"✅ 已保存：camera_index={current} → {cfg_path}")
            break
        if ord("0") <= key <= ord("9"):
            idx = key - ord("0")
            if idx in [c["index"] for c in cams]:
                current = idx
                print(f"切换到 index {idx}")

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
