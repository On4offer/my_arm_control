#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 Demo：视觉抓取闭环（定位 → 接近 → 下降 → 夹取 → 抬起 → 校验 → 回位）
==========================================================================

前提：
  1. 已跑通 d4_calibrate_table.py（存在 config/d4_table_calib.json）
  2. 已用 d4_detect_demo.py 调好目标 HSV（写入 d4_config.json target_detect.hsv）
  3. 目标物体颜色单一、大于夹爪张开宽度、摆放在标定网格覆盖的工作区内

用法：
  python d4_grasp_demo.py --trials 5          # 连跑 5 轮（默认）
  python d4_grasp_demo.py --trials 50         # 验收：随机摆放 50 次，统计成功率
  python d4_grasp_demo.py --no-random         # 每轮不移动目标（人工摆放）
  python d4_grasp_demo.py --port COM24        # 指定端口
  python d4_grasp_demo.py --estop             # 中断时失能力矩（急停）

每轮流程：回安全位 → 拍照检测 → 像素→基座XY → 接近(z_pre)→下降(z_grasp)→
夹爪闭合→抬起(z_lift)→校验夹爪到位反馈（关到底=空抓→重试）→ 回安全位。

Ctrl+C：回安全位后退出（保持力矩）；--estop 额外失能力矩。
"""

import argparse
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d4_common import (  # noqa: E402
    load_d4_config,
    load_table_calib,
    make_bus,
    make_camera,
    make_controller,
    make_detector,
    make_kinematics,
)
from my_arm_control.grasp import GraspController  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：视觉抓取闭环")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--camera", type=int, default=None, help="相机索引（默认取 d4_config.json，不可用自动选择）")
    p.add_argument("--trials", type=int, default=None, help="抓取轮数（默认取 d4_config.json）")
    p.add_argument("--no-random", action="store_true", help="不随机摆放目标（人工摆放）")
    p.add_argument("--estop", action="store_true", help="中断时失能力矩（急停）")
    p.add_argument("--no-camera", action="store_true", help="无相机模式（用提示框确认目标）")
    p.add_argument("--preview", action="store_true",
                   help="每轮检测前显示相机画面（带检测框），确认目标可见后按任意键继续")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config()
    if args.port:
        config["hardware"]["port"] = args.port
    if args.trials:
        config["grasp_loop"]["n_trials"] = args.trials
    if args.no_random:
        config["grasp_loop"]["random_place"] = False

    calib = load_table_calib()
    if calib is None:
        print("!! 未找到标定文件，请先运行：python d4_calibrate_table.py")
        return 1

    kin = make_kinematics(config)
    detector = make_detector(config)
    bus = make_bus(config)
    controller = make_controller(config, bus)
    grasp = GraspController(config, kin, calib, detector, bus, controller)

    cam = None
    if not args.no_camera:
        cam = make_camera(config, camera_index=args.camera)
        grasp._camera_read = cam.read_undistorted

    grasp.install_sigint_handler(estop=args.estop)

    print("=" * 60)
    print("D4 视觉抓取闭环")
    print(f"  端口   : {config['hardware']['port']}")
    print(f"  轮数   : {grasp.n_trials}   随机摆放: {config['grasp_loop']['random_place']}")
    print(f"  标定   : {config['target_detect']['hsv']}")
    print("=" * 60)

    try:
        if args.no_camera:
            # 无相机模式：检测由用户确认（开发/演示用）
            for i in range(grasp.n_trials):
                input(f"[{i + 1}/{grasp.n_trials}] 摆放目标后按 Enter（跳过输入 x 自动拍照）")
                frame = None
                grasp.grasp_once(frame, log=print)
                grasp.safe_home()
        else:
            preview_fn = None
            if args.preview:
                import cv2  # noqa: PLC0415

                def preview_fn(i, n):  # noqa: ANN001
                    print(f"[预览] 第 {i}/{n} 轮：实时画面，确认目标可见后按任意键继续 / q 退出")
                    while True:
                        frame = cam.read_undistorted()
                        display = detector.draw(frame, detector.detect(frame))
                        cv2.putText(display, f"round {i}/{n}: 确认目标可见（绿框）→ 任意键继续 / q 退出",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.imshow("d4_preview", display)
                        key = cv2.waitKey(30) & 0xFF
                        if key == ord("q"):
                            cv2.destroyWindow("d4_preview")
                            sys.exit(0)
                        if key != 255:  # 任意键继续
                            break
                    cv2.destroyWindow("d4_preview")

            result = grasp.run_trials(log=print, preview=preview_fn)
            print("\n成功率为验收指标：roadmap 阶段 C 要求 ≥80%（50 次）")
            print("如需 50 次验收：python d4_grasp_demo.py --trials 50")
    finally:
        bus.close()
        if cam is not None:
            cam.cap.release()

    return 0


if __name__ == "__main__":
    sys.exit(main())
