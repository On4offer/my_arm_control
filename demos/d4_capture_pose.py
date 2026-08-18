#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 安全位姿捕获：手动摆放 → 记录关节角度 → 写入 d4_config.json safe_pose
=====================================================================

用途：自定义"回安全位"位姿（默认是量程中点=夹爪朝前，可能挡住相机目标）。
把你想要的安全位姿（如臂竖直收拢、不挡目标）手动摆出来并记录。

操作：
  1. 运行本工具 → 提示失能力矩（大臂会下垂，务必扶住！）
  2. 手动把机械臂摆到期望的安全位姿（例如：臂竖直、夹爪朝上/收拢，不遮挡工作区）
  3. 扶着保持不动，按 Enter → 程序读取各关节角度 → 重新使能 → 写入配置
  4. 之后所有"回安全位"都用这个位姿

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_capture_pose.py
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_capture_pose.py --port COM24
"""

import argparse
import json
import sys
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d4_common import (  # noqa: E402
    D4_CONFIG,
    load_d4_config,
    make_bus,
    make_controller,
    make_kinematics,
)
from my_arm_control.protocol import ADDR  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：安全位姿捕获（手动摆放并记录）")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="配置文件路径")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config(args.config)
    if args.port:
        config["hardware"]["port"] = args.port

    kin = make_kinematics(config)
    bus = make_bus(config)
    controller = make_controller(config, bus)
    print(f"已连接 {config['hardware']['port']}")

    try:
        print("\n即将【失能力矩】——大臂（ID2）会因重力下垂，请立刻扶住机械臂！")
        input("准备好后按 Enter 失能 ...")
        for sid in controller.servo_ids:
            bus.write_u8(sid, ADDR["torque_enable"][0], 0)
        print("已失能力矩。请手动把机械臂摆到【期望的安全位姿】（例如：臂竖直收拢、不遮挡工作区）。")

        input("摆好后【保持扶着】按 Enter 读取位姿 ...")

        # 读取各关节角度（失能时编码器仍工作）
        pose = {}
        present = controller.read_positions()
        names = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex",
                 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
        for sid, name in names.items():
            pose[name] = round(kin.code_to_deg(name, present[sid]), 1)
        print("\n读取到位姿：")
        for name, deg in pose.items():
            print(f"  {name:15s} = {deg:+.1f}°")

        # 重新使能（保持当前位置）
        controller.enable_torque()
        print("\n已重新使能力矩（保持当前位置）。")

        cfg_path = Path(args.config)
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
        cfg["safe_pose"] = pose
        cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"已写入 {cfg_path}: safe_pose")
        print("之后所有【回安全位】将使用该位姿。")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
