#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 校准步骤 1：验证关节方向（sign）
====================================

逐个让关节做 +delta（码值增大）运动，你观察机械臂往哪个物理方向动，
与模型假设（sign=+1）对比，确认 d4_config.json 的 kinematics.sign 是否需要改。

运行前：
  - 机械臂安全（无 0x20）、大臂竖直、周围无阻挡
  - 每个关节运动幅度很小（默认 120 码 ≈ 10°），安全

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_calib_sign.py
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_calib_sign.py --delta 150
"""

import argparse
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d4_common import load_d4_config, make_bus, make_controller  # noqa: E402
from my_arm_control.protocol import ADDR, ProtocolError  # noqa: E402

# (关节ID, 名称, {sign: 该 sign 下"码值增大"的物理方向描述})
# 说明：方向描述按【模型数学】给出（φ=s·θ+o，零位 θ=0 时 φ=offset）——
#   shoulder_lift: φ2 增大 → z=L1·cosφ2 减小 → 夹爪端下降
#   elbow_flex:    φ3 增大 → 前臂折叠 → 夹爪端下降
#   wrist_flex:    零位 φ_tip=92.8°(<180°)，φ4 增大 → 夹爪端下垂
# 验证前请把机械臂摆到【大臂竖直、夹爪朝下】的常规位姿，方向判断才可靠。
JOINTS = [
    (1, "shoulder_pan", {1.0: "整臂绕底座旋转（俯视：夹爪端向左侧转）", -1.0: "整臂绕底座旋转（俯视：夹爪端向右侧转）"}),
    (2, "shoulder_lift", {1.0: "大臂前倾、夹爪端下降", -1.0: "大臂后仰、夹爪端抬起"}),
    (3, "elbow_flex", {1.0: "前臂向前折叠（夹爪端下降）", -1.0: "前臂向后展开（夹爪端抬起）"}),
    (4, "wrist_flex", {1.0: "手腕垂下（夹爪端下垂）", -1.0: "手腕抬起（夹爪端上仰）"}),
]


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：关节方向验证（sign 校准）")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--delta", type=int, default=170, help="单步码值（默认 170 ≈ 15°）")
    p.add_argument("--steps", type=int, default=12, help="分步数（默认 12 步）")
    p.add_argument("--step-delay", type=float, default=0.4, help="每步间隔秒（默认 0.4s，越大越慢）")
    return p.parse_args(argv)


def move_slowly(bus, servo_id: int, start: int, goal: int, steps: int, step_delay: float) -> None:
    """分步慢速运动：每步只移动一小段并等待，总时长 ≈ steps*step_delay 秒（不依赖 Goal_Time）。"""
    for i in range(1, steps + 1):
        target = int(round(start + (goal - start) * i / steps))
        for _ in range(3):
            try:
                bus.write_u16(servo_id, ADDR["goal_position"][0], target)
                break
            except ProtocolError:
                time.sleep(0.05)
        time.sleep(step_delay)


def read_pos(bus, servo_id: int) -> int:
    for _ in range(3):
        try:
            return bus.read_u16(servo_id, ADDR["present_position"][0])
        except ProtocolError:
            time.sleep(0.05)
    raise RuntimeError(f"读位置失败 servo={servo_id}")


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config()
    if args.port:
        config["hardware"]["port"] = args.port

    bus = make_bus(config)
    controller = make_controller(config, bus)
    print(f"已连接 {config['hardware']['port']}")
    print(f"提示：确认机械臂安全、周围无阻挡。每个关节分 {args.steps} 步慢速运动约 {args.delta // 11}°"
          f"（总约 {args.steps * args.step_delay:.0f} 秒）。")
    input("准备好后按 Enter 开始 ...")

    results = []
    signs = config["kinematics"]["sign"]  # 读取当前配置的 sign
    try:
        controller.enable_torque()
        for sid, name, dir_map in JOINTS:
            cur_sign = float(signs[name])
            assume = dir_map[cur_sign]  # 当前 sign 对应的假设方向
            start = read_pos(bus, sid)
            goal = start + args.delta  # 码值增大方向
            print(f"\n=== {name} (ID={sid}) ===")
            print(f"  将向【码值增大】方向运动 {args.delta} 码（当前 {start} → 目标 {goal}）")
            move_slowly(bus, sid, start, goal, args.steps, args.step_delay)
            time.sleep(1.0)  # 停留观察
            end = read_pos(bus, sid)
            print(f"  已运动，到位码值: {end}（位移 {end - start:+d} 码）")
            print(f"  当前配置 sign={cur_sign:+} → 码值增大应表现为：{assume}")
            print(f"  请观察实际运动方向是否与此一致")
            ans = input("  [y]=实际方向一致  [n]=实际方向相反（需翻转 sign）  [s]=跳过  > ").strip().lower()
            if ans == "y":
                results.append((name, "一致", f"sign 保持 {cur_sign:+}"))
            elif ans == "n":
                results.append((name, "相反", f"sign 改为 {-cur_sign:+}"))
            else:
                results.append((name, "跳过", "待人工确认"))
            # 回原位（同样慢速）
            move_slowly(bus, sid, end, start, args.steps, args.step_delay)
            time.sleep(0.5)

        print("\n" + "=" * 60)
        print("方向验证汇总：")
        changed = False
        for name, obs, action in results:
            print(f"  {name:15s} {obs:6s} → {action}")
            if "改为" in action:
                changed = True
        if changed:
            print("\n!! 有关节方向与假设相反 → 需要修改 d4_config.json 的 kinematics.sign")
            print("   （把对应关节 sign 从 1.0 改为 -1.0）")
        else:
            print("\n✅ 全部关节方向与模型假设一致（sign 无需修改）")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
