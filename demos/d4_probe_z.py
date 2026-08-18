#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 安全探高工具：实测"模型 z"与真实桌面的偏差，修正标定/抓取高度
=================================================================

背景：运动学模型（sign/offset/连杆长度）算出的绝对高度在重装机械臂或
翻转 sign 之后不可信——护栏只能挡"模型可见"的危险。因此**默认用手动探高**：
脚本不移动机械臂，你手动把臂放低到指尖刚接触桌面，脚本只读编码器算 z。

手动模式（默认，零撞桌风险）：
  1. 运行 → 机械臂失力（力矩关闭）
  2. 你手动把臂摆到探高点上方，缓慢放低到【指尖刚接触桌面】
  3. 按 Enter → 脚本恢复力矩（保持位姿）→ 读编码器 → FK 算模型 z
  4. 按 w 写入配置（z_safe_min/z_touch/z_grasp/z_lift 自动加安全余量）

自动模式（--auto，模型已可信时才用）：
  安全移动到探高点 → 每次 Enter 垂直下降 --step → c 标记 → w 写入。
  注意：模型不可信时自动模式可能移动到物理低位而撞桌。

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_probe_z.py                # 手动探高（推荐）
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_probe_z.py --auto          # 自动模式
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_probe_z.py --step 0.005    # 自动模式步长
"""

import argparse
import json
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d4_common import (  # noqa: E402
    D4_CONFIG,
    load_d4_config,
    make_bus,
    make_controller,
    make_kinematics,
    move_joints_deg_held,
    safe_move_xyz,
)
from my_arm_control.kinematics import KinematicsError  # noqa: E402


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：安全探高（实测桌面高度偏差，修正 z 配置）")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--x", type=float, default=0.07, help="探高点的基座 X（米）")
    p.add_argument("--y", type=float, default=0.10, help="探高点的基座 Y（米）")
    p.add_argument("--z-start", type=float, default=0.12, help="自动模式起始模型 z（米，高位安全）")
    p.add_argument("--step", type=float, default=0.005, help="自动模式每次下降步长（米，默认 0.5cm）")
    p.add_argument("--auto", action="store_true", help="自动模式（模型已可信才用；默认手动模式）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="写入的配置文件路径")
    return p.parse_args(argv)


def _read_tip(controller, kin) -> tuple[dict, tuple[float, float, float]]:
    """读编码器 → 各关节角 + 指尖模型 (x, y, z)。"""
    present = controller.read_positions()
    deg = {
        "shoulder_pan": kin.code_to_deg("shoulder_pan", present[1]),
        "shoulder_lift": kin.code_to_deg("shoulder_lift", present[2]),
        "elbow_flex": kin.code_to_deg("elbow_flex", present[3]),
        "wrist_flex": kin.code_to_deg("wrist_flex", present[4]),
    }
    xyz = kin.fk({**deg, "wrist_roll": 0.0, "gripper": 0.0})
    return deg, xyz


def _write_z(config_path: str, z_marked: float) -> bool:
    """把 z_marked 写入配置（z_safe_min=桌面+1cm 等安全余量）。

    z_grasp=桌面+2cm：夹爪指尖低于盒子顶部，才能包住盒子侧边；
    +3cm 会悬在盒子顶上"顶到物品"（D4 实测反馈）。
    """
    cfg_path = Path(config_path)
    cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    cfg["table"]["z_safe_min"] = round(z_marked + 0.01, 4)  # 防撞硬约束=桌面+1cm
    cfg["table"]["z_touch"] = round(z_marked + 0.01, 4)
    cfg["table"]["z_grasp"] = round(z_marked + 0.02, 4)
    cfg["table"]["z_lift"] = round(z_marked + 0.08, 4)
    cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  已写入 {cfg_path}: z_safe_min={cfg['table']['z_safe_min']} z_touch={cfg['table']['z_touch']} "
          f"z_grasp={cfg['table']['z_grasp']} z_lift={cfg['table']['z_lift']}")
    return True


def _print_advice(z_marked: float) -> None:
    print(f"\n  ★已标记：指尖接触桌面时模型 z={z_marked:.3f}")
    print(f"  → 建议：z_safe_min={z_marked + 0.01:.3f}（桌面+1cm 防撞下限）")
    print(f"          z_touch={z_marked + 0.01:.3f}（标定平面，指尖距桌面 1cm）")
    print(f"          z_grasp={z_marked + 0.03:.3f}  z_lift={z_marked + 0.08:.3f}")


def run_manual_probe(controller, kin, args: argparse.Namespace) -> int:
    """手动探高：脚本零自主运动，只读编码器算模型 z（任意模型误差下都安全）。"""
    print("\n【手动探高模式】脚本不会移动机械臂，零撞桌风险")
    print(f"  建议探高位置：基座 ({args.x:.3f}, {args.y:.3f}) 上方（仅提示，你手动摆就行）")
    z_marked = None
    try:
        controller.disable_torque()
        while True:
            input("\n  机械臂已失力。请【手动】把臂摆到探高点上方，缓慢放低到"
                  "【指尖刚接触桌面】\n  摆好后按 Enter 读取当前模型 z ... ")
            controller.enable_torque()  # 保持位姿（不会移动）
            time.sleep(0.5)
            deg, xyz = _read_tip(controller, kin)
            print(f"  读取到位姿：指尖模型 (x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}) m")
            print(f"            关节角 lift={deg['shoulder_lift']:.1f}° elbow={deg['elbow_flex']:.1f}° "
                  f"wrist={deg['wrist_flex']:.1f}°")
            ans = input("  [y]=确认指尖已接触桌面  [r]=重新摆位  [q]=退出  > ").strip().lower()
            if ans.startswith("y"):
                z_marked = xyz[2]
                break
            if ans.startswith("q"):
                return 0
            controller.disable_torque()  # r：再次失力重新摆位
        _print_advice(z_marked)
        ans = input("\n  按 [w] 写入配置 / [q] 退出  > ").strip().lower()
        if ans.startswith("w"):
            _write_z(args.config, z_marked)
        else:
            print("  未写入，配置未修改。")
    finally:
        controller.enable_torque()  # 结束保持力矩，避免手臂突然下垂
    return 0


def run_auto_probe(controller, kin, config, args: argparse.Namespace) -> int:
    """自动探高（模型已可信才用）：安全移动→逐步垂直下降→标记。"""
    z = args.z_start
    z_marked = None

    # 1) 安全移动到探高点（三段式抬升→高位水平→下降，路径级防撞护栏生效）
    print(f"[1] 安全移动到探高点 ({args.x:.3f}, {args.y:.3f}) @ z={z:.3f}（高位，安全）...")
    safe_move_xyz(controller, kin, args.x, args.y, z,
                  v_max=float(config["motion"]["grasp_v_max"]),
                  a_max=float(config["motion"]["grasp_a_max"]), settle_s=0.5)
    print("  已就位。")

    # 2) 探高下降：目标就是降到桌面找到"接触点"，需临时绕过 z_safe_min 护栏；
    #    但下降保持固定 XY 垂直小步、由你逐步确认，安全。
    kin.safe_z_min = None
    print("  开始逐步下降，观察夹爪与桌面的实际距离：")

    while True:
        print(f"\n  当前模型 z={z:.3f} m（模型认为指尖距基座底部 {z * 1000:.0f} mm）")
        if z_marked is not None:
            prompt = "  ★已标记 z=%.3f：按 [w] 写入配置 / [r] 取消标记继续降 / [q] 退出  > " % z_marked
        else:
            prompt = "  按 [Enter]=下降 %.0fmm  [c]=指尖已接近桌面  [q]=退出  > " % (args.step * 1000)
        cmd = input(prompt)
        cmd = (cmd.strip().lower() or "x")[0]  # 容错：取首个字符（cc/c 都算 c）
        if cmd == "q":
            break
        if cmd == "w" and z_marked is not None:
            _write_z(args.config, z_marked)
            break
        if cmd == "r":
            z_marked = None
            print("  已取消标记，可继续下降")
            continue
        if cmd == "c" and z_marked is None:
            z_marked = z
            _print_advice(z_marked)
            continue
        if z_marked is not None:
            # 已标记状态下 Enter 等其他输入：不下降，仅提示
            print("  已标记，请输入 w（写入）/ r（取消标记）/ q（退出）")
            continue
        # 默认（未标记）：下降一步
        z_next = z - args.step
        if z_next < -0.02:
            print("  已到模型保护下限（z<-0.02m），停止下降")
            break
        try:
            joints = kin.ik_vertical(args.x, args.y, z_next)
        except KinematicsError as e:
            print(f"  !! {e}（该高度不可达，停止）")
            break
        move_joints_deg_held(controller, kin, joints,
                             v_max=float(config["motion"]["grasp_v_max"]),
                             a_max=float(config["motion"]["grasp_a_max"]), settle_s=0.3)
        z = z_next

    if z_marked is None:
        print("\n未标记桌面位置，配置未修改。")
    return 0


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config(args.config)
    if args.port:
        config["hardware"]["port"] = args.port

    kin = make_kinematics(config)
    bus = make_bus(config)
    controller = make_controller(config, bus)
    print(f"已连接 {config['hardware']['port']}")
    print("提示：确认机械臂状态安全（无过载报警 0x20、无卡阻）再继续！")

    try:
        if args.auto:
            return run_auto_probe(controller, kin, config, args)
        return run_manual_probe(controller, kin, args)
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
