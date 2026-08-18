#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 校准：夹爪零位偏置 grip_offset_deg 实测标定（校准计划第 7 项）
=================================================================

背景：相机在机械臂正上方，竖直手爪的爪尖从相机拍不全，点爪尖法不可行。
改用【实物对齐法】：夹爪接近盒子到观察高度，你用 a/d 键微调腕部角度，
直到从侧面看【夹爪两片连线垂直于盒子长边】，工具按你调的量更新 grip_offset_deg。
不需要估角度，看着调就行。

流程：
  1. 蓝盒子放桌上（任意位置/朝向），本工具检测盒子 → 接近到 z_pre（不夹取）
  2. 按 a / d 微调 wrist_roll（每步 2°），直到夹爪两片连线 ⊥ 盒子长边
  3. 按 Enter 完成 → 工具算 Δ → 按 w 写入 grip_offset_deg
  4. 【验证】把盒子换个位置/朝向再跑一次：
       - 基本不用再调 → 完成
       - 又需要大幅调整 → 腕部方向相反，需翻转 kinematics.wrist_roll_sign

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_calib_grip_offset.py
"""

import argparse
import math
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from d4_common import (  # noqa: E402
    D4_CONFIG,
    load_d4_config,
    load_table_calib,
    make_bus,
    make_camera,
    make_controller,
    make_detector,
    make_kinematics,
    move_joints_deg_held,
    safe_move_xyz,
)


from my_arm_control.protocol import ADDR  # noqa: E402


def _get_key() -> str:
    """Windows 单键读取（无回车）：a/d=微调，Enter=完成，q/Esc=退出。

    非 Windows 环境回退到 input()。
    """
    try:
        import msvcrt  # noqa: PLC0415

        if not msvcrt.kbhit():
            return ""
        ch = msvcrt.getch().lower()
        if ch in (b"a",):
            return "a"
        if ch in (b"d",):
            return "d"
        if ch in (b"\r", b"\n"):
            return "enter"
        if ch in (b"q", b"\x1b"):
            return "q"
        return ""
    except ImportError:
        return input("  [a/d]微调 [Enter]完成 [q]退出 > ").strip().lower()


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：夹爪零位偏置 grip_offset_deg 实测标定")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="写入的配置文件路径")
    p.add_argument("--step", type=float, default=2.0, help="a/d 单步微调角度（默认 2°）")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_d4_config(args.config)
    if args.port:
        config["hardware"]["port"] = args.port

    calib = load_table_calib()
    if calib is None:
        print("!! 没有手眼标定 d4_table_calib.json，先跑 d4_calibrate_table.py")
        return 1

    kin = make_kinematics(config)
    det = make_detector(config)
    bus = make_bus(config)
    controller = make_controller(config, bus)
    cam = make_camera(config)
    print(f"已连接 {config['hardware']['port']}，相机就绪")

    t = config["table"]
    z_pre = float(t["z_pre"])
    wrist_sign = float(config["kinematics"].get("wrist_roll_sign", 1.0))
    grip_off = float(config["gripper"].get("grip_offset_deg", 0.0))
    v_max = float(config["motion"]["grasp_v_max"])
    a_max = float(config["motion"]["grasp_a_max"])

    try:
        # 1) 检测盒子（重试几次）
        target = None
        for i in range(5):
            frame = cam.read_undistorted() if hasattr(cam, "read_undistorted") else cam.read()
            target = det.detect_one(frame)
            if target is not None:
                break
            print(f"  未检测到盒子，重试 {i + 1}/5 ...")
            time.sleep(1.0)
        if target is None:
            print("!! 一直没检测到蓝盒子，请把盒子放进相机视野再跑")
            return 1
        if target.angle is None:
            print("!! 盒子太小（面积 <400px），检测不到长边方向，请把盒子放近一点再跑")
            return 1
        print(f"检测到盒子：中心像素 ({target.x}, {target.y})，面积 {target.area:.0f}")

        # 2) 像素 → 夹爪中心目标基座 XY（与 grasp.pixel_to_center 同公式）。
        #    自洽性：标定记录"爪尖像素↔爪尖基座 G+u"；pixel_to_center=H-u，
        #    把它当爪尖目标移动后，中心 = (H-u)+u = H ≈ 盒子基座（半宽抵消，勿改）。
        bx, by = calib.pixel_to_base(target.x, target.y)
        g = config["gripper"]
        half, side = float(g["half_span"]), float(g["jaw_side"])
        pan0 = math.atan2(by - kin.y0, bx - kin.x0)
        bx += side * half * math.sin(pan0)
        by -= side * half * math.cos(pan0)

        # 3) 目标长边（基座系）→ 正确夹爪方向 → 当前命令腕角
        theta_img = math.radians(target.angle)
        bx0, by0 = calib.pixel_to_base(target.x, target.y)
        bx1, by1 = calib.pixel_to_base(target.x + 60.0 * math.cos(theta_img),
                                       target.y + 60.0 * math.sin(theta_img))
        theta_long = math.atan2(by1 - by0, bx1 - bx0)
        theta_grip = theta_long + math.pi / 2
        pan = math.atan2(by - kin.y0, bx - kin.x0)
        psi_cmd = wrist_sign * math.degrees(theta_grip - pan) + grip_off
        psi_cmd = (psi_cmd + 180.0) % 360.0 - 180.0
        print(f"盒子基座位 ({bx:.3f}, {by:.3f})  长边基座角 {math.degrees(theta_long):+.1f}°")
        print(f"正确夹爪方向 = 长边+90° = {math.degrees(theta_grip):+.1f}°（基座系）")
        print(f"当前命令腕角 wrist_roll = {psi_cmd:+.1f}°（grip_offset_deg={grip_off:+.1f}）")

        # 4) 接近到 z_pre（夹爪竖直朝下、带当前腕角），夹爪张开
        print(f"接近到盒子正上方 z={z_pre:.3f} ...")
        safe_move_xyz(controller, kin, bx, by, z_pre, v_max=v_max, a_max=a_max, settle_s=0.5)
        move_joints_deg_held(controller, kin, {"wrist_roll": psi_cmd}, v_max=v_max, a_max=a_max, settle_s=0.4)
        c = kin.cal["gripper"]
        open_code = int(round(100.0 / 100.0 * (c["max"] - c["min"]) + c["min"]))
        for _ in range(3):  # 串口偶发超时重试
            try:
                bus.write_u16(6, ADDR["goal_position"][0], open_code)
                break
            except Exception:  # noqa: BLE001
                time.sleep(0.1)
        time.sleep(0.6)

        # 5) 手动微调直到对齐（a/d 单键，无回车）
        print("\n" + "=" * 60)
        print("  现在从侧面看夹爪与盒子（夹爪在盒子上方 3~4cm）：")
        print(f"  按 [a] 逆时针 / [d] 顺时针 微调腕部（每步 {args.step:g}°，按住连按）")
        print("  直到【夹爪两片连线垂直于盒子长边】→ 按 Enter 完成 / q 退出")
        psi = psi_cmd  # 累计（不 wrap），避免跨 ±180 时 Δ 算错；下发时再 wrap
        last_disp = None
        while True:
            key = _get_key()
            if key == "a":
                psi -= args.step
                move_joints_deg_held(controller, kin, {"wrist_roll": (psi + 180.0) % 360.0 - 180.0},
                                     v_max=v_max, a_max=a_max, settle_s=0.4)
            elif key == "d":
                psi += args.step
                move_joints_deg_held(controller, kin, {"wrist_roll": (psi + 180.0) % 360.0 - 180.0},
                                     v_max=v_max, a_max=a_max, settle_s=0.4)
            elif key == "enter":
                break
            elif key == "q":
                print("\n  已退出，未写入。")
                return 0
            if key in ("a", "d") or (last_disp != psi):
                cmd = (psi + 180.0) % 360.0 - 180.0
                print(f"\r  当前 wrist_roll = {cmd:+.1f}°（相对命令 {(psi - psi_cmd):+.1f}°）   ",
                      end="", flush=True)
                last_disp = psi
            time.sleep(0.02)

        # 6) 计算修正量并写入
        delta = psi - psi_cmd
        grip_new = grip_off + delta  # 对齐时 α0 = -grip_off - Δ → 新 grip_off = grip_off + Δ
        print("\n" + "=" * 60)
        print(f"你调了 Δ = {delta:+.1f}°（相对命令腕角）")
        print(f"grip_offset_deg: {grip_off:+.1f} → {grip_new:+.1f}")
        ans = input("  按 [w] 写入配置 / [q] 退出  > ").strip().lower()
        if ans.startswith("w"):
            cfg_path = Path(args.config)
            import json
            cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
            cfg["gripper"]["grip_offset_deg"] = round(grip_new, 1)
            cfg_path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")
            print(f"  已写入 {cfg_path}: grip_offset_deg={grip_new:+.1f}")
            print("  【验证】把盒子换个位置/朝向再跑一次：如果基本不用再调 → 完成；")
            print("  如果又要大幅调整 → 腕部方向相反，需要翻转 kinematics.wrist_roll_sign（告诉我）")
        else:
            print("  未写入，配置未修改。")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
