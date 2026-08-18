#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D4 校准：wrist_roll 腕部旋转参考系标定（方向 sign + 零位偏置 offset）
====================================================================

背景：长方形目标"垂直长边夹取"时，模型假设 wrist_roll=0 时夹爪两片连线
指向臂的径向方向（且顺时针为正）。但腕部舵机的物理零位/旋转方向从未验证，
导致夹爪角度偏几十度。本工具实测 wrist_roll 在几个已知角度下"夹爪两片连线"
的实际方向，拟合出：
  - kinematics.wrist_roll_sign   (±1)：旋转方向是否与模型假设一致
  - gripper.grip_offset_deg      (度) ：零位偏置

流程：
  1. 机械臂安全移动到标定位姿（默认 0.11, 0.0, 0.20，可 --x/--y/--z 调整到
     相机能看清竖直手爪爪尖的位置），夹爪张开
  2. 依次命令 wrist_roll = -60/-30/0/+30/+60（度）
  3. 每个角度：在画面里点夹爪两个爪尖（可滚轮缩放/右键拖动）→ Enter 确认
  4. 全部采集完 → 拟合 → 打印结果 → 按 w 写入配置

注意：必须保持【夹爪竖直朝下】（腕部旋转轴竖直）标定才有效；
不要把 4 号位（腕部）放平——那样旋转轴变水平，测出的角度无效。

用法：
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_calib_wrist.py
  D:\\miniconda\\envs\\lerobot\\python.exe demos\\d4_calib_wrist.py --x 0.14 --y 0.04 --z 0.22
"""

import argparse
import math
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path（demos 目录直接运行时可导入 my_arm_control）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from d4_calibrate_table import ZoomPanView, click_callback  # noqa: E402（复用缩放视图）
from d4_common import (  # noqa: E402
    D4_CONFIG,
    load_d4_config,
    load_table_calib,
    make_bus,
    make_camera,
    make_controller,
    make_kinematics,
    move_joints_deg_held,
    safe_move_xyz,
)
from my_arm_control.protocol import ADDR  # noqa: E402

# 采集的腕部角度（度）；每次换个角度，夹爪两片尖的连线会跟着转
WRIST_ANGLES = [-60.0, -30.0, 0.0, 30.0, 60.0]


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D4：wrist_roll 腕部方向参考系标定")
    p.add_argument("--port", type=str, default=None, help="舵机总线串口（默认取 d4_config.json）")
    p.add_argument("--x", type=float, default=0.11, help="机械臂定位 X（米，默认 0.11）")
    p.add_argument("--y", type=float, default=0.0, help="机械臂定位 Y（米，默认 0.0）")
    p.add_argument("--z", type=float, default=0.20, help="机械臂定位高度 z（米，默认 0.20，高点好观察）")
    p.add_argument("--config", type=str, default=str(D4_CONFIG), help="写入的配置文件路径")
    return p.parse_args(argv)


def click_two_points(cam, calib, psi: float) -> tuple[float, float] | None:
    """在画面中依次点击夹爪两片爪尖（左/右各一点），返回连线在【基座系】的方位角（度）。

    交互：左键点 2 个爪尖（画面实时显示 X/2），Enter/Space 确认，Esc 跳过。
    返回 None 表示跳过（Esc）。
    """
    ctx = {"view": None, "last_click": None, "new_click": False, "dragging": None}
    pts = []
    frame0 = cam.read_undistorted() if hasattr(cam, "read_undistorted") else cam.read()
    view = ZoomPanView(frame0.shape[1], frame0.shape[0])
    ctx["view"] = view
    cv2.namedWindow("wrist")  # 先创建窗口再绑定鼠标回调
    cv2.setMouseCallback("wrist", click_callback, ctx)
    print(f"\n=== wrist_roll={psi:+.0f}° ===")
    print("  依次点【两个爪尖】（各左键一次，可滚轮缩放/右键拖动）→ 点满后 Enter 确认 / Esc 跳过")
    while True:
        frame = cam.read_undistorted() if hasattr(cam, "read_undistorted") else cam.read()
        display = view.render(frame)
        cv2.putText(display, f"wrist_roll={psi:+.0f}deg  picked {len(pts)}/2 -> ENTER",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"zoom x{view.scale:.1f}  [wheel]zoom [R-drag]pan [L]pick",
                    (10, display.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
        if ctx["new_click"]:
            ctx["new_click"] = False
            pts.append(tuple(round(v) for v in ctx["last_click"]))
            print(f"  已选点 {len(pts)}/2  ({ctx['last_click'][0]}, {ctx['last_click'][1]})")
        for p in pts:
            wx, wy = view.orig_to_window(*p)
            cv2.circle(display, (int(wx), int(wy)), 6, (0, 0, 255), -1)
        if len(pts) >= 2:
            p0w = view.orig_to_window(*pts[0])
            p1w = view.orig_to_window(*pts[1])
            cv2.line(display, (int(p0w[0]), int(p0w[1])), (int(p1w[0]), int(p1w[1])), (0, 0, 255), 2)
        cv2.imshow("wrist", display)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10, 32) and len(pts) >= 2:  # ENTER / Space 确认
            cv2.destroyWindow("wrist")
            a0 = calib.pixel_to_base(pts[0][0], pts[0][1])
            a1 = calib.pixel_to_base(pts[1][0], pts[1][1])
            return math.degrees(math.atan2(a1[1] - a0[1], a1[0] - a0[0]))
        if key in (13, 10, 32) and len(pts) < 2:
            print(f"  （提示）还差 {2 - len(pts)} 个点，请再点一下夹爪爪尖")
        if key == 27:  # ESC 跳过
            cv2.destroyWindow("wrist")
            return None


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
    bus = make_bus(config)
    controller = make_controller(config, bus)
    cam = make_camera(config)
    print(f"已连接 {config['hardware']['port']}，相机就绪")

    try:
        controller.enable_torque()
        # 1) 安全移动到用户指定位置（夹爪竖直朝下、腕部旋转轴竖直，标定才有效）
        pan_deg = math.degrees(math.atan2(args.y - kin.y0, args.x - kin.x0))
        print(f"安全移动到位姿 ({args.x:.3f}, {args.y:.3f}, {args.z:.3f})，pan={pan_deg:+.1f}° ...")
        safe_move_xyz(controller, kin, args.x, args.y, args.z,
                      v_max=float(config["motion"]["grasp_v_max"]),
                      a_max=float(config["motion"]["grasp_a_max"]), settle_s=0.5)
        time.sleep(0.5)
        # 夹爪张开到最大（两片爪尖可见）
        c = kin.cal["gripper"]
        open_code = int(round(100.0 / 100.0 * (c["max"] - c["min"]) + c["min"]))
        bus.write_u16(6, ADDR["goal_position"][0], open_code)
        time.sleep(0.5)

        # 2) 依次采集
        data: list[tuple[float, float]] = []
        for psi in WRIST_ANGLES:
            move_joints_deg_held(controller, kin, {"wrist_roll": psi},
                                 v_max=float(config["motion"]["grasp_v_max"]),
                                 a_max=float(config["motion"]["grasp_a_max"]), settle_s=0.5)
            time.sleep(0.6)
            alpha = click_two_points(cam, calib, psi)
            if alpha is None:
                print("  跳过该角度")
                continue
            print(f"  夹爪连线基座方位角 α={alpha:+.1f}°")
            data.append((psi, alpha))
        cv2.destroyAllWindows()

        if len(data) < 3:
            print("!! 有效采集 <3 个，无法拟合")
            return 1

        # 3) 拟合 α = A + s·ψ（连线角 mod 180，先做平滑展开避免 ±180 跳变）
        psi0, alpha0 = data[0]
        alphas = [alpha0]
        for psi, a in data[1:]:
            while a - alphas[-1] > 90:
                a -= 180
            while a - alphas[-1] < -90:
                a += 180
            alphas.append(a)
        ps = np.array([d[0] for d in data], dtype=float)
        als = np.array(alphas, dtype=float)
        m, b = np.polyfit(ps, als, 1)  # α = m·ψ + b；m=斜率（旋转方向），b=截距
        sgn = 1.0 if m >= 0 else -1.0
        # 物理模型 α = pan + α0 + s·ψ → 截距 b = pan + α0 → α0 = b - pan
        # 代码用 ψ = sign*(θ_grip-pan) + grip_off
        alpha0_phys = b - pan_deg
        if sgn > 0:
            grip_off = -alpha0_phys
        else:
            grip_off = alpha0_phys
        print("\n" + "=" * 60)
        print(f"拟合：α = {b:+.1f} + {m:+.3f}·ψ（连线角按 ±180 折叠展开后拟合，pan={pan_deg:+.1f}°）")
        print(f"旋转方向 s = {sgn:+.0f} → kinematics.wrist_roll_sign = {sgn:+.0f}"
              f"{'（与模型一致）' if sgn > 0 else '（与模型相反，代码将翻转）'}")
        print(f"零位偏置 α0 = {alpha0_phys:+.1f}° → gripper.grip_offset_deg = {grip_off:+.1f}")
        ans = input("\n  按 [w] 写入配置 / [q] 退出  > ").strip().lower()
        if ans.startswith("w"):
            cfg_path = Path(args.config)
            cfg = json_load(cfg_path)
            cfg["kinematics"]["wrist_roll_sign"] = sgn
            cfg["gripper"]["grip_offset_deg"] = round(grip_off, 1)
            cfg_path.write_text(json_dump(cfg), encoding="utf-8")
            print(f"  已写入 {cfg_path}: wrist_roll_sign={sgn:+.0f} grip_offset_deg={grip_off:+.1f}")
        else:
            print("  未写入，配置未修改。")
    finally:
        bus.close()
    return 0


def json_load(p: Path) -> dict:
    import json
    return json.loads(p.read_text(encoding="utf-8"))


def json_dump(cfg: dict) -> str:
    import json
    return json.dumps(cfg, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    sys.exit(main())
