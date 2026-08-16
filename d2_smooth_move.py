#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D2 Demo：手搓运动控制层 —— 多关节梯形速度规划平滑运动
=======================================================

流程（roadmap 阶段 B D2 验收）：
  1. 打开串口 → 顺序 Ping 扫描确认舵机
  2. 读各关节 Present_Position 与 Min/Max_Position_Limit
  3. 用运动控制层（梯形速度规划 / 线性插值 / 缓动）把多关节平滑运动到目标角度
  4. 记录轨迹 CSV，回读验证到位

用法：
  python d2_smooth_move.py --port COM22 --target "30,-20,15,-15,10,0"
  python d2_smooth_move.py --port COM22 --target "30,-20,15,-15,10,0" --profile linear --duration-ms 2000
  python d2_smooth_move.py --port COM22 --dry-run          # 只读状态不运动

说明：
  - 目标为"相对当前角度"（度），数量与舵机数一致（默认 6）
  - 目标自动截断到各关节限位；--max-step 兜底逐帧限幅（对照 LeRobot max_relative_target）
  - profile: trapezoid（梯形速度，vmax/amax 限制）/ linear（LeRobot 线性插值对照）/ ease（缓动）
  - --return：到达后自动返回起点（方便录制来回运动）
"""

import argparse
import sys
import time

from motion import ArmController
from protocol import DEFAULT_BAUDRATE, FeetechSerialBus, angle_to_counts, counts_to_angle

DEFAULT_SERVOS = [1, 2, 3, 4, 5, 6]
DEFAULT_TARGET = "30,-20,15,-15,10,0"  # 相对角度（度），靠限位兜底安全


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D2：手搓运动控制层，多关节平滑运动")
    p.add_argument("--port", required=True, help="舵机总线串口，如 COM22")
    p.add_argument("--servos", default="1,2,3,4,5,6", help="目标舵机 ID 列表（逗号分隔）")
    p.add_argument("--target", default=DEFAULT_TARGET, help="各关节相对转动角度（度，逗号分隔）")
    p.add_argument("--profile", choices=["trapezoid", "linear", "ease"], default="trapezoid")
    p.add_argument("--vmax", type=float, default=150.0, help="梯形规划最大速度（码/秒，默认 150）")
    p.add_argument("--amax", type=float, default=300.0, help="梯形规划最大加速度（码/秒²，默认 300）")
    p.add_argument("--fps", type=int, default=50, help="控制频率（默认 50）")
    p.add_argument("--max-step", type=float, default=None, help="每帧最大步长码值（None=由轨迹速度决定）")
    p.add_argument("--safety-margin", type=float, default=150.0, help="目标距离限位的最小安全余量（码，默认 150≈13°）")
    p.add_argument("--duration-ms", type=int, default=None, help="linear/ease 固定时长（毫秒）")
    p.add_argument("--return", action="store_true", help="到达后自动返回起点")
    p.add_argument("--log", default=None, help="轨迹 CSV 输出路径（默认自动生成）")
    p.add_argument("--dry-run", action="store_true", help="只读状态不运动")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    servo_ids = [int(s) for s in args.servos.split(",")]
    rel_angles = [float(a) for a in args.target.split(",")]
    if len(rel_angles) == 1:
        rel_angles *= len(servo_ids)
    if len(rel_angles) != len(servo_ids):
        print(f"!! 目标角度数({len(rel_angles)})与舵机数({len(servo_ids)})不一致")
        return 1

    print("=" * 64)
    print("D2 手搓运动控制层 · 多关节平滑运动")
    print(f"  端口   : {args.port} @ {DEFAULT_BAUDRATE}")
    print(f"  舵机   : {servo_ids}")
    print(f"  目标   : 相对 {rel_angles}°   profile={args.profile}")
    if args.profile == "trapezoid":
        print(f"  规划   : vmax={args.vmax} 码/s, amax={args.amax} 码/s²")
    else:
        dur = args.duration_ms or "自动估算"
        print(f"  规划   : {args.profile}（时长 {dur} ms）")
    print("=" * 64)

    bus = FeetechSerialBus(port=args.port, baudrate=DEFAULT_BAUDRATE)
    try:
        ctrl = ArmController(bus, servo_ids, fps=args.fps, max_step=args.max_step)

        # 1. 扫描 + 读状态
        models = bus.scan()
        if not models:
            print("!! 总线无应答：请检查 12V 电源与 BusLinker")
            return 1
        print(f"扫描到 {len(models)} 个舵机: {sorted(models)}")
        present = ctrl.read_positions()
        limits = ctrl.get_limits()
        print("\n当前状态（码值 / 角度，限位）：")
        for sid in servo_ids:
            lo, hi = limits[sid]
            print(f"  ID={sid}: {present[sid]:>5} ({counts_to_angle(present[sid]):>8.2f}°)  限位 [{lo}, {hi}]")

        if args.dry_run:
            print("\n[dry-run] 仅读取状态，未运动。")
            return 0

        # 2. 计算目标（相对角度 → 码值），并留出安全余量防过载（限位 ± safety_margin）
        margin = args.safety_margin
        targets = {}
        clamped_hit = []
        for i, sid in enumerate(servo_ids):
            lo, hi = limits[sid]
            q = present[sid] + angle_to_counts(rel_angles[i])
            q_c = min(max(q, lo + margin), hi - margin)
            if q_c != q:
                clamped_hit.append(sid)
            targets[sid] = q_c
        if clamped_hit:
            print(f"!! 以下关节目标被安全余量截断: {clamped_hit}")

        # 3. 运动
        log_path = args.log or f"d2_traj_{args.profile}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        print(f"\n开始运动（{args.profile}）... 轨迹日志: {log_path}")
        t0 = time.perf_counter()
        result = ctrl.move_to(
            targets,
            profile=args.profile,
            v_max=args.vmax,
            a_max=args.amax,
            duration_s=(args.duration_ms / 1000.0 if args.duration_ms else None),
            log_path=log_path,
        )
        elapsed = time.perf_counter() - t0

        # 4. 结果
        end = result["end"]
        print(f"\n运动完成：规划 {result['duration_s']:.2f}s，实际 {elapsed:.2f}s，{len(result['rows'])} 帧")
        for sid in servo_ids:
            err = end[sid] - int(round(targets[sid]))
            print(f"  ID={sid}: {present[sid]} → {end[sid]}（目标 {int(round(targets[sid]))}，误差 {err:+d} 码 ≈ {counts_to_angle(abs(err)):.2f}°）")
        max_err = max(abs(end[sid] - int(round(targets[sid]))) for sid in servo_ids)
        print(f"\n最大误差 {max_err} 码 ≈ {counts_to_angle(max_err):.2f}°（轨迹文件: {log_path}）")

        # 5. 可选返回起点（方便录制来回）
        if getattr(args, "return"):
            print("\n返回起点 ...")
            ctrl.move_to(
                {sid: float(present[sid]) for sid in servo_ids},
                profile=args.profile,
                v_max=args.vmax,
                a_max=args.amax,
                duration_s=(args.duration_ms / 1000.0 if args.duration_ms else None),
            )
            back = ctrl.read_positions()
            for sid in servo_ids:
                print(f"  ID={sid}: {back[sid]}（起点 {present[sid]}）")

        print("\nD2 运动完成 ✅（舵机保持力矩）")
        return 0
    except KeyboardInterrupt:
        print("\n已中断（Ctrl+C）。舵机保持当前位置。")
        return 130
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
