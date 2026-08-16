#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D2 波浪运动 Demo：多关节交替正反向摆动（涟漪效果），用于录视频
================================================================

以启动时的当前姿态为中心，各关节按"正、反、正、反"交替相位摆动
（相邻关节反向 → 视觉上像波浪），可循环多次、每次到达后短暂停顿。

安全设计：
  - 每关节幅度 = min(请求幅度, 该关节距限位±安全余量的最大可摆幅度)
  - 目标始终在 [Min+margin, Max-margin] 内，防止近限位过载
  - 结束回到中心姿态

用法：
  python d2_wave_demo.py --port COM22
  python d2_wave_demo.py --port COM22 --center mid --amplitude 15 --loops 2 --pause-ms 800
  python d2_wave_demo.py --port COM22 --dry-run        # 只读状态
"""

import argparse
import sys
import time

from motion import ArmController
from protocol import DEFAULT_BAUDRATE, FeetechSerialBus, angle_to_counts, counts_to_angle

DEFAULT_SERVOS = [1, 2, 3, 4, 5, 6]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D2：多关节波浪运动（录视频用）")
    p.add_argument("--port", required=True, help="舵机总线串口，如 COM22")
    p.add_argument("--servos", default="1,2,3,4,5,6", help="目标舵机 ID 列表（逗号分隔）")
    p.add_argument("--center", choices=["current", "mid"], default="current",
                   help="波浪中心：current=当前姿态（默认）/ mid=先平滑移到各关节量程中点（6 关节都能摆）")
    p.add_argument("--amplitude", type=float, default=12.0, help="各关节摆动幅度（度，默认 12）")
    p.add_argument("--loops", type=int, default=3, help="正反向摆动循环次数（默认 3）")
    p.add_argument("--pause-ms", type=int, default=800, help="每次到位后停顿（毫秒，默认 800）")
    p.add_argument("--vmax", type=float, default=150.0, help="梯形规划最大速度（码/秒）")
    p.add_argument("--amax", type=float, default=300.0, help="梯形规划最大加速度（码/秒²）")
    p.add_argument("--fps", type=int, default=50, help="控制频率（默认 50）")
    p.add_argument("--safety-margin", type=float, default=150.0, help="目标距限位最小安全余量（码）")
    p.add_argument("--log", default=None, help="轨迹 CSV 前缀（默认自动生成）")
    p.add_argument("--dry-run", action="store_true", help="只读状态不运动")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    servo_ids = [int(s) for s in args.servos.split(",")]

    print("=" * 64)
    print("D2 波浪运动 · 多关节交替摆动（录视频用）")
    print(f"  端口    : {args.port} @ {DEFAULT_BAUDRATE}")
    print(f"  舵机    : {servo_ids}")
    print(f"  幅度    : {args.amplitude}°  循环 {args.loops} 次  停顿 {args.pause_ms}ms")
    print("=" * 64)

    bus = FeetechSerialBus(port=args.port, baudrate=DEFAULT_BAUDRATE)
    try:
        ctrl = ArmController(bus, servo_ids, fps=args.fps)

        # 1. 扫描 + 读中心姿态与限位
        models = bus.scan()
        if not models:
            print("!! 总线无应答：请检查 12V 电源与 BusLinker")
            return 1
        center = ctrl.read_positions()
        limits = ctrl.get_limits()
        print(f"扫描到 {len(models)} 个舵机: {sorted(models)}")
        print("\n当前姿态（码值 / 角度，限位）：")
        for sid in servo_ids:
            lo, hi = limits[sid]
            print(f"  ID={sid}: {center[sid]:>5} ({counts_to_angle(center[sid]):>8.2f}°)  限位 [{lo}, {hi}]")

        if args.dry_run:
            print("\n[dry-run] 仅读取状态，未运动。")
            return 0

        # 2. 可选：先平滑移到各关节量程中点，作为波浪中心（6 关节都能摆动）
        if args.center == "mid":
            print("\n平滑移动到量程中点（作为波浪中心）...")
            mid_targets = {sid: float((limits[sid][0] + limits[sid][1]) / 2) for sid in servo_ids}
            ctrl.move_to(mid_targets, profile="trapezoid", v_max=args.vmax, a_max=args.amax)
            center = ctrl.read_positions()
            for sid in servo_ids:
                print(f"  ID={sid}: 中心 {center[sid]}")

        # 3. 计算每关节实际可摆幅度（受限位±安全余量约束）
        margin = args.safety_margin
        amp_raw = angle_to_counts(args.amplitude)
        amps = {}
        for sid in servo_ids:
            lo, hi = limits[sid]
            max_amp = min(center[sid] - (lo + margin), (hi - margin) - center[sid])
            amps[sid] = max(0.0, min(amp_raw, max_amp))  # 每关节幅度（码）
        reduced = [sid for sid in servo_ids if amps[sid] < amp_raw - 1e-6]
        if reduced:
            print(f"\n!! 以下关节幅度受限位余量截断: {reduced}")
        for sid in servo_ids:
            print(f"  实际幅度 ID={sid}: {amps[sid]:.0f} 码 ≈ {counts_to_angle(amps[sid]):.1f}°")
        if all(amps[sid] < 10 for sid in servo_ids):
            print("!! 所有关节可摆幅度过小，请调整 --amplitude 或 --safety-margin")
            return 1

        # 4. 正反向姿态（相邻关节交替相位 → 波浪效果）
        def pose(sign: float) -> dict[int, float]:
            return {sid: float(center[sid] + sign * amps[sid] * (-1) ** i) for i, sid in enumerate(servo_ids)}

        fwd, bwd = pose(+1.0), pose(-1.0)

        # 5. 波浪循环
        log_prefix = args.log or f"d2_wave_{time.strftime('%Y%m%d_%H%M%S')}"
        print(f"\n开始波浪运动（{args.loops} 次循环）...")
        for loop in range(1, args.loops + 1):
            for name, targets in (("正摆", fwd), ("反摆", bwd)):
                print(f"  [loop {loop}/{args.loops}] {name} ...")
                ctrl.move_to(
                    targets,
                    profile="trapezoid",
                    v_max=args.vmax,
                    a_max=args.amax,
                    log_path=f"{log_prefix}_l{loop}_{name}.csv",
                )
                if args.pause_ms > 0:
                    time.sleep(args.pause_ms / 1000.0)

        # 6. 回到中心
        print("\n回到中心姿态 ...")
        ctrl.move_to({sid: float(center[sid]) for sid in servo_ids}, profile="trapezoid",
                     v_max=args.vmax, a_max=args.amax)
        final = ctrl.read_positions()
        for sid in servo_ids:
            print(f"  ID={sid}: {final[sid]}（中心 {center[sid]}）")

        print("\nD2 波浪运动完成 ✅（舵机保持力矩）")
        return 0
    except KeyboardInterrupt:
        print("\n已中断（Ctrl+C）。舵机保持当前位置。")
        return 130
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
