#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D1 Demo：手搓串口协议点亮 SO-ARM101 舵机
==========================================

流程（roadmap 阶段 B D1 验收）：
  1. 打开串口（1M 波特率，8N1）
  2. 广播 Ping 搜索总线 → 识别全部舵机 ID 与型号号
  3. 让指定舵机从当前位置平滑转动指定角度（默认 1 号舵机转 30°）
  4. 回读 Present_Position 验证到位

用法：
  python d1_scan_and_move.py --port COM3
  python d1_scan_and_move.py --port COM3 --servo 2 --angle -45 --duration-ms 2000

说明：
  - 全程只依赖 pyserial，不调用 LeRobot / scservo_sdk，协议由 protocol.py 手搓实现
  - 运动前自动把目标位置截断到 Min/Max_Position_Limit 内，防止越限
  - 结束后保留力矩（舵机保持位置）；按 Ctrl+C 中断安全
"""

import argparse
import sys
import time
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.protocol import (  # noqa: E402
    ADDR,
    DEFAULT_BAUDRATE,
    FeetechSerialBus,
    MODEL_RESOLUTION_STS3215,
    ProtocolError,
    angle_to_counts,
    counts_to_angle,
    decode_sign_magnitude,
    encode_sign_magnitude,
)

MODEL_STS3215 = 777  # 幻尔 HX 舵机兼容 Feetech STS3215


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="D1：手搓串口协议点亮 SO-ARM101 舵机")
    p.add_argument("--port", required=True, help="舵机总线串口，如 COM3")
    p.add_argument("--servo", type=int, default=1, help="目标舵机 ID（默认 1）")
    p.add_argument("--angle", type=float, default=30.0, help="转动角度（度），负值为反向（默认 30）")
    p.add_argument("--duration-ms", type=int, default=1500, help="运动耗时（毫秒），越大越平滑（默认 1500）")
    p.add_argument("--tolerance", type=int, default=5, help="到位判定容差（码值，默认 5）")
    p.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE, help="波特率（默认 1000000）")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    print("=" * 60)
    print("D1 手搓串口协议 · SO-ARM101 舵机点亮")
    print(f"  端口    : {args.port} @ {args.baudrate}")
    print(f"  舵机    : ID={args.servo}  目标转动 {args.angle}°")
    print("=" * 60)

    bus = FeetechSerialBus(port=args.port, baudrate=args.baudrate)
    try:
        # ---- 1. 搜索总线 ----
        print("\n[1/4] 顺序 Ping 搜索总线 ...")
        models = bus.scan()
        if not models:
            print("!! 总线无应答：请检查 12V 电源已开、BusLinker 已插好、端口是否正确")
            return 1
        for sid, model in sorted(models.items()):
            tag = " (STS3215 兼容)" if model == MODEL_STS3215 else ""
            print(f"     舵机 ID={sid}  型号号={model}{tag}")

        # ---- 2. 校验目标舵机 ----
        if args.servo not in models:
            print(f"!! 未找到目标舵机 ID={args.servo}，在线舵机: {sorted(models)}")
            return 1
        model_nb = models[args.servo]
        if model_nb != MODEL_STS3215:
            print(f"!! 警告：ID={args.servo} 型号号 {model_nb} ≠ {MODEL_STS3215}(STS3215)，继续尝试")

        # ---- 3. 计算并写入目标位置 ----
        pos = decode_sign_magnitude(bus.read_u16(args.servo, ADDR["present_position"][0]))
        vmin = bus.read_u16(args.servo, ADDR["min_position_limit"][0])
        vmax = bus.read_u16(args.servo, ADDR["max_position_limit"][0])
        print(f"\n[2/4] 当前状态: Present_Position={pos} ({counts_to_angle(pos):.2f}°)  限位=[{vmin}, {vmax}]")

        delta = angle_to_counts(args.angle)
        target = pos + delta
        clamped = max(vmin, min(vmax, target))
        if clamped != target:
            print(f"!! 目标 {target} 超出限位，已截断为 {clamped}")
            target = clamped
        if target == pos:
            print("!! 目标与当前位置相同，无需运动（可增大 --angle）")
            return 0

        print(f"\n[3/4] 使能力矩并运动 {args.angle}°（{args.duration_ms} ms 平滑）...")
        bus.write_u8(args.servo, ADDR["torque_enable"][0], 1)
        bus.write_u16(args.servo, ADDR["goal_time"][0], args.duration_ms)
        bus.write_u16(args.servo, ADDR["goal_position"][0], encode_sign_magnitude(target))
        print(f"     Goal_Position ← {target} ({counts_to_angle(target):.2f}°)")

        # ---- 4. 轮询回读，验证到位 ----
        deadline = time.time() + max(30.0, args.duration_ms / 1000 * 2 + 10)
        cur = pos
        while time.time() < deadline:
            try:
                cur = decode_sign_magnitude(bus.read_u16(args.servo, ADDR["present_position"][0]))
            except ProtocolError:
                continue  # 偶发通信失败则重试
            if abs(cur - target) <= args.tolerance:
                break
            time.sleep(0.05)

        print(f"\n[4/4] 回读验证: Present_Position={cur} ({counts_to_angle(cur):.2f}°)  目标={target}")
        if abs(cur - target) <= args.tolerance:
            print("\nD1 验收通过 ✅  舵机已平滑转动指定角度")
            return 0
        else:
            print(f"\n!! 未到位（差 {abs(cur - target)} 码 ≈ {counts_to_angle(abs(cur - target)):.2f}°）")
            return 2
    except KeyboardInterrupt:
        print("\n已中断（Ctrl+C）。舵机保持当前位置。")
        return 130
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
