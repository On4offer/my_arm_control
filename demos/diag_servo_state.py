#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
舵机状态诊断工具（D2/D3 真机排查用）
====================================

排查两类问题：
  1. 只读 dump：每个舵机的关键寄存器全貌（ID/型号/限位/当前位置/homing_offset/
     phase/加速度/力矩开关/温度电压/moving），用于判断 ID 映射、标定、方向相关配置。
  2. 方向探针（--probe）：对单个关节做小幅 ±delta 运动，实测"码值增大 → 物理转动方向"，
     并读回实际位移，用于确认码值方向与机械方向是否一致、是否反装。

用法：
  python diag_servo_state.py --port COM22 --dump
  python diag_servo_state.py --port COM22 --probe 2 --delta 200   # 单关节 ±200 码小步验证方向
  python diag_servo_state.py --port COM22 --dump --probe 2        # 先 dump 再 probe
  python diag_servo_state.py --port COM22 --manual 2              # 失能后手动推臂实测机械范围/方向
  python diag_servo_state.py --port COM22 --manual 2 --write-limits  # 实测并写回 EEPROM 限位
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.protocol import (  # noqa: E402
    ADDR,
    DEFAULT_BAUDRATE,
    FeetechSerialBus,
    ProtocolError,
    decode_sign_magnitude,
    encode_sign_magnitude,
)

DEFAULT_SERVOS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

# 需要 dump 的关键寄存器：{寄存器名: (地址, 字节数)}
DUMP_REGISTERS = {
    "model_number": ADDR["model_number"],
    "id": ADDR["id"],
    "baud_rate": ADDR["baud_rate"],
    "return_delay_time": ADDR["return_delay_time"],
    "min_position_limit": ADDR["min_position_limit"],
    "max_position_limit": ADDR["max_position_limit"],
    "phase": ADDR["phase"],
    "homing_offset": ADDR["homing_offset"],
    "operating_mode": ADDR["operating_mode"],
    "torque_enable": ADDR["torque_enable"],
    "acceleration": ADDR["acceleration"],
    "goal_position": ADDR["goal_position"],
    "goal_time": ADDR["goal_time"],
    "goal_velocity": ADDR["goal_velocity"],
    "torque_limit": ADDR["torque_limit"],
    "lock": ADDR["lock"],
    "present_position": ADDR["present_position"],
    "present_velocity": ADDR["present_velocity"],
    "present_voltage": ADDR["present_voltage"],
    "present_temperature": ADDR["present_temperature"],
    "moving": ADDR["moving"],
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="舵机状态诊断：dump 寄存器 / 单关节方向探针")
    p.add_argument("--port", required=True, help="舵机总线串口，如 COM22")
    p.add_argument("--servos", default="1,2,3,4,5,6", help="目标舵机 ID 列表（逗号分隔）")
    p.add_argument("--dump", action="store_true", help="只读 dump 各舵机关键寄存器")
    p.add_argument("--probe", type=int, default=None, metavar="ID", help="对指定关节做 ±delta 小步方向验证")
    p.add_argument("--delta", type=int, default=200, help="探针步长（码，默认 200≈17.6°）")
    p.add_argument("--manual", type=int, default=None, metavar="ID",
                   help="失能后定时采样实测机械范围/方向（无需按键，按提示推臂）")
    p.add_argument("--window", type=float, default=45.0, help="--manual 采样窗口（秒，默认 45）")
    p.add_argument("--write-limits", action="store_true", help="配合 --manual：把实测范围写回 EEPROM 限位")
    return p.parse_args(argv)


def read_or_none(bus: FeetechSerialBus, sid: int, addr: int, length: int) -> tuple[int | None, str | None]:
    """读寄存器，失败返回 (None, 错误信息)，不中断。"""
    try:
        if length == 1:
            return bus.read_u8(sid, addr), None
        return bus.read_u16(sid, addr), None
    except ProtocolError as e:
        return None, str(e)


def fmt_value(name: str, addr: int, length: int, val: int | None) -> str:
    if val is None:
        return "--"
    if name in ("min_position_limit", "max_position_limit", "present_position", "goal_position", "homing_offset"):
        # 位置类寄存器按符号-数值解码显示，同时给出原始码值
        return f"{decode_sign_magnitude(val):>6} (raw 0x{val:04X})"
    if length == 2:
        return f"{val} (0x{val:04X})"
    return f"{val} (0x{val:02X})"


def dump_all(bus: FeetechSerialBus, servo_ids: list[int]) -> None:
    print("=" * 92)
    print("只读 dump：各舵机关键寄存器（位置类按符号-数值解码）")
    print("=" * 92)
    for sid in servo_ids:
        print(f"\n--- 舵机 ID={sid} {JOINT_NAMES.get(sid, '')} ---")
        for name, (addr, length) in DUMP_REGISTERS.items():
            val, err = read_or_none(bus, sid, addr, length)
            if err:
                print(f"  @0x{addr:02X} {name:<22}: ERROR {err}")
            else:
                print(f"  @0x{addr:02X} {name:<22}: {fmt_value(name, addr, length, val)}")


def probe_joint(bus: FeetechSerialBus, sid: int, delta: int) -> None:
    """对单个关节做 ±delta 小步：实测码值-物理方向，并验证小步能正常转动。"""
    print("=" * 92)
    print(f"方向探针：舵机 ID={sid} {JOINT_NAMES.get(sid, '')}，步长 ±{delta} 码（≈{delta*360/4096:.1f}°）")
    print("请观察物理转动方向并对照打印结果。")
    print("=" * 92)

    min_pos = bus.read_u16(sid, ADDR["min_position_limit"][0])
    max_pos = bus.read_u16(sid, ADDR["max_position_limit"][0])
    p0 = decode_sign_magnitude(bus.read_u16(sid, ADDR["present_position"][0]))
    print(f"限位 [{min_pos}, {max_pos}]，当前位置 {p0}（{p0*360/4096:.1f}°）")

    # 降低舵机内部加速度，防止小步突跳
    try:
        bus.write_u8(sid, ADDR["acceleration"][0], 100)
    except ProtocolError as e:
        print(f"  !! 降加速度失败: {e}")

    def step(direction: int) -> None:
        q = p0 + direction * delta
        q_clamped = min(max(q, min_pos + 50), max_pos - 50)
        if q_clamped != q:
            print(f"  !! 目标 {q} 超出安全限位，已截断到 {q_clamped}（可能已接近机械极限）")
        print(f"\n  写 Goal = {q_clamped} ({q_clamped*360/4096:.1f}°) ...", end=" ", flush=True)
        try:
            bus.write_u16(sid, ADDR["goal_position"][0], encode_sign_magnitude(q_clamped))
            # 等舵机内部走完（moving 位清零或超时）
            for _ in range(200):  # 最多 ~10s
                moving = bus.read_u8(sid, ADDR["moving"][0])
                if moving == 0:
                    break
                time.sleep(0.05)
            p1 = decode_sign_magnitude(bus.read_u16(sid, ADDR["present_position"][0]))
            print(f"到位，实测位置 {p1}（Δ={p1 - p0:+d} 码）")
        except ProtocolError as e:
            print(f"失败: {e}（可能过载/越限）")

    step(+1)
    # 回到起点附近
    try:
        bus.write_u16(sid, ADDR["goal_position"][0], encode_sign_magnitude(p0))
        for _ in range(200):
            moving = bus.read_u8(sid, ADDR["moving"][0])
            if moving == 0:
                break
            time.sleep(0.05)
    except ProtocolError as e:
        print(f"  回起点失败: {e}")
    step(-1)


def manual_calibrate(bus: FeetechSerialBus, sid: int, write_limits: bool, window: float = 45.0) -> None:
    """失能力矩 → 定时采样（无终端交互）→ 实测机械范围 + 方向 → 可选写回 EEPROM。

    用户动作（脚本启动后按顺序做一遍）：
      1) 先把该关节推到【机械最高点】停 3 秒
      2) 再推到【机械最低点】停 3 秒
    脚本每 0.4s 刷新一次读数（\r 覆盖打印），全程约 window 秒后自动停止，
    期间用户只需观察终端上不断变化的数值。

    方向结论由"前 20% 采样中位数 vs 后 20% 采样中位数"自动判断：
      前大后小 → 码值增大 = 物理向上/抬（正常装配）
      前小后大 → 码值增大 = 物理向下/压（反装，需排查）
    """
    print("=" * 92)
    print(f"手动标定：舵机 ID={sid} {JOINT_NAMES.get(sid, '')}")
    print("流程：失能力矩 → 定时采样实测机械范围与码值方向（约 %.0f 秒）" % window)
    print("=" * 92)

    ee_min = bus.read_u16(sid, ADDR["min_position_limit"][0])
    ee_max = bus.read_u16(sid, ADDR["max_position_limit"][0])
    p0 = decode_sign_magnitude(bus.read_u16(sid, ADDR["present_position"][0]))
    print(f"EEPROM 限位 [{ee_min}, {ee_max}]，当前读数 {p0}（{p0*360/4096:.1f}°）")

    # 1) 失能力矩（该关节可被外力自由转动）
    bus.write_u8(sid, ADDR["torque_enable"][0], 0)
    print(f"\n✅ ID={sid} 力矩已失能（其他关节保持力矩，机械臂不会散架）")
    print("请现在开始按顺序操作（无需按键，40 秒内完成）：")
    print("  ① 握住该关节驱动的构件，缓慢推到【机械最高点】，停住 3 秒；")
    print("  ② 再缓慢推到【机械最低点】，停住 3 秒；")
    print("  ③ 之后可以松手让重力自然下垂，等待采样结束。")
    print("  提示：推不动说明已到机械挡块；若太重，推到能到的最远位置即可。")
    print("  —— 实时读数见下方（每 0.4s 刷新）——\n")

    # 2) 定时采样
    samples: list[tuple[float, float]] = []
    t0 = time.perf_counter()
    mn = mx = float(p0)
    last_err = 0
    try:
        while True:
            t = time.perf_counter() - t0
            if t > window:
                break
            try:
                pos = float(decode_sign_magnitude(bus.read_u16(sid, ADDR["present_position"][0])))
                last_err = 0
            except ProtocolError as e:
                pos = float("nan")
                last_err += 1
            samples.append((t, pos))
            if pos == pos:  # 非 NaN
                mn = min(mn, pos)
                mx = max(mx, pos)
            deg = pos * 360.0 / 4096.0 if pos == pos else float("nan")
            print(f"\r  t={t:5.1f}s/{window:.0f}s  present={pos:>7.0f} ({deg:>7.1f}°)  min={mn:>7.0f}  max={mx:>7.0f}", end="", flush=True)
            time.sleep(0.4)
    except KeyboardInterrupt:
        print("\n用户中断采样")
    print()

    # 3) 统计与方向判断
    valid = [(t, p) for t, p in samples if p == p]
    if len(valid) < 10:
        print("!! 采样数据不足（至少 10 个有效点），未完成标定")
        return
    n = len(valid)
    head = sorted(p for _, p in valid[: n // 5])
    tail = sorted(p for _, p in valid[-(n // 5):])
    h_mid = head[len(head) // 2]
    t_mid = tail[len(tail) // 2]
    print(f"\n实测范围 [min={mn:.0f}, max={mx:.0f}]，跨度 {(mx - mn) * 360 / 4096:.1f}°")
    print(f"前段中位数(≈最高点)={h_mid:.0f}，后段中位数(≈最低点)={t_mid:.0f}")
    if h_mid > t_mid:
        print("方向结论：前段(最高)>后段(最低) → 码值增大 = 物理向上/抬（正常装配）")
        lo, hi = mn, mx
    else:
        print("方向结论：前段(最高)<后段(最低) → 码值增大 = 物理向下/压（反装，需排查）")
        lo, hi = mn, mx  # 写 EEPROM 时保证 min<max

    # 4) 可选写回 EEPROM 限位（留 20 码余量）
    if write_limits:
        margin = 20
        new_min = int(max(0, lo + margin))
        new_max = int(min(4095, hi - margin))
        if new_min >= new_max:
            print("  !! 实测范围异常（min>=max），不写入 EEPROM")
        else:
            try:
                bus.write_u16(sid, ADDR["min_position_limit"][0], new_min)
                bus.write_u16(sid, ADDR["max_position_limit"][0], new_max)
                print(f"  ✅ 已写入 EEPROM 限位 [{new_min}, {new_max}]（留 {margin} 码余量）")
            except ProtocolError as e:
                print(f"  !! EEPROM 写入失败: {e}")

    # 5) 不自动恢复力矩：让用户先把大臂摆到中间位置，恢复命令单独执行（避免方向未知时硬拉）
    print("\n⚠ ID=%d 保持失能。请把该关节手动摆回量程中部（中间位置），告诉我读数，我再执行恢复力矩命令。" % sid)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    servo_ids = [int(s) for s in args.servos.split(",")]
    if not (args.dump or args.probe or args.manual):
        print("请至少指定 --dump / --probe ID / --manual ID")
        return 1

    bus = FeetechSerialBus(port=args.port, baudrate=DEFAULT_BAUDRATE)
    try:
        models = bus.scan()
        if not models:
            print(f"!! {args.port} 总线无应答：请检查 12V 电源与 BusLinker")
            return 1
        print(f"扫描到 {len(models)} 个舵机: {sorted(models)}")

        if args.dump:
            dump_all(bus, servo_ids)
        if args.probe is not None:
            if args.probe not in servo_ids:
                print(f"!! 探针目标 {args.probe} 不在舵机列表 {servo_ids} 中")
                return 1
            probe_joint(bus, args.probe, args.delta)
        if args.manual is not None:
            if args.manual not in servo_ids:
                print(f"!! 标定目标 {args.manual} 不在舵机列表 {servo_ids} 中")
                return 1
            manual_calibrate(bus, args.manual, args.write_limits, args.window)
        print("\n诊断完成 ✅")
        return 0
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    finally:
        bus.close()


if __name__ == "__main__":
    sys.exit(main())
