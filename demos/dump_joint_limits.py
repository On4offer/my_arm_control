#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
关节限位诊断工具：读取两臂各关节的 EEPROM 限位/当前值，输出参考表 + JSON
======================================================================

关节命名（SO-101，厂商使用文档）：从上往下
  gripper(ID6) wrist_roll(ID5) wrist_flex(ID4) elbow_flex(ID3)
  shoulder_lift(ID2) shoulder_pan(ID1)

用法：
  python dump_joint_limits.py COM22            # 单臂
  python dump_joint_limits.py COM22 COM24      # 双臂对比
  python dump_joint_limits.py COM22 --json out.json
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.protocol import ADDR, DEFAULT_BAUDRATE, FeetechSerialBus, ProtocolError, counts_to_angle, decode_sign_magnitude

# ID -> (关节名, 备注)
JOINT_NAMES = {
    1: ("shoulder_pan", "底盘/肩部旋转"),
    2: ("shoulder_lift", "肩部抬升(大臂，重载)"),
    3: ("elbow_flex", "肘部"),
    4: ("wrist_flex", "腕部俯仰"),
    5: ("wrist_roll", "腕部旋转"),
    6: ("gripper", "夹爪"),
}


def dump(port: str) -> dict:
    bus = FeetechSerialBus(port=port, baudrate=DEFAULT_BAUDRATE)
    rows = {}
    try:
        models = bus.scan()
        for sid in sorted(models):
            lo = bus.read_u16(sid, ADDR["min_position_limit"][0])
            hi = bus.read_u16(sid, ADDR["max_position_limit"][0])
            present = decode_sign_magnitude(bus.read_u16(sid, ADDR["present_position"][0]))
            name, note = JOINT_NAMES.get(sid, (f"id{sid}", ""))
            rows[sid] = {
                "name": name,
                "note": note,
                "model": models[sid],
                "min": lo,
                "max": hi,
                "range_deg": round(counts_to_angle(hi - lo), 1),
                "present": present,
                "present_deg": round(counts_to_angle(present), 1),
            }
    finally:
        bus.close()
    return rows


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="关节限位诊断")
    p.add_argument("ports", nargs="+", help="一个或多个串口，如 COM22 COM24")
    p.add_argument("--json", default=None, help="输出 JSON 路径")
    args = p.parse_args(argv)

    result = {}
    for port in args.ports:
        print(f"\n{'=' * 66}\n端口 {port}\n{'=' * 66}")
        print(f"{'ID':<4}{'关节':<16}{'型号':<6}{'Min':>7}{'Max':>7}{'行程°':>8}{'当前':>7}{'当前°':>9}")
        rows = dump(port)
        for sid in sorted(rows):
            r = rows[sid]
            print(f"{sid:<4}{r['name']:<16}{r['model']:<6}{r['min']:>7}{r['max']:>7}"
                  f"{r['range_deg']:>8}{r['present']:>7}{r['present_deg']:>9}  {r['note']}")
        result[port] = rows

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        print(f"\n已保存: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
