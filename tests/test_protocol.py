#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
D1 协议层离线单测：无需硬件，验证帧构造、解析、校验和、符号-数值编解码。

运行：
  python test_protocol.py        # 直接运行
  pytest test_protocol.py        # 或 pytest 方式
"""

import sys
from pathlib import Path

# 把项目根目录加入 sys.path，使 my_arm_control 包可直接导入（无需 pip install）
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from my_arm_control.protocol import (
    BROADCAST_ID,
    INST_PING,
    INST_READ,
    INST_WRITE,
    ProtocolError,
    StatusPacket,
    angle_to_counts,
    build_instruction_packet,
    checksum,
    counts_to_angle,
    decode_sign_magnitude,
    encode_sign_magnitude,
    join_u16,
    parse_status_packet,
    parse_status_stream,
    split_u16,
)


# ---- 帧构造（校验和手算对照 SDK 公式） ----
def test_ping_frame():
    # body=[ID=1, LEN=2, INST=0x01] sum=4 → chk=~4&0xFF=0xFB
    assert build_instruction_packet(1, INST_PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def test_read_frame():
    # READ ID=1 @0x2A len=2: body=[1,4,2,0x2A,2] sum=0x33 → chk=0xCC
    assert build_instruction_packet(1, INST_READ, [0x2A, 0x02]) == bytes(
        [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2A, 0x02, 0xCC]
    )


def test_write_frame():
    # WRITE ID=1 @0x2A data=[0x80,0x08] (0x0880): body=[1,5,3,0x2A,0x80,0x08] sum=0xBB → chk=0x44
    assert build_instruction_packet(1, INST_WRITE, [0x2A, 0x80, 0x08]) == bytes(
        [0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x80, 0x08, 0x44]
    )


def test_broadcast_ping_frame():
    # 广播 Ping: ID=0xFE, body=[0xFE,2,1] sum=0x101=257 → chk=~257&0xFF=0xFE
    pkt = build_instruction_packet(BROADCAST_ID, INST_PING)
    assert pkt[0:2] == b"\xff\xff" and pkt[2] == BROADCAST_ID
    assert checksum(list(pkt[2:-1])) == pkt[-1]


# ---- 状态帧解析 ----
def test_parse_status_packet():
    # 读 u16 应答 value=0x0880: FF FF 01 04 00 80 08 72
    s = parse_status_packet(bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x80, 0x08, 0x72]))
    assert isinstance(s, StatusPacket)
    assert (s.servo_id, s.error, s.length) == (1, 0, 4)
    assert s.params == bytes([0x80, 0x08])


def test_parse_status_packet_bad_checksum():
    try:
        parse_status_packet(bytes([0xFF, 0xFF, 0x01, 0x04, 0x00, 0x80, 0x08, 0x00]))
        raise AssertionError("应抛出 ProtocolError")
    except ProtocolError:
        pass


def test_parse_status_stream_multi():
    # 两个 Ping 应答：ID=1 (chk=0xFC) + ID=2 (chk=0xFB)，中间夹杂垃圾字节
    raw = b"\xaa\xbb\xff\xff\x01\x02\x00\xfc\xff\xff\x02\x02\x00\xfb\x99"
    packets = parse_status_stream(raw)
    assert [(p.servo_id, p.error) for p in packets] == [(1, 0), (2, 0)]


# ---- 符号-数值编解码 ----
def test_sign_magnitude_roundtrip():
    for v in [-4095, -2048, -1, 0, 1, 2047, 4095]:
        assert decode_sign_magnitude(encode_sign_magnitude(v)) == v
    assert encode_sign_magnitude(-5) == 0x8005
    assert decode_sign_magnitude(0x8005) == -5


# ---- 小端序 ----
def test_split_join_u16():
    assert split_u16(0x0880) == (0x80, 0x08)
    assert join_u16(0x80, 0x08) == 0x0880


# ---- 角度 ↔ 码值 ----
def test_angle_counts():
    assert angle_to_counts(360.0) == 4096
    assert angle_to_counts(30.0) == 341  # round(30*4096/360)=341
    assert abs(counts_to_angle(341) - 29.97) < 0.05


if __name__ == "__main__":
    import sys
    import traceback

    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    passed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
            passed += 1
        except Exception:
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n汇总: {passed} PASS / {len(fns) - passed} FAIL / {len(fns)} 总检查项")
    sys.exit(0 if passed == len(fns) else 1)
