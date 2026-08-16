# -*- coding: utf-8 -*-
"""
手搓 Feetech（STS/SMS/SCS 系列，兼容幻尔 HX 舵机）串口协议层。

对照阅读（仅作参考，本文件为独立实现，不依赖 LeRobot / scservo_sdk）：
- LeRobot   src/lerobot/motors/feetech/feetech.py + tables.py
- Feetech SDK scservo_sdk/protocol_packet_handler.py

协议要点（Protocol 1.0，LeRobot 中 protocol_version=0）：
  指令帧: FF FF | ID | LENGTH | INSTRUCTION | PARAMS... | CHECKSUM
  状态帧: FF FF | ID | LENGTH | ERROR | PARAMS... | CHECKSUM
  LENGTH  = 除头部(FF FF ID LENGTH)外其余字节数 = INST/ERR + PARAMS + CHK
  CHECKSUM = ~(ID + LENGTH + INST/ERR + sum(PARAMS)) & 0xFF
  多字节数据为小端序（低字节在前）
  半双工单总线：写前清空接收缓冲，写后等待状态帧
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial

# ---- 指令码 ----
INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

BROADCAST_ID = 0xFE  # 254 广播地址
MAX_ID = 0xFC  # 252 最大可用 ID

# 状态帧 ERROR 字节的错误位
ERRBIT_VOLTAGE = 0x01
ERRBIT_ANGLE = 0x02
ERRBIT_OVERHEAT = 0x04
ERRBIT_OVERELE = 0x08
ERRBIT_OVERLOAD = 0x20

DEFAULT_BAUDRATE = 1_000_000  # HX 舵机 / STS3215 均为 1M

# STS3215：12 位磁编码器，4096 码 / 360°
MODEL_RESOLUTION_STS3215 = 4096
# 符号-数值编码：STS 系列位置/速度类 16 位寄存器以 bit15 为符号位
SIGN_MAGNITUDE_BIT = 15

# 关键寄存器地址（STS 系列控制表，见 tables.py；地址, 字节数）
ADDR = {
    "model_number": (3, 2),
    "id": (5, 1),
    "baud_rate": (6, 1),
    "return_delay_time": (7, 1),
    "min_position_limit": (9, 2),
    "max_position_limit": (11, 2),
    "phase": (18, 1),
    "homing_offset": (31, 2),
    "operating_mode": (33, 1),
    "torque_enable": (40, 1),
    "acceleration": (41, 1),
    "goal_position": (42, 2),
    "goal_time": (44, 2),
    "goal_velocity": (46, 2),
    "torque_limit": (48, 2),
    "lock": (55, 1),
    "present_position": (56, 2),
    "present_velocity": (58, 2),
    "present_voltage": (62, 1),
    "present_temperature": (63, 1),
    "moving": (66, 1),
}

# 型号号 → 分辨率
MODEL_RESOLUTION = {"sts3215": MODEL_RESOLUTION_STS3215}


class ProtocolError(Exception):
    """协议层错误（帧校验失败、超时、舵机错误位、应答不匹配）。"""


def checksum(body: list[int]) -> int:
    """计算校验和：~(非头部非校验和字节之和) & 0xFF。body 为 ID..最后一个 PARAM。"""
    return (~sum(body)) & 0xFF


def build_instruction_packet(
    servo_id: int, instruction: int, params: list[int] | tuple[int, ...] = ()
) -> bytes:
    """构造指令帧：FF FF ID LEN INST P0..PN CHK，LEN = 1(INST) + N(params) + 1(CHK)。"""
    if not 0 <= servo_id <= BROADCAST_ID:
        raise ProtocolError(f"非法舵机 ID: {servo_id}")
    body = [servo_id, 1 + len(params) + 1, instruction, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


@dataclass
class StatusPacket:
    servo_id: int
    error: int
    params: bytes
    length: int


def parse_status_packet(packet: bytes) -> StatusPacket:
    """解析单个完整状态帧。帧格式：FF FF ID LEN ERR P0..PN CHK，LEN = 1(ERR)+N+1(CHK)。"""
    if len(packet) < 6:
        raise ProtocolError(f"状态帧过短: {packet.hex()}")
    if packet[0] != 0xFF or packet[1] != 0xFF:
        raise ProtocolError(f"状态帧头部错误: {packet.hex()}")
    servo_id, length, error = packet[2], packet[3], packet[4]
    params = packet[5:-1]
    if len(params) + 2 != length:
        raise ProtocolError(f"状态帧 LENGTH 不符: 期望 {len(params) + 2}, 实际 {length}")
    if packet[-1] != checksum([servo_id, length, error, *params]):
        raise ProtocolError(f"状态帧校验和错误: {packet.hex()}")
    return StatusPacket(servo_id, error, params, length)


def parse_status_stream(raw: bytes) -> list[StatusPacket]:
    """从连续字节流中解析出全部状态帧（用于广播 Ping 多舵机应答场景）。"""
    packets: list[StatusPacket] = []
    i = 0
    while i < len(raw) - 1:
        if raw[i] != 0xFF or raw[i + 1] != 0xFF:
            i += 1
            continue  # 重新对齐帧头
        if i + 4 > len(raw):
            break  # 头部不完整
        length = raw[i + 3]
        end = i + 4 + length  # 4(头部) + LEN
        if end > len(raw):
            break  # 数据不完整
        try:
            packets.append(parse_status_packet(raw[i:end]))
        except ProtocolError:
            pass  # 跳过坏帧，继续对齐
        i = end
    return packets


# ---- 符号-数值编解码（STS 系列，bit15 为符号位） ----
def encode_sign_magnitude(value: int, bit: int = SIGN_MAGNITUDE_BIT) -> int:
    """负数以符号-数值编码：符号位置 1，数值取绝对值。"""
    return (-value | (1 << bit)) if value < 0 else value


def decode_sign_magnitude(value: int, bit: int = SIGN_MAGNITUDE_BIT) -> int:
    """解码符号-数值编码。"""
    return -(value & ~(1 << bit)) if value & (1 << bit) else value


def split_u16(value: int) -> tuple[int, int]:
    """小端序拆分 2 字节。"""
    return value & 0xFF, (value >> 8) & 0xFF


def join_u16(lo: int, hi: int) -> int:
    """小端序合成 2 字节。"""
    return lo | (hi << 8)


class FeetechSerialBus:
    """基于 pyserial 的 Feetech 总线（半双工 UART）。

    半双工方向切换由 BusLinker(CH343) 硬件自动完成，软件侧只需：
    写前清空接收缓冲 → 发送 → 读回状态帧。
    """

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, read_timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=read_timeout,
        )
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    # ---- 底层收发 ----
    def _tx(self, packet: bytes) -> None:
        self.ser.reset_input_buffer()  # 丢弃残余接收数据，避免串包
        self.ser.write(packet)
        self.ser.flush()

    def _read_exact(self, n: int) -> bytes:
        """阻塞读取 n 字节（受串口 timeout 约束），超时抛 ProtocolError。"""
        chunks = bytearray()
        while len(chunks) < n:
            chunk = self.ser.read(n - len(chunks))
            if not chunk:
                raise ProtocolError(f"串口读取超时（期望 {n} 字节，实际 {len(chunks)}）")
            chunks += chunk
        return bytes(chunks)

    def _rx_status(self, expected_id: int | None = None) -> StatusPacket:
        """读取并解析一个状态帧，可选校验应答舵机 ID。"""
        header = self._read_exact(4)
        if header[0:2] != b"\xff\xff":
            raise ProtocolError(f"状态帧头部错误: {header.hex()}")
        length = header[3]
        status = parse_status_packet(header + self._read_exact(length))
        if expected_id is not None and status.servo_id != expected_id:
            raise ProtocolError(f"应答舵机 ID 不符: 期望 {expected_id}, 实际 {status.servo_id}")
        return status

    # ---- 基本指令 ----
    def ping(self, servo_id: int) -> StatusPacket | None:
        """Ping 单个舵机；无应答返回 None（不抛异常）。"""
        try:
            self._tx(build_instruction_packet(servo_id, INST_PING))
            return self._rx_status(expected_id=servo_id)
        except ProtocolError:
            return None

    def read_register(self, servo_id: int, address: int, length: int) -> bytes:
        """读寄存器原始字节。"""
        self._tx(build_instruction_packet(servo_id, INST_READ, [address, length]))
        status = self._rx_status(expected_id=servo_id)
        if status.error:
            raise ProtocolError(
                f"舵机 {servo_id} 读 @0x{address:02X} 返回错误位 0x{status.error:02X}"
            )
        if len(status.params) != length:
            raise ProtocolError(
                f"舵机 {servo_id} 读 @0x{address:02X} 数据长度不符: 期望 {length}, 实际 {len(status.params)}"
            )
        return status.params

    def read_u16(self, servo_id: int, address: int) -> int:
        return join_u16(*self.read_register(servo_id, address, 2))

    def read_u8(self, servo_id: int, address: int) -> int:
        return self.read_register(servo_id, address, 1)[0]

    def write_register(self, servo_id: int, address: int, data: list[int] | tuple[int, ...]) -> None:
        """写寄存器（等待状态帧确认）。"""
        self._tx(build_instruction_packet(servo_id, INST_WRITE, [address, *data]))
        status = self._rx_status(expected_id=servo_id)
        if status.error:
            raise ProtocolError(
                f"舵机 {servo_id} 写 @0x{address:02X} 返回错误位 0x{status.error:02X}"
            )

    def write_u16(self, servo_id: int, address: int, value: int) -> None:
        self.write_register(servo_id, address, split_u16(value))

    def write_u8(self, servo_id: int, address: int, value: int) -> None:
        self.write_register(servo_id, address, [value])

    # ---- 总线扫描 ----
    def scan(self, max_id: int = 32, ping_timeout: float = 0.02) -> dict[int, int]:
        """顺序 Ping 扫描总线，返回 {舵机ID: 型号号}。

        注：实测 HX 固件下广播 Ping（ID=0xFE）多舵机同时应答会在半双工单线上
        碰撞损坏（每次只收到 1-2 个且随机），LeRobot 同样依赖顺序 Ping 兜底。
        这里以顺序 Ping 为准：对 1..max_id 逐个探测，速度可接受且稳定。
        """
        saved_timeout = self.ser.timeout
        self.ser.timeout = ping_timeout  # 缩短无应答等待，加快扫描
        models: dict[int, int] = {}
        try:
            for servo_id in range(1, max_id + 1):
                if self.ping(servo_id) is not None:
                    try:
                        models[servo_id] = self.read_u16(servo_id, ADDR["model_number"][0])
                    except ProtocolError:
                        models[servo_id] = -1  # 型号号读取失败
        finally:
            self.ser.timeout = saved_timeout
        return models

    def scan_broadcast(self, quiet_timeout: float = 0.05) -> dict[int, int]:
        """广播 Ping 扫描（仅作协议学习/对照用；HX 固件下结果不稳定）。

        发送一次广播 Ping（ID=0xFE），收集到总线静默后解析应答，再逐个读型号号。
        """
        self._tx(build_instruction_packet(BROADCAST_ID, INST_PING))
        raw = bytearray()
        saved_timeout = self.ser.timeout
        self.ser.timeout = quiet_timeout
        try:
            while True:
                chunk = self.ser.read(max(self.ser.in_waiting, 1))
                if not chunk:
                    break  # 超过 quiet_timeout 无新数据 → 应答结束
                raw += chunk
        finally:
            self.ser.timeout = saved_timeout

        ids = sorted({p.servo_id for p in parse_status_stream(bytes(raw))})
        models: dict[int, int] = {}
        for servo_id in ids:
            try:
                models[servo_id] = self.read_u16(servo_id, ADDR["model_number"][0])
            except ProtocolError:
                models[servo_id] = -1  # 型号号读取失败
        return models


def angle_to_counts(angle_deg: float, resolution: int = MODEL_RESOLUTION_STS3215) -> int:
    """角度 → 编码器码值（4096 码 = 360°）。"""
    return round(angle_deg * resolution / 360.0)


def counts_to_angle(counts: int, resolution: int = MODEL_RESOLUTION_STS3215) -> float:
    """编码器码值 → 角度。"""
    return counts * 360.0 / resolution


# 兼容 LeRobot tables.py 的别名，便于对照阅读
sts_sms_series_control_table = {k: v for k, v in ADDR.items()}
