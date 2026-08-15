"""本地 MAVLink 仿真器。

该模块只在本机 UDP 端口产生模拟遥测，并对收到的 COMMAND_LONG 返回模拟 ACK。
它不会连接串口、不会发送真实飞行指令，也不会修改飞控。
"""

from __future__ import annotations

import math
import socket
import time
from dataclasses import dataclass
from typing import Iterable

from pymavlink.dialects.v10 import common as mavlink


@dataclass
class SimState:
    time_boot_ms: int = 0
    lat: float = 23.1291
    lon: float = 113.2644
    alt: float = 20.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    battery_remaining: int = 100


def encode_messages(messages: Iterable[object]) -> bytes:
    """把 pymavlink 消息编码为 MAVLink1 字节流。"""
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    return b"".join(message.pack(encoder) for message in messages)


def make_telemetry(state: SimState) -> list[object]:
    """生成一组适合教学和联调的 MAVLink 遥测消息。"""
    return [
        mavlink.MAVLink_heartbeat_message(
            mavlink.MAV_TYPE_QUADROTOR,
            mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
            3,
        ),
        mavlink.MAVLink_attitude_message(
            state.time_boot_ms,
            math.radians(state.roll),
            math.radians(state.pitch),
            math.radians(state.yaw),
            0.0,
            0.0,
            0.0,
        ),
        mavlink.MAVLink_global_position_int_message(
            state.time_boot_ms,
            int(state.lat * 10_000_000),
            int(state.lon * 10_000_000),
            int(state.alt * 1000),
            int(state.alt * 1000),
            0,
            0,
            0,
            0,
        ),
    ]


def parse_datagram(data: bytes) -> list[object]:
    """解析一段 UDP 数据中的 MAVLink 消息。"""
    parser = mavlink.MAVLink(None)
    messages: list[object] = []
    for byte in data:
        message = parser.parse_char(bytes((byte,)))
        if message is not None:
            messages.append(message)
    return messages


def command_ack(command: int, result: int = mavlink.MAV_RESULT_ACCEPTED) -> object:
    return mavlink.MAVLink_command_ack_message(command, result)


def handle_message(message: object) -> list[object]:
    """处理仿真器收到的消息；目前只对 COMMAND_LONG 做 ACK。"""
    if isinstance(message, mavlink.MAVLink_command_long_message):
        return [command_ack(message.command)]
    return []


def run_udp_simulator(
    host: str = "127.0.0.1",
    port: int = 14560,
    rate_hz: float = 2.0,
    duration_s: float = 0.0,
) -> None:
    """运行本机 UDP 仿真器；duration_s 为 0 表示持续运行。"""
    if rate_hz <= 0:
        raise ValueError("rate_hz 必须大于 0")
    state = SimState()
    period = 1.0 / rate_hz
    started = time.monotonic()
    next_tick = started
    last_client: tuple[str, int] | None = None
    parser = mavlink.MAVLink(None)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        sock.settimeout(0.05)
        print(f"A9 离线 MAVLink 仿真器已启动：udp://{host}:{port}")
        print("不会连接真实飞控；按 Ctrl+C 停止。")
        while duration_s <= 0 or time.monotonic() - started < duration_s:
            now = time.monotonic()
            if now >= next_tick:
                state.time_boot_ms = int((now - started) * 1000)
                state.yaw = (state.time_boot_ms / 1000.0 * 10) % 360
                if last_client:
                    sock.sendto(encode_messages(make_telemetry(state)), last_client)
                next_tick = now + period

            try:
                data, address = sock.recvfrom(65535)
                last_client = address
                for byte in data:
                    message = parser.parse_char(bytes((byte,)))
                    if message is not None:
                        for response in handle_message(message):
                            sock.sendto(encode_messages([response]), address)
                        print(f"收到 {message.get_type()} from {address[0]}:{address[1]}")
            except socket.timeout:
                continue

