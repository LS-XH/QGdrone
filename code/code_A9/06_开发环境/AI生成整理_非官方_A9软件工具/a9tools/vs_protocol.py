"""VS V1 AI—嵌入式协议的离线编解码器。"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag


MAGIC = b"VS"
VERSION = 1
MAX_PAYLOAD = 128
HEADER_SIZE = 14


class ProtocolError(ValueError):
    """协议帧无法解析或校验失败。"""


class FrameFlags(IntFlag):
    ACK_REQUIRED = 1 << 0
    IS_RESPONSE = 1 << 1
    HIGH_PRIORITY = 1 << 2


class MessageType(IntEnum):
    HEARTBEAT = 0x0001
    ACK = 0x0002
    SET_TARGET_LOCAL = 0x0101
    ACTION_COMMAND = 0x0102
    SCAN_STATUS = 0x0103
    HAZARD_ALERT = 0x0104
    FLIGHT_STATUS = 0x0201
    MISSION_EVENT = 0x0202
    SAFETY_EVENT = 0x0203


class AckResult(IntEnum):
    ACCEPTED = 0
    TEMPORARILY_REJECTED = 1
    PERMANENTLY_REJECTED = 2
    MALFORMED = 3
    EXPIRED = 4
    DUPLICATE = 5


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    timestamp_ms: int
    payload: bytes
    version: int = VERSION


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def encode_frame(frame: Frame) -> bytes:
    payload = bytes(frame.payload)
    if frame.version != VERSION:
        raise ValueError("unsupported protocol version")
    if not 0 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("payload length must be 0..128")
    if not 0 <= int(frame.message_type) <= 0xFFFF:
        raise ValueError("message type out of range")
    if not 0 <= int(frame.flags) <= 0xFF:
        raise ValueError("flags out of range")
    if not 0 <= int(frame.sequence) <= 0xFFFF:
        raise ValueError("sequence out of range")
    if not 0 <= int(frame.timestamp_ms) <= 0xFFFFFFFF:
        raise ValueError("timestamp out of range")
    body = struct.pack(
        "<BBHHHI",
        frame.version,
        int(frame.flags),
        int(frame.message_type),
        len(payload),
        int(frame.sequence),
        int(frame.timestamp_ms),
    ) + payload
    return MAGIC + body + struct.pack("<H", crc16_ccitt_false(body))


def decode_frames(data: bytes) -> list[Frame]:
    frames: list[Frame] = []
    cursor = 0
    while cursor < len(data):
        start = data.find(MAGIC, cursor)
        if start < 0:
            break
        if len(data) - start < HEADER_SIZE + 2:
            raise ProtocolError("truncated frame header")
        version, flags, message_type, payload_length, sequence, timestamp = struct.unpack_from(
            "<BBHHHI", data, start + 2
        )
        if version != VERSION:
            raise ProtocolError(f"unsupported protocol version: {version}")
        if payload_length > MAX_PAYLOAD:
            raise ProtocolError("payload length exceeds 128")
        total = HEADER_SIZE + payload_length + 2
        if len(data) - start < total:
            raise ProtocolError("truncated frame")
        body = data[start + 2 : start + 2 + HEADER_SIZE - 2 + payload_length]
        # body is version..timestamp plus payload: 12 + payload bytes.
        expected = struct.unpack_from("<H", data, start + total - 2)[0]
        actual = crc16_ccitt_false(body)
        if expected != actual:
            raise ProtocolError(f"CRC mismatch: expected 0x{expected:04X}, got 0x{actual:04X}")
        payload_start = start + HEADER_SIZE
        frames.append(Frame(message_type, flags, sequence, timestamp, data[payload_start : payload_start + payload_length], version))
        cursor = start + total
    return frames


def pack_set_target_local(
    target_id: int,
    x_m: float,
    y_m: float,
    z_m: float,
    yaw_deg: float,
    max_speed_m_s: float,
    acceptance_radius_m: float,
    valid_for_ms: int,
) -> bytes:
    values = [_finite(value, name) for value, name in zip(
        (x_m, y_m, z_m, yaw_deg, max_speed_m_s, acceptance_radius_m),
        ("x_m", "y_m", "z_m", "yaw_deg", "max_speed_m_s", "acceptance_radius_m"),
    )]
    if not 0 <= int(target_id) <= 0xFFFFFFFF or not 0 <= int(valid_for_ms) <= 0xFFFFFFFF:
        raise ValueError("target id or validity is out of range")
    return struct.pack("<I6fI", int(target_id), *values, int(valid_for_ms))


def unpack_set_target_local(payload: bytes) -> dict[str, int | float]:
    if len(payload) != struct.calcsize("<I6fI"):
        raise ProtocolError("SET_TARGET_LOCAL payload length mismatch")
    target_id, x, y, z, yaw, max_speed, radius, valid_for = struct.unpack("<I6fI", payload)
    values = (x, y, z, yaw, max_speed, radius)
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("SET_TARGET_LOCAL contains non-finite float")
    return {
        "target_id": target_id,
        "x_m": x,
        "y_m": y,
        "z_m": z,
        "yaw_deg": yaw,
        "max_speed_m_s": max_speed,
        "acceptance_radius_m": radius,
        "valid_for_ms": valid_for,
    }


def pack_ack(acked_sequence: int, acked_type: int, result: int, reason: int) -> bytes:
    return struct.pack("<HHBB", acked_sequence, int(acked_type), int(result), reason)


def unpack_ack(payload: bytes) -> dict[str, int]:
    if len(payload) != struct.calcsize("<HHBB"):
        raise ProtocolError("ACK payload length mismatch")
    sequence, message_type, result, reason = struct.unpack("<HHBB", payload)
    return {"acked_sequence": sequence, "acked_type": message_type, "result": result, "reason": reason}


def pack_hazard_alert(
    hazard_id: int,
    x_m: float,
    y_m: float,
    z_m: float,
    radius_m: float,
    level: int,
    suggested_action: int,
    valid_for_ms: int,
) -> bytes:
    values = [_finite(value, name) for value, name in zip(
        (x_m, y_m, z_m, radius_m), ("x_m", "y_m", "z_m", "radius_m")
    )]
    if not 0 <= int(hazard_id) <= 0xFFFFFFFF:
        raise ValueError("hazard id out of range")
    if not 0 <= int(level) <= 0xFF or not 0 <= int(suggested_action) <= 0xFF:
        raise ValueError("hazard level or action out of range")
    if not 0 <= int(valid_for_ms) <= 0xFFFFFFFF:
        raise ValueError("hazard validity out of range")
    return struct.pack(
        "<I4fBBHI", int(hazard_id), *values, int(level), int(suggested_action), 0, int(valid_for_ms)
    )


def unpack_hazard_alert(payload: bytes) -> dict[str, int | float]:
    if len(payload) != struct.calcsize("<I4fBBHI"):
        raise ProtocolError("HAZARD_ALERT payload length mismatch")
    hazard_id, x, y, z, radius, level, action, _reserved, valid_for = struct.unpack(
        "<I4fBBHI", payload
    )
    if not all(math.isfinite(value) for value in (x, y, z, radius)):
        raise ProtocolError("HAZARD_ALERT contains non-finite float")
    return {
        "hazard_id": hazard_id,
        "x_m": x,
        "y_m": y,
        "z_m": z,
        "radius_m": radius,
        "level": level,
        "suggested_action": action,
        "valid_for_ms": valid_for,
    }
