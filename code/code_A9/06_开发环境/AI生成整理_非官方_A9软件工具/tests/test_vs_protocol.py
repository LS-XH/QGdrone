import struct

import pytest

from a9tools.vs_protocol import (
    AckResult,
    Frame,
    FrameFlags,
    MessageType,
    ProtocolError,
    decode_frames,
    encode_frame,
    pack_ack,
    pack_set_target_local,
    unpack_ack,
    unpack_set_target_local,
)


def test_set_target_round_trip_uses_little_endian_and_v1_header():
    payload = pack_set_target_local(7, 1.25, -2.5, 3.0, 90.0, 0.8, 0.4, 1500)
    raw = encode_frame(Frame(MessageType.SET_TARGET_LOCAL, FrameFlags.ACK_REQUIRED, 12, 100, payload))
    assert raw[:2] == b"VS"
    assert raw[2] == 1
    assert raw[4:8] == struct.pack("<HH", MessageType.SET_TARGET_LOCAL, len(payload))
    frames = decode_frames(raw)
    assert len(frames) == 1
    assert frames[0].sequence == 12
    assert unpack_set_target_local(frames[0].payload)["target_id"] == 7


def test_crc_error_is_rejected():
    raw = bytearray(encode_frame(Frame(MessageType.HEARTBEAT, 0, 1, 2, b"")))
    raw[-1] ^= 0xFF
    with pytest.raises(ProtocolError, match="CRC"):
        decode_frames(bytes(raw))


def test_stream_decoder_resynchronizes_after_noise_and_truncated_frame():
    first = encode_frame(Frame(MessageType.HEARTBEAT, 0, 1, 2, b"abc"))
    second = encode_frame(Frame(MessageType.ACK, FrameFlags.IS_RESPONSE, 2, 3, b"ack"))
    assert [frame.sequence for frame in decode_frames(b"noise" + first + second)] == [1, 2]


def test_payload_length_and_non_finite_float_are_rejected():
    with pytest.raises(ValueError, match="payload"):
        encode_frame(Frame(MessageType.HEARTBEAT, 0, 1, 2, b"x" * 129))
    with pytest.raises(ValueError, match="finite"):
        pack_set_target_local(1, float("nan"), 0, 0, 0, 1, 1, 1000)


def test_ack_payload_round_trip():
    payload = pack_ack(12, MessageType.SET_TARGET_LOCAL, AckResult.ACCEPTED, 0)
    assert unpack_ack(payload) == {
        "acked_sequence": 12,
        "acked_type": MessageType.SET_TARGET_LOCAL,
        "result": AckResult.ACCEPTED,
        "reason": 0,
    }


def test_hazard_alert_payload_round_trip():
    from a9tools.vs_protocol import pack_hazard_alert, unpack_hazard_alert

    payload = pack_hazard_alert(3, 1.0, -2.0, 0.5, 2.5, 3, 4, 5000)
    assert unpack_hazard_alert(payload) == {
        "hazard_id": 3,
        "x_m": 1.0,
        "y_m": -2.0,
        "z_m": 0.5,
        "radius_m": 2.5,
        "level": 3,
        "suggested_action": 4,
        "valid_for_ms": 5000,
    }
