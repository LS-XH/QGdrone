"""生成 VS V1 的目标点和 ACK 示例帧；只做离线编码，不连接飞控。"""

from a9tools.vs_protocol import (
    Frame,
    FrameFlags,
    MessageType,
    encode_frame,
    pack_ack,
    pack_hazard_alert,
    pack_set_target_local,
)


def main() -> None:
    target = encode_frame(
        Frame(
            MessageType.SET_TARGET_LOCAL,
            FrameFlags.ACK_REQUIRED,
            sequence=1,
            timestamp_ms=1000,
            payload=pack_set_target_local(7, 1.25, -2.5, 3.0, 90.0, 0.8, 0.4, 1500),
        )
    )
    ack = encode_frame(
        Frame(
            MessageType.ACK,
            FrameFlags.IS_RESPONSE,
            sequence=2,
            timestamp_ms=1100,
            payload=pack_ack(1, MessageType.SET_TARGET_LOCAL, 0, 0),
        )
    )
    hazard = encode_frame(
        Frame(
            MessageType.HAZARD_ALERT,
            FrameFlags.ACK_REQUIRED | FrameFlags.HIGH_PRIORITY,
            sequence=3,
            timestamp_ms=1200,
            payload=pack_hazard_alert(3, 1.0, -2.0, 0.5, 2.5, 3, 4, 5000),
        )
    )
    print("SET_TARGET_LOCAL:", target.hex(" "))
    print("ACK:", ack.hex(" "))
    print("HAZARD_ALERT:", hazard.hex(" "))


if __name__ == "__main__":
    main()
