from pymavlink.dialects.v10 import common as mavlink

from a9tools.simulator import SimState, encode_messages, handle_message, make_telemetry, parse_datagram


def test_telemetry_round_trip():
    messages = make_telemetry(SimState())
    parsed = parse_datagram(encode_messages(messages))
    assert [message.get_type() for message in parsed] == ["HEARTBEAT", "ATTITUDE", "GLOBAL_POSITION_INT"]


def test_command_long_gets_simulated_ack():
    command = mavlink.MAVLink_command_long_message(1, 1, mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    responses = handle_message(command)
    assert len(responses) == 1
    assert responses[0].get_type() == "COMMAND_ACK"
    assert responses[0].command == mavlink.MAV_CMD_NAV_TAKEOFF

