from a9tools.scan_protocol import AiEvent, EventType, TargetPoint
from a9tools.scan_state_machine import FlightState, ScanStateMachine


def test_ai_target_scan_and_map_complete_flow():
    machine = ScanStateMachine()

    machine.handle(AiEvent.next_target(TargetPoint(0.0, 0.0, 2.0, "S1")))
    machine.handle(AiEvent.scan_done("S1"))
    machine.handle(AiEvent.next_target(TargetPoint(2.0, 0.0, 2.0, "S2")))
    machine.handle(AiEvent.scan_done("S2"))
    machine.handle(AiEvent.map_complete())

    assert machine.state is FlightState.COMPLETE
    assert machine.last_scan_id == "S2"
    assert [event.type for event in machine.outbox] == [
        EventType.TARGET_REACHED,
        EventType.HOLDING,
        EventType.TARGET_REACHED,
        EventType.HOLDING,
        EventType.MAP_COMPLETE,
    ]


def test_hazard_pauses_and_replan_can_resume():
    machine = ScanStateMachine()
    machine.handle(AiEvent.next_target(TargetPoint(1.0, 0.0, 2.0, "S1")))
    machine.handle(AiEvent.hazard("前方危险区域"))

    assert machine.state is FlightState.REPLAN
    assert machine.outbox[-1].type is EventType.REQUEST_REPLAN
    assert machine.outbox[-1].data["reason"] == "前方危险区域"

    machine.handle(AiEvent.next_target(TargetPoint(1.0, -1.0, 2.0, "R1")))
    assert machine.state is FlightState.HOLD
    assert machine.current_target.scan_id == "R1"


def test_coverage_gap_creates_recovery_target():
    machine = ScanStateMachine()
    machine.handle(AiEvent.next_target(TargetPoint(0.0, 0.0, 2.0, "S1")))
    machine.handle(AiEvent.coverage_gap(TargetPoint(0.5, 0.5, 2.0, "GAP-1")))

    assert machine.state is FlightState.HOLD
    assert machine.current_target.scan_id == "GAP-1"
    assert machine.outbox[-2].type is EventType.TARGET_REACHED
    assert machine.outbox[-1].type is EventType.HOLDING


def test_critical_events_enter_failsafe():
    for event in (AiEvent.localization_lost(), AiEvent.manual_takeover(), AiEvent.link_lost()):
        machine = ScanStateMachine()
        machine.handle(event)
        assert machine.state is FlightState.FAILSAFE
        assert machine.outbox[-1].type is EventType.FAILSAFE
