"""AI 端和飞行状态机的可重复离线演示。"""

from __future__ import annotations

from .scan_protocol import AiEvent, TargetPoint
from .scan_state_machine import ScanStateMachine


def run_demo() -> tuple[ScanStateMachine, list[str]]:
    machine = ScanStateMachine()
    events = [
        AiEvent.next_target(TargetPoint(0.0, 0.0, 2.0, "S1")),
        AiEvent.scan_done("S1"),
        AiEvent.next_target(TargetPoint(2.0, 0.0, 2.0, "S2")),
        AiEvent.hazard("前方疑似危险区域"),
        AiEvent.next_target(TargetPoint(2.0, -1.0, 2.0, "R1")),
        AiEvent.scan_done("R1"),
        AiEvent.coverage_gap(TargetPoint(1.0, 1.0, 2.0, "补扫-01")),
        AiEvent.scan_done("补扫-01"),
        AiEvent.map_complete(),
    ]
    trace: list[str] = []
    for event in events:
        before = machine.state.value
        machine.handle(event)
        after = machine.state.value
        trace.append(f"{before} --{event.type.value}--> {after}")
    return machine, trace

