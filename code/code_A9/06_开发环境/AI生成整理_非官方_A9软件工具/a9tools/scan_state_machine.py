"""嵌入式组扫描任务状态机的离线模型。"""

from __future__ import annotations

from enum import Enum

from .scan_protocol import AiEvent, EventType, output_event, TargetPoint


class FlightState(str, Enum):
    IDLE = "待机"
    FLY_TO_TARGET = "前往目标"
    HOLD = "悬停等待扫描"
    REPLAN = "等待重新规划"
    COMPLETE = "任务完成"
    FAILSAFE = "保护状态"


class ScanStateMachine:
    """模拟 AI 事件进入嵌入式任务层后的安全状态转换。

    真实工程中，outbox 对应发送给 AI/上位机的状态事件；
    飞行动作本身仍应交给 A9 现有控制链路执行。
    """

    def __init__(self) -> None:
        self.state = FlightState.IDLE
        self.current_target: TargetPoint | None = None
        self.last_scan_id: str | None = None
        self.outbox: list[AiEvent] = []

    def _emit(self, event_type: EventType, **data: object) -> None:
        self.outbox.append(output_event(event_type, **data))

    def _arrive(self, target: TargetPoint) -> None:
        self.current_target = target
        self.state = FlightState.FLY_TO_TARGET
        self.state = FlightState.HOLD
        self._emit(EventType.TARGET_REACHED, scan_id=target.scan_id, x=target.x, y=target.y, z=target.z)
        self._emit(EventType.HOLDING, scan_id=target.scan_id)

    def handle(self, event: AiEvent) -> None:
        critical = {EventType.LOCALIZATION_LOST, EventType.MANUAL_TAKEOVER, EventType.LINK_LOST}
        if event.type in critical:
            self.state = FlightState.FAILSAFE
            self._emit(EventType.FAILSAFE, reason=event.type.value)
            return

        if event.type is EventType.HAZARD:
            self.state = FlightState.FAILSAFE
            self._emit(EventType.FAILSAFE, reason=event.data.get("reason", "AI报告危险"))
            self.state = FlightState.REPLAN
            self._emit(EventType.REQUEST_REPLAN, reason=event.data.get("reason", "AI报告危险"))
            return

        if event.type is EventType.NEXT_TARGET:
            if self.state in {FlightState.IDLE, FlightState.HOLD, FlightState.REPLAN}:
                self._arrive(event.data["target"])
            return

        if event.type is EventType.COVERAGE_GAP:
            if self.state in {FlightState.HOLD, FlightState.REPLAN}:
                self._arrive(event.data["target"])
            return

        if event.type is EventType.SCAN_DONE:
            if self.state is FlightState.HOLD:
                self.last_scan_id = str(event.data["scan_id"])
            return

        if event.type is EventType.MAP_COMPLETE:
            if self.state in {FlightState.HOLD, FlightState.REPLAN}:
                self.state = FlightState.COMPLETE
                self._emit(EventType.MAP_COMPLETE)

