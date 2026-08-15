"""AI 组与嵌入式扫描状态机之间的离线协议模型。

这是项目内部的 Python 联调模型，不是 A9 厂家官方协议，也不直接连接串口。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class EventType(str, Enum):
    NEXT_TARGET = "NEXT_TARGET"
    SCAN_DONE = "SCAN_DONE"
    COVERAGE_GAP = "COVERAGE_GAP"
    HAZARD = "HAZARD"
    MAP_COMPLETE = "MAP_COMPLETE"
    LOCALIZATION_LOST = "LOCALIZATION_LOST"
    MANUAL_TAKEOVER = "MANUAL_TAKEOVER"
    LINK_LOST = "LINK_LOST"
    TARGET_REACHED = "TARGET_REACHED"
    HOLDING = "HOLDING"
    REQUEST_REPLAN = "REQUEST_REPLAN"
    FAILSAFE = "FAILSAFE"


@dataclass(frozen=True)
class TargetPoint:
    x: float
    y: float
    z: float
    scan_id: str


@dataclass(frozen=True)
class AiEvent:
    type: EventType
    data: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def next_target(target: TargetPoint) -> "AiEvent":
        return AiEvent(EventType.NEXT_TARGET, {"target": target})

    @staticmethod
    def scan_done(scan_id: str) -> "AiEvent":
        return AiEvent(EventType.SCAN_DONE, {"scan_id": scan_id})

    @staticmethod
    def coverage_gap(target: TargetPoint) -> "AiEvent":
        return AiEvent(EventType.COVERAGE_GAP, {"target": target})

    @staticmethod
    def hazard(reason: str) -> "AiEvent":
        return AiEvent(EventType.HAZARD, {"reason": reason})

    @staticmethod
    def map_complete() -> "AiEvent":
        return AiEvent(EventType.MAP_COMPLETE)

    @staticmethod
    def localization_lost() -> "AiEvent":
        return AiEvent(EventType.LOCALIZATION_LOST)

    @staticmethod
    def manual_takeover() -> "AiEvent":
        return AiEvent(EventType.MANUAL_TAKEOVER)

    @staticmethod
    def link_lost() -> "AiEvent":
        return AiEvent(EventType.LINK_LOST)


def output_event(event_type: EventType, **data: Any) -> AiEvent:
    return AiEvent(event_type, data)

