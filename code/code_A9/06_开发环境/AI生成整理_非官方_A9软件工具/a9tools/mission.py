"""A9 任务/航线离线预检查。"""

from __future__ import annotations

import html
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class MissionPolicy:
    """项目预检查策略，不等同于 A9 固件的全部硬限制。"""

    max_points: int = 60_000
    max_altitude_m: float = 120.0
    max_speed_m_s: float = 15.0


SUPPORTED = {"TAKEOFF", "WAYPOINT", "LAND", "RTL", "RTH", "HOVER", "PWM"}


def _issue(level: str, code: str, message: str, index: int | None = None) -> dict[str, Any]:
    return {"level": level, "code": code, "message": message, "index": index}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    value = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(min(1.0, value)))


def _in_fence(point: dict[str, Any], fence: dict[str, Any]) -> bool:
    return (
        fence["min_lat"] <= point["lat"] <= fence["max_lat"]
        and fence["min_lon"] <= point["lon"] <= fence["max_lon"]
    )


def check_mission(document: dict[str, Any], policy: MissionPolicy | None = None) -> dict[str, Any]:
    policy = policy or MissionPolicy()
    issues: list[dict[str, Any]] = []
    items = document.get("mission", document.get("items"))
    if not isinstance(items, list):
        return {"ok": False, "points": 0, "distance_m": 0.0, "issues": [_issue("ERROR", "MISSION_TYPE", "mission 必须是数组")], "items": []}

    if len(items) > policy.max_points:
        issues.append(_issue("ERROR", "POINT_LIMIT", f"任务点数量 {len(items)} 超过预检查上限 {policy.max_points}"))

    fence = document.get("geofence")
    if fence is not None:
        required = {"min_lat", "max_lat", "min_lon", "max_lon"}
        if not isinstance(fence, dict) or not required.issubset(fence):
            issues.append(_issue("ERROR", "FENCE_FORMAT", "geofence 缺少 min_lat/max_lat/min_lon/max_lon"))
            fence = None

    total_distance = 0.0
    previous: dict[str, Any] | None = None
    normalized: list[dict[str, Any]] = []
    for index, raw in enumerate(items):
        if not isinstance(raw, dict):
            issues.append(_issue("ERROR", "ITEM_TYPE", "任务点必须是对象", index))
            continue
        item = {str(k): v for k, v in raw.items()}
        command = str(item.get("command", item.get("type", ""))).upper()
        item["command"] = command
        normalized.append(item)
        if command not in SUPPORTED:
            issues.append(_issue("ERROR", "COMMAND", f"不支持的任务指令：{command or '<空>'}", index))

        has_position = all(key in item for key in ("lat", "lon"))
        if has_position:
            try:
                lat, lon = float(item["lat"]), float(item["lon"])
                item["lat"], item["lon"] = lat, lon
                if not -90 <= lat <= 90 or not -180 <= lon <= 180:
                    issues.append(_issue("ERROR", "COORDINATE", "经纬度超出范围", index))
                if fence and not _in_fence(item, fence):
                    issues.append(_issue("ERROR", "GEOFENCE", "任务点超出 geofence", index))
                if previous is not None:
                    distance = haversine_m(previous["lat"], previous["lon"], lat, lon)
                    total_distance += distance
                    if distance < 0.01:
                        issues.append(_issue("WARN", "DUPLICATE", "与上一个坐标几乎重复", index))
                previous = item
            except (TypeError, ValueError):
                issues.append(_issue("ERROR", "COORDINATE", "经纬度必须是数字", index))
        elif command in {"TAKEOFF", "WAYPOINT", "LAND"}:
            issues.append(_issue("ERROR", "COORDINATE", f"{command} 缺少 lat/lon", index))

        if "alt" in item:
            try:
                altitude = float(item["alt"])
                if altitude < 0:
                    issues.append(_issue("ERROR", "ALTITUDE", "高度不能为负数", index))
                if altitude > policy.max_altitude_m:
                    issues.append(_issue("ERROR", "ALTITUDE", f"高度超过预检查上限 {policy.max_altitude_m} m", index))
            except (TypeError, ValueError):
                issues.append(_issue("ERROR", "ALTITUDE", "alt 必须是数字", index))

        if "speed" in item:
            try:
                speed = float(item["speed"])
                if speed <= 0 or speed > policy.max_speed_m_s:
                    issues.append(_issue("ERROR", "SPEED", f"速度应在 (0, {policy.max_speed_m_s}] m/s 内", index))
            except (TypeError, ValueError):
                issues.append(_issue("ERROR", "SPEED", "speed 必须是数字", index))

        if command == "PWM":
            try:
                channel, value = int(item["channel"]), int(item["value"])
                if not 1 <= channel <= 14:
                    issues.append(_issue("ERROR", "PWM_CHANNEL", "PWM 通道应为 1..14", index))
                if not 0 <= value <= 65000:
                    issues.append(_issue("ERROR", "PWM_VALUE", "PWM 值应为 0..65000", index))
            except (KeyError, TypeError, ValueError):
                issues.append(_issue("ERROR", "PWM_FORMAT", "PWM 需要数字 channel 和 value", index))

    errors = sum(issue["level"] == "ERROR" for issue in issues)
    warnings = sum(issue["level"] == "WARN" for issue in issues)
    return {
        "ok": errors == 0,
        "points": len(items),
        "distance_m": round(total_distance, 3),
        "errors": errors,
        "warnings": warnings,
        "issues": issues,
        "items": normalized,
    }


def load_and_check(path: str | Path, policy: MissionPolicy | None = None) -> dict[str, Any]:
    source = Path(path)
    document = json.loads(source.read_text(encoding="utf-8-sig"))
    result = check_mission(document, policy)
    result["source"] = str(source)
    return result


def render_html(result: dict[str, Any], output: str | Path) -> Path:
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = "".join(
        f"<tr><td>{html.escape(str(issue.get('level')))}</td><td>{html.escape(str(issue.get('index', '')))}</td><td>{html.escape(str(issue.get('code')))}</td><td>{html.escape(str(issue.get('message')))}</td></tr>"
        for issue in result["issues"]
    ) or "<tr><td colspan='4'>未发现问题</td></tr>"
    status = "通过" if result["ok"] else "不通过"
    content = f"""<!doctype html><meta charset='utf-8'><title>A9 任务预检查</title>
<style>body{{font-family:Segoe UI,Microsoft YaHei,sans-serif;max-width:1000px;margin:2em auto;color:#202124}}table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #ddd;padding:8px}}th{{background:#f3f6f9}}.ok{{color:#087f23}}.bad{{color:#b3261e}}</style>
<h1>A9 任务/航线预检查</h1><h2 class='{'ok' if result['ok'] else 'bad'}'>{status}</h2>
<p>任务点：{result['points']}　累计航线距离：{result['distance_m']} m　错误：{result['errors']}　警告：{result['warnings']}</p>
<table><tr><th>级别</th><th>序号</th><th>代码</th><th>说明</th></tr>{rows}</table>
<p><small>此报告是项目侧离线预检查，不替代 A9 地面站、固件和现场安全检查。</small></p>"""
    destination.write_text(content, encoding="utf-8")
    return destination

