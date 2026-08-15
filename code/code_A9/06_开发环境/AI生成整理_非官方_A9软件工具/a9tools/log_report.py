"""把导出的 A9 飞行日志 CSV 转成 HTML/PNG/CSV/JSON 报告。"""

from __future__ import annotations

import html
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def _numeric(frame: pd.DataFrame, column: str) -> pd.Series | None:
    if column not in frame:
        return None
    return pd.to_numeric(frame[column], errors="coerce")


def _distance(frame: pd.DataFrame) -> float:
    if not {"lat", "lon"}.issubset(frame.columns):
        return 0.0
    lat = pd.to_numeric(frame["lat"], errors="coerce").to_numpy(dtype=float)
    lon = pd.to_numeric(frame["lon"], errors="coerce").to_numpy(dtype=float)
    total = 0.0
    for lat1, lon1, lat2, lon2 in zip(lat[:-1], lon[:-1], lat[1:], lon[1:]):
        if any(math.isnan(value) for value in (lat1, lon1, lat2, lon2)):
            continue
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        total += 2 * 6_371_000 * math.asin(math.sqrt(min(1.0, a)))
    return total


def summarize(frame: pd.DataFrame, source: str = "") -> dict[str, Any]:
    summary: dict[str, Any] = {"source": source, "sample_count": int(len(frame)), "columns": list(frame.columns)}
    if "timestamp" in frame:
        numeric_time = pd.to_numeric(frame["timestamp"], errors="coerce")
        if numeric_time.notna().any():
            summary["duration_s"] = round(float(numeric_time.max() - numeric_time.min()), 3)
        else:
            parsed = pd.to_datetime(frame["timestamp"], errors="coerce")
            summary["duration_s"] = round(float((parsed.max() - parsed.min()).total_seconds()), 3) if parsed.notna().any() else None
    else:
        summary["duration_s"] = None
    summary["distance_m"] = round(_distance(frame), 3)

    battery = _numeric(frame, "battery_voltage")
    if battery is not None and battery.notna().any():
        summary["min_battery_voltage"] = round(float(battery.min()), 3)
    remaining = _numeric(frame, "battery_remaining")
    if remaining is not None and remaining.notna().any():
        summary["min_battery_remaining_pct"] = round(float(remaining.min()), 1)
    for axis in ("roll", "pitch", "yaw"):
        values = _numeric(frame, axis)
        if values is not None and values.notna().any():
            summary[f"max_abs_{axis}_deg"] = round(float(values.abs().max()), 3)
    altitude = _numeric(frame, "alt")
    if altitude is not None and altitude.notna().any():
        summary["min_alt_m"] = round(float(altitude.min()), 3)
        summary["max_alt_m"] = round(float(altitude.max()), 3)
    return summary


def _plot(frame: pd.DataFrame, columns: list[str], title: str, ylabel: str, path: Path) -> bool:
    available = [column for column in columns if column in frame]
    if not available:
        return False
    x = frame["timestamp"] if "timestamp" in frame else np.arange(len(frame))
    plt.figure(figsize=(10, 4.5))
    for column in available:
        plt.plot(x, pd.to_numeric(frame[column], errors="coerce"), label=column)
    plt.title(title)
    plt.xlabel("timestamp / sample")
    plt.ylabel(ylabel)
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=140)
    plt.close()
    return True


def generate_report(input_path: str | Path, output_dir: str | Path) -> dict[str, Any]:
    source = Path(input_path)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if source.suffix.lower() != ".csv":
        raise ValueError("当前版本先处理从博睿日志分析软件导出的 CSV；原始 .aclog 请先用官方分析软件导出")
    frame = pd.read_csv(source)
    summary = summarize(frame, str(source))
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    pd.DataFrame([summary]).to_csv(output / "summary.csv", index=False, encoding="utf-8-sig")
    images: list[str] = []
    for columns, title, ylabel, filename in [
        (["alt"], "Altitude", "m", "altitude.png"),
        (["roll", "pitch", "yaw"], "Attitude", "deg", "attitude.png"),
        (["battery_voltage", "battery_remaining"], "Battery", "value", "battery.png"),
    ]:
        if _plot(frame, columns, title, ylabel, output / filename):
            images.append(filename)
    cards = "".join(f"<li><b>{html.escape(str(k))}</b>: {html.escape(str(v))}</li>" for k, v in summary.items() if k not in {"columns", "source"})
    image_html = "".join(f"<h2>{html.escape(image)}</h2><img src='{html.escape(image)}' style='max-width:100%'>" for image in images)
    report = f"""<!doctype html><meta charset='utf-8'><title>A9 飞行日志报告</title>
<style>body{{font-family:Segoe UI,Microsoft YaHei,sans-serif;max-width:1100px;margin:2em auto;color:#202124}}li{{margin:.35em 0}}img{{border:1px solid #ddd}}</style>
<h1>A9 飞行日志报告</h1><p>输入：{html.escape(str(source))}</p><ul>{cards}</ul>{image_html}
<p><small>此报告只基于导出的表格字段，不能替代博睿官方日志分析软件的原始诊断结论。</small></p>"""
    (output / "report.html").write_text(report, encoding="utf-8")
    summary["report_html"] = str(output / "report.html")
    summary["output_dir"] = str(output)
    return summary

