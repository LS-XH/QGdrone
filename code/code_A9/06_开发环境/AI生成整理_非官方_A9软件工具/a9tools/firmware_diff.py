"""A9 源码目录和 HEX 文件的离线差异检查。"""

from __future__ import annotations

import hashlib
import html
import json
import re
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _files(root: Path) -> dict[str, Path]:
    return {path.relative_to(root).as_posix(): path for path in root.rglob("*") if path.is_file()}


def _version_from_text(text: str) -> str | None:
    patterns = [
        r"Firmware_Version\s*=\s*\"([^\"]+)\"",
        r"FIRMWARE_VERSION\s*[^\n]*?\"([^\"]+)\"",
        r"Firmware_Version\s*[:=]\s*([0-9][^\s;]*)",
    ]
    for pattern in patterns:
        match = re.search(pattern, text, flags=re.IGNORECASE)
        if match:
            return match.group(1)
    return None


def extract_version(root: Path) -> str | None:
    candidates = list(root.rglob("*.cpp")) + list(root.rglob("*.h")) + list(root.rglob("*.c"))
    for path in candidates:
        try:
            version = _version_from_text(path.read_text(encoding="utf-8", errors="ignore"))
        except OSError:
            continue
        if version:
            return version
    return None


def intel_hex_range(path: str | Path) -> dict[str, int | None]:
    """读取 HEX 的数据地址范围；只分析，不写入芯片。"""
    base = 0
    minimum: int | None = None
    maximum: int | None = None
    for line_number, line in enumerate(Path(path).read_text(encoding="ascii", errors="ignore").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"第 {line_number} 行不是 Intel HEX")
        raw = bytes.fromhex(line[1:])
        length, address, record_type = raw[0], int.from_bytes(raw[1:3], "big"), raw[3]
        if (sum(raw) & 0xFF) != 0:
            raise ValueError(f"第 {line_number} 行校验和错误")
        data = raw[4 : 4 + length]
        if record_type == 0:
            start = base + address
            end = start + len(data) - 1
            minimum = start if minimum is None else min(minimum, start)
            maximum = end if maximum is None else max(maximum, end)
        elif record_type == 2 and len(data) == 2:
            base = int.from_bytes(data, "big") << 4
        elif record_type == 4 and len(data) == 2:
            base = int.from_bytes(data, "big") << 16
    return {"min_address": minimum, "max_address": maximum}


def compare_sources(left: str | Path, right: str | Path) -> dict[str, Any]:
    left_root, right_root = Path(left), Path(right)
    if not left_root.is_dir() or not right_root.is_dir():
        raise ValueError("源码差异工具需要两个已解压的源码目录")
    left_files, right_files = _files(left_root), _files(right_root)
    added = sorted(set(right_files) - set(left_files))
    deleted = sorted(set(left_files) - set(right_files))
    modified = sorted(name for name in set(left_files) & set(right_files) if sha256(left_files[name]) != sha256(right_files[name]))
    return {
        "left": str(left_root),
        "right": str(right_root),
        "left_version": extract_version(left_root),
        "right_version": extract_version(right_root),
        "left_file_count": len(left_files),
        "right_file_count": len(right_files),
        "added": added,
        "deleted": deleted,
        "modified": modified,
    }


def compare_hex(left: str | Path, right: str | Path) -> dict[str, Any]:
    left_path, right_path = Path(left), Path(right)
    return {
        "left": str(left_path),
        "right": str(right_path),
        "left_sha256": sha256(left_path),
        "right_sha256": sha256(right_path),
        "same": sha256(left_path) == sha256(right_path),
        "left_range": intel_hex_range(left_path),
        "right_range": intel_hex_range(right_path),
    }


def render_html(result: dict[str, Any], output: str | Path) -> Path:
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for key in ("added", "deleted", "modified"):
        for name in result.get(key, []):
            rows.append(f"<tr><td>{html.escape(key)}</td><td>{html.escape(name)}</td></tr>")
    rows_html = "".join(rows) or "<tr><td colspan='2'>没有文件差异</td></tr>"
    text = json.dumps(result, ensure_ascii=False, indent=2)
    content = f"""<!doctype html><meta charset='utf-8'><title>A9 版本差异</title>
<style>body{{font-family:Segoe UI,Microsoft YaHei,sans-serif;max-width:1100px;margin:2em auto}}td,th{{border:1px solid #ddd;padding:7px;text-align:left}}table{{border-collapse:collapse;width:100%}}pre{{background:#f5f5f5;padding:1em;overflow:auto}}</style>
<h1>A9 源码/固件差异报告</h1><p>左版本：{html.escape(str(result.get('left_version')))}　右版本：{html.escape(str(result.get('right_version')))}</p>
<table><tr><th>类型</th><th>文件</th></tr>{rows_html}</table><h2>原始 JSON</h2><pre>{html.escape(text)}</pre>
<p><small>差异报告不会判断改动是否安全，也不会执行编译或烧录。</small></p>"""
    destination.write_text(content, encoding="utf-8")
    return destination

