"""a9tools 命令行入口。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .firmware_diff import compare_hex, compare_sources, render_html as render_diff_html
from .log_report import generate_report
from .mission import load_and_check, render_html as render_mission_html
from .scan_demo import run_demo
from .simulator import run_udp_simulator


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="A9 离线学习与工程辅助工具（不连接真实飞控）")
    sub = parser.add_subparsers(dest="command", required=True)

    simulate = sub.add_parser("simulate", help="启动本机 MAVLink UDP 遥测仿真器")
    simulate.add_argument("--host", default="127.0.0.1")
    simulate.add_argument("--port", type=int, default=14560)
    simulate.add_argument("--rate", type=float, default=2.0, help="遥测频率 Hz")
    simulate.add_argument("--duration", type=float, default=0.0, help="运行秒数，0 为持续运行")

    mission = sub.add_parser("mission-check", help="检查任务 JSON 并生成 HTML 报告")
    mission.add_argument("input", type=Path)
    mission.add_argument("--html", type=Path, default=Path("reports/mission_report.html"))

    log = sub.add_parser("log-report", help="把导出的 CSV 生成飞行日志报告")
    log.add_argument("input", type=Path)
    log.add_argument("--out", type=Path, default=Path("reports/flight_report"))

    diff = sub.add_parser("firmware-diff", help="比较两个已解压源码目录，或两个 HEX 文件")
    diff.add_argument("left", type=Path)
    diff.add_argument("right", type=Path)
    diff.add_argument("--html", type=Path, default=Path("reports/firmware_diff.html"))

    sub.add_parser("self-test", help="运行本工具单元测试")
    sub.add_parser("scan-demo", help="运行 AI 事件与飞行状态机离线演示")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "simulate":
        run_udp_simulator(args.host, args.port, args.rate, args.duration)
        return 0
    if args.command == "mission-check":
        result = load_and_check(args.input)
        render_mission_html(result, args.html)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"HTML 报告：{args.html}")
        return 0 if result["ok"] else 2
    if args.command == "log-report":
        result = generate_report(args.input, args.out)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"HTML 报告：{args.out / 'report.html'}")
        return 0
    if args.command == "firmware-diff":
        if args.left.suffix.lower() == ".hex" and args.right.suffix.lower() == ".hex":
            result = compare_hex(args.left, args.right)
        else:
            result = compare_sources(args.left, args.right)
        render_diff_html(result, args.html)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print(f"HTML 报告：{args.html}")
        return 0
    if args.command == "self-test":
        import pytest

        # 当前 Windows 环境下 pytest 的 cacheprovider 可能在退出时等待缓存锁；
        # 测试本身不依赖缓存，因此关闭它让一键测试稳定退出；同时把临时目录
        # 放在项目盘，避免受系统临时目录权限影响。
        base_temp = str((Path("reports") / ".pytest-tmp").resolve())
        return int(pytest.main(["-q", "-p", "no:cacheprovider", "--basetemp", base_temp, "tests"]))
    if args.command == "scan-demo":
        machine, trace = run_demo()
        print("--- AI to FlightStateMachine simulation ---")
        for index, line in enumerate(trace, 1):
            print(f"{index:02d}. {line}")
        print("--- RESULT ---")
        print(f"final_state={machine.state.value}")
        print(f"last_scan={machine.last_scan_id}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
