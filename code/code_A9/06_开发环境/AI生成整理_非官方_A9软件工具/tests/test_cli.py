from a9tools.cli import build_parser


def test_cli_has_expected_commands():
    parser = build_parser()
    args = parser.parse_args(["mission-check", "examples/mission_sample.json"])
    assert args.command == "mission-check"

