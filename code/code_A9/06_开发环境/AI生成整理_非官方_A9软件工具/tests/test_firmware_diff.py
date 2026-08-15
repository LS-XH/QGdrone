from pathlib import Path

from a9tools.firmware_diff import compare_sources, intel_hex_range


def test_source_diff_and_version(tmp_path):
    left = tmp_path / "left"
    right = tmp_path / "right"
    (left / "Main").mkdir(parents=True)
    (right / "Main").mkdir(parents=True)
    (left / "Main" / "main.cpp").write_text('const char* Firmware_Version = "1.0";', encoding="utf-8")
    (right / "Main" / "main.cpp").write_text('const char* Firmware_Version = "1.1";', encoding="utf-8")
    (right / "new.c").write_text("new", encoding="utf-8")
    result = compare_sources(left, right)
    assert result["left_version"] == "1.0"
    assert result["right_version"] == "1.1"
    assert "new.c" in result["added"]
    assert "Main/main.cpp" in result["modified"]


def test_intel_hex_range(tmp_path):
    # :020000040804EE sets the upper address to 0x08040000;
    # :0400000001020304F2 writes four bytes at that address.
    path = tmp_path / "sample.hex"
    path.write_text(":020000040804EE\n:0400000001020304F2\n:00000001FF\n", encoding="ascii")
    result = intel_hex_range(path)
    assert result == {"min_address": 0x08040000, "max_address": 0x08040003}

