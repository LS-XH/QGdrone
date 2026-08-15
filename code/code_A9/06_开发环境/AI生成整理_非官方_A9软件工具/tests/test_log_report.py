import pandas as pd

from a9tools.log_report import generate_report, summarize


def test_log_summary_and_report(tmp_path):
    frame = pd.DataFrame(
        {
            "timestamp": [0, 1, 2],
            "roll": [0, 2, -3],
            "pitch": [0, 1, -1],
            "yaw": [0, 10, 20],
            "lat": [23, 23.0001, 23.0002],
            "lon": [113, 113.0001, 113.0002],
            "alt": [0, 10, 5],
            "battery_voltage": [16.8, 16.5, 16.2],
            "battery_remaining": [100, 90, 80],
        }
    )
    summary = summarize(frame)
    assert summary["sample_count"] == 3
    assert summary["duration_s"] == 2.0
    frame.to_csv(tmp_path / "flight.csv", index=False)
    generated = generate_report(tmp_path / "flight.csv", tmp_path / "report")
    assert generated["distance_m"] > 0
    assert (tmp_path / "report" / "report.html").exists()
    assert (tmp_path / "report" / "summary.json").exists()
