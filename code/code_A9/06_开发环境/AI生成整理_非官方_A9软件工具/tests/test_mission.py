from a9tools.mission import check_mission, haversine_m


def test_valid_mission_passes():
    result = check_mission({"mission": [{"command": "TAKEOFF", "lat": 23, "lon": 113, "alt": 10}, {"command": "LAND", "lat": 23.001, "lon": 113.001, "alt": 0}]})
    assert result["ok"]
    assert result["distance_m"] > 0


def test_invalid_mission_reports_safety_errors():
    result = check_mission({"mission": [{"command": "WAYPOINT", "lat": 99, "lon": 113, "alt": 200, "speed": 30}, {"command": "PWM", "channel": 20, "value": 70000}]})
    assert not result["ok"]
    codes = {issue["code"] for issue in result["issues"]}
    assert {"COORDINATE", "ALTITUDE", "SPEED", "PWM_CHANNEL", "PWM_VALUE"} <= codes


def test_haversine_zero():
    assert haversine_m(23, 113, 23, 113) == 0

