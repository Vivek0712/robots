"""Behavior tests for the AckermannRosRobot mesh bridge.

The bridge presents an Ackermann-steering ROS 2 car (reference platform: AWS
DeepRacer) as a strands robot. Like ``test_ros_bridge.py``, every test runs
rclpy-free: the module's ``use_ros`` reference is monkeypatched with a
recorder, so the forwarding contract, the bicycle-model conversion, the
enable handshake, and the safety behaviors are all exercised with no ROS 2
installed.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from strands_robots.mesh import ackermann_robot as ack_mod
from strands_robots.mesh.ackermann_robot import _twist_to_servo

# Bicycle model ---------------------------------------------------------------

_LIMITS = {"wheelbase_m": 0.164, "max_speed": 1.5, "max_steering_rad": 0.5236}


def test_model_straight_full_speed() -> None:
    angle, throttle = _twist_to_servo(1.5, 0.0, **_LIMITS)
    assert angle == 0.0
    assert throttle == 1.0


def test_model_rest_is_zero() -> None:
    # Below the rest epsilon the command maps to zeros: an Ackermann platform
    # cannot yaw at rest, and atan2 would amplify noise near v = 0.
    assert _twist_to_servo(0.0, 2.0, **_LIMITS) == (0.0, 0.0)
    assert _twist_to_servo(5e-4, 2.0, **_LIMITS) == (0.0, 0.0)


def test_model_left_turn_matches_hand_computation() -> None:
    angle, throttle = _twist_to_servo(1.0, 2.0, **_LIMITS)
    expected_delta = math.atan2(0.164 * 2.0, 1.0)  # ~0.317 rad, inside the clamp
    assert angle == pytest.approx(expected_delta / 0.5236)
    assert throttle == pytest.approx(1.0 / 1.5)
    assert angle > 0  # positive angular -> positive (left) steering


def test_model_right_turn_is_negative() -> None:
    angle, _ = _twist_to_servo(1.0, -2.0, **_LIMITS)
    assert angle == pytest.approx(-math.atan2(0.164 * 2.0, 1.0) / 0.5236)


def test_model_steering_clamps_to_limit() -> None:
    # Huge angular rate: delta saturates at max_steering_rad -> angle_norm +/-1.
    angle, _ = _twist_to_servo(0.1, 50.0, **_LIMITS)
    assert angle == pytest.approx(1.0)
    angle, _ = _twist_to_servo(0.1, -50.0, **_LIMITS)
    assert angle == pytest.approx(-1.0)


def test_model_throttle_clamps_beyond_max_speed() -> None:
    _, throttle = _twist_to_servo(4.0, 0.0, **_LIMITS)
    assert throttle == 1.0
    _, throttle = _twist_to_servo(-4.0, 0.0, **_LIMITS)
    assert throttle == -1.0


def test_model_reverse_steering_geometry() -> None:
    # Reversing with positive angular: atan2 handles the quadrant (delta flips
    # sign relative to forward motion), matching real Ackermann behavior.
    fwd_angle, _ = _twist_to_servo(1.0, 2.0, **_LIMITS)
    rev_angle, rev_throttle = _twist_to_servo(-1.0, 2.0, **_LIMITS)
    assert rev_throttle == pytest.approx(-1.0 / 1.5)
    assert rev_angle == pytest.approx(-fwd_angle)


# Recorder --------------------------------------------------------------------


class _Recorder:
    """Stand-in for use_ros: records calls, returns scripted results."""

    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []
        self.responses: list[dict[str, Any]] = []

    def __call__(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(kwargs)
        if self.responses:
            return self.responses.pop(0)
        return {"status": "success", "content": [{"text": "ok"}]}


@pytest.fixture
def rec(monkeypatch: pytest.MonkeyPatch) -> _Recorder:
    recorder = _Recorder()
    monkeypatch.setattr(ack_mod, "use_ros", recorder)
    return recorder


def _car(**overrides: Any) -> ack_mod.AckermannRosRobot:
    kwargs: dict[str, Any] = {"node_name": "car", "servo_topic": "/servo"}
    kwargs.update(overrides)
    return ack_mod.AckermannRosRobot(**kwargs)


_HANDSHAKE = [
    {
        "service": "/ctrl_pkg/vehicle_state",
        "type": "deepracer_interfaces_pkg/srv/ActiveStateSrv",
        "fields": {"state": 1},
    },
    {
        "service": "/ctrl_pkg/enable_state",
        "type": "deepracer_interfaces_pkg/srv/EnableStateSrv",
        "fields": {"is_active": True},
    },
]


# Construction ----------------------------------------------------------------


def test_invalid_names_rejected_at_construction() -> None:
    with pytest.raises(ValueError, match="invalid servo_topic"):
        _car(servo_topic="/servo topic")
    with pytest.raises(ValueError, match="invalid node_name"):
        _car(node_name="car; rm")
    with pytest.raises(ValueError, match="invalid init_services service"):
        _car(init_services=[{"service": "/bad svc", "type": "a/srv/B", "fields": {}}])


def test_init_service_missing_type_rejected_at_construction() -> None:
    # enable() indexes item["type"]; a missing type must fail loudly at
    # construction, not as a KeyError escaping enable() later.
    with pytest.raises(ValueError, match="missing its 'type'"):
        _car(init_services=[{"service": "/ctrl_pkg/vehicle_state", "fields": {"state": 1}}])


@pytest.mark.parametrize("field", ["wheelbase_m", "max_speed", "max_steering_rad", "max_duration", "publish_rate"])
def test_nonpositive_numerics_rejected(field: str) -> None:
    with pytest.raises(ValueError, match=field):
        _car(**{field: 0.0})


def test_from_deepracer_wiring_pinned() -> None:
    car = ack_mod.AckermannRosRobot.from_deepracer(node_name="dr")
    assert car.servo_topic == "/webserver_pkg/manual_drive"
    assert car.servo_type == "deepracer_interfaces_pkg/msg/ServoCtrlMsg"
    assert car.scan_topic == "/rplidar_ros/scan"
    assert car.wheelbase_m == 0.164
    assert car.max_speed == 1.5
    assert car.max_steering_rad == 0.5236
    assert car.publish_rate == 20.0
    assert car.init_services == _HANDSHAKE


def test_from_deepracer_accepts_overrides() -> None:
    car = ack_mod.AckermannRosRobot.from_deepracer(node_name="dr", max_speed=0.8, scan_topic=None)
    assert car.max_speed == 0.8
    assert car.scan_topic is None
    assert car.servo_topic == "/webserver_pkg/manual_drive"  # untouched default


# enable ----------------------------------------------------------------------


def test_enable_calls_init_services_in_order(rec: _Recorder) -> None:
    car = _car(init_services=_HANDSHAKE)
    result = car.enable()
    assert result["status"] == "success"
    assert [c["action"] for c in rec.calls] == ["service_call", "service_call"]
    assert rec.calls[0]["service"] == "/ctrl_pkg/vehicle_state"
    assert rec.calls[0]["type"] == "deepracer_interfaces_pkg/srv/ActiveStateSrv"
    assert rec.calls[0]["fields"] == {"state": 1}
    assert rec.calls[1]["service"] == "/ctrl_pkg/enable_state"
    assert rec.calls[1]["fields"] == {"is_active": True}


def test_enable_is_idempotent(rec: _Recorder) -> None:
    car = _car(init_services=_HANDSHAKE)
    car.enable()
    car.enable()
    assert len(rec.calls) == 2  # handshake ran once, not twice


def test_enable_failure_surfaces_and_does_not_latch(rec: _Recorder) -> None:
    car = _car(init_services=_HANDSHAKE)
    failure = {"status": "error", "content": [{"text": "use_ros: service not available"}]}
    rec.responses = [failure]
    result = car.enable()
    assert result is failure
    assert len(rec.calls) == 1  # stopped at the first failing call
    # A later retry runs the handshake again from the start.
    result = car.enable()
    assert result["status"] == "success"
    assert len(rec.calls) == 3


def test_enable_without_init_services_is_success(rec: _Recorder) -> None:
    result = _car().enable()
    assert result["status"] == "success"
    assert rec.calls == []


# stop / get_scan ---------------------------------------------------------------


def test_stop_publishes_zero_and_needs_no_enable(rec: _Recorder) -> None:
    car = _car(init_services=_HANDSHAKE)
    result = car.stop()
    assert result["status"] == "success"
    assert len(rec.calls) == 1  # no handshake - stopping must never be gated
    call = rec.calls[0]
    assert call["action"] == "publish"
    assert call["topic"] == "/servo"
    assert call["fields"] == {"angle": 0.0, "throttle": 0.0}
    assert call["count"] == 1


def test_get_scan_without_topic_is_error(rec: _Recorder) -> None:
    result = _car().get_scan()
    assert result["status"] == "error"
    assert "no scan_topic" in result["content"][0]["text"]
    assert rec.calls == []


def test_get_scan_forwards_echo(rec: _Recorder) -> None:
    car = _car(scan_topic="/scan")
    car.get_scan(timeout=3.0)
    call = rec.calls[0]
    assert call["action"] == "echo"
    assert call["topic"] == "/scan"
    assert call["timeout"] == 3.0
    assert call["count"] == 1
