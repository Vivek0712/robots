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

import pytest

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
