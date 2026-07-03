"""Ackermann ROS 2 mesh bridge - steering-geometry cars as strands robots.

An :class:`AckermannRosRobot` wraps an Ackermann-steering ROS 2 car (reference
platform: AWS DeepRacer) so an agent can drive it with the same
``Agent(tools=robot.tools)`` pattern as every other strands robot. Ackermann
platforms differ from the differential-drive bases served by
:class:`~strands_robots.mesh.ros_bridge.RosBridgedRobot` in three ways this
class absorbs:

1. Commands are normalized servo pairs (``angle``/``throttle`` in [-1, 1]),
   not ``geometry_msgs/msg/Twist``. :meth:`drive` keeps the fleet-wide
   ``(linear, angular)`` contract and converts internally with a bicycle
   model, so the agent never learns a second vocabulary.
2. The vehicle must be switched into a commandable state first (the DeepRacer
   needs a two-step manual-mode service handshake). ``init_services`` declares
   that handshake; :meth:`drive` runs it once, automatically, before the first
   command and refuses to drive if it fails.
3. There is no odometry topic on the stock platform, so there is deliberately
   no ``get_pose`` here.

All ROS 2 I/O forwards through :func:`strands_robots.tools.use_ros.use_ros`,
so the bridge owns no rclpy state and is safe to construct without ROS 2.

Typical usage::

    from strands import Agent
    from strands_robots.mesh import AckermannRosRobot

    car = AckermannRosRobot.from_deepracer(node_name="deepracer")
    car.drive(linear=0.5, duration=2.0)   # forward half a metre per second
    print(car.get_scan())

    agent = Agent(tools=car.tools)
    agent("drive forward slowly for two seconds, then read the lidar")
"""

from __future__ import annotations

import math

_SERVO_TYPE = "deepracer_interfaces_pkg/msg/ServoCtrlMsg"

# Below this absolute linear velocity a command is treated as rest and maps to
# zero output: a stock Ackermann platform cannot yaw at rest, and the bicycle
# model's atan2 would otherwise amplify noise near v = 0 into hard steering.
_REST_VELOCITY_EPS = 1e-3


def _twist_to_servo(
    linear: float,
    angular: float,
    *,
    wheelbase_m: float,
    max_speed: float,
    max_steering_rad: float,
) -> tuple[float, float]:
    """Convert a body-frame velocity command to a normalized servo pair.

    Bicycle model: the equivalent front-wheel steering angle for yaw rate
    ``angular`` at speed ``linear`` is ``atan(wheelbase_m * angular / linear)``,
    clamped to the platform's steering limit and normalized by it. Throttle is
    ``linear`` normalized by ``max_speed``. Both outputs are in [-1, 1].

    Implemented as ``atan2`` on ``abs(linear)`` with the sign restored for
    reverse motion: the naive ``atan2(wheelbase_m * angular, linear)`` lands in
    the second quadrant when ``linear`` is negative, commanding near-full
    opposite steering while backing up instead of the mirrored angle.
    """
    v = float(linear)
    if abs(v) < _REST_VELOCITY_EPS:
        return (0.0, 0.0)
    delta = math.atan2(wheelbase_m * float(angular), abs(v))
    if v < 0:
        delta = -delta
    delta = max(-max_steering_rad, min(max_steering_rad, delta))
    throttle = max(-1.0, min(1.0, v / max_speed))
    return (delta / max_steering_rad, throttle)
