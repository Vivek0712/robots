#!/usr/bin/env python3
"""Universal rosbridge tool - any ROS graph over a WebSocket, no ROS install.

Where :func:`strands_robots.tools.use_ros.use_ros` speaks to a ROS 2 graph
through in-process ``rclpy`` (requiring a sourced distro), ``use_rosbridge``
speaks the rosbridge JSON protocol over a WebSocket via ``roslibpy`` - a pure
pip dependency. That gives it two properties no other transport here has:

* **ROS 1 robots** (rosbridge_suite ships for ROS1 and ROS2 alike) - e.g. the
  NASA Curiosity rover Gazebo simulation (ROS1 Noetic).
* **No ROS environment on this machine** - the agent can run on macOS, CI, or
  any laptop and drive a robot across the network.

Requirements:
    ``pip install "strands-robots[rosbridge]"`` (roslibpy). The robot side
    runs ``rosbridge_server`` (with ``rosapi``) - standard in every rosbridge
    install. rosbridge is unauthenticated by default: use on trusted networks.

Graph introspection uses the ``rosapi`` node's services. Interface types are
ROS1-style two-segment names (``geometry_msgs/Twist``); field payloads are
plain JSON dicts, exactly as rosbridge transmits them.

Actions:
    status         - roslibpy availability + connectivity to host:port.
    list_topics    - topics with their types (rosapi /rosapi/topics).
    list_services  - services (rosapi /rosapi/services).
    echo           - subscribe and return up to N messages as JSON. Type
                     auto-resolved via rosapi when omitted.
    publish        - advertise, publish N messages built from ``fields``,
                     unadvertise.
    service_call   - call a service with a JSON request dict.

Examples:
    use_rosbridge(action="status", host="192.168.1.20")
    use_rosbridge(action="list_topics")
    use_rosbridge(action="echo", topic="/curiosity_mars_rover/odom", count=1)
    use_rosbridge(action="publish",
                  topic="/curiosity_mars_rover/ackermann_drive_controller/cmd_vel",
                  type="geometry_msgs/Twist",
                  fields={"linear": {"x": 1.0}, "angular": {"z": 0.0}})
"""

from __future__ import annotations

import logging
import re
import threading
import time
from typing import Any

from strands import tool

logger = logging.getLogger(__name__)

# Graph names: same allowlist posture as use_ros. Types are ROS1 two-segment.
_NAME_RE = re.compile(r"^[A-Za-z0-9_/~]+$")
_TYPE_RE = re.compile(r"^[A-Za-z0-9_]+/[A-Za-z0-9_]+$")
_HOST_RE = re.compile(r"^[A-Za-z0-9._-]+$")

_INSTALL_HINT = (
    "roslibpy is not importable - install the rosbridge extra: "
    'pip install "strands-robots[rosbridge]". The rosbridge transport is pure '
    "pip (WebSocket); no ROS environment is needed on this machine."
)


class _RosbridgeBackend:
    """Process-wide cache of live roslibpy connections, keyed by (host, port).

    Tool calls are stateless; the WebSocket underneath is reused across calls
    (the rosbridge analogue of use_ros's single long-lived rclpy node). All
    access is serialised through ``lock``.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._connections: dict[tuple[str, int], Any] = {}
        self._available: bool | None = None

    def available(self) -> bool:
        if self._available is None:
            try:
                import roslibpy  # noqa: F401

                self._available = True
            except ImportError:
                self._available = False
        return self._available

    def connect(self, host: str, port: int, timeout: float) -> Any:
        """Return a live connection to host:port, dialing (or re-dialing) if needed."""
        import roslibpy

        ros = self._connections.get((host, port))
        if ros is not None and getattr(ros, "is_connected", False):
            return ros
        if ros is not None:
            # Stale WebSocket: tear down best-effort and re-dial fresh below.
            try:
                ros.terminate()
            except Exception:  # noqa: BLE001 - third-party teardown must never block a re-dial
                logger.debug("terminate of stale rosbridge connection failed", exc_info=True)
            self._connections.pop((host, port), None)
        ros = roslibpy.Ros(host=host, port=port)
        try:
            ros.run(timeout=timeout)
        except Exception as exc:  # noqa: BLE001 - roslibpy raises library-specific errors; all mean "no connection"
            raise TimeoutError(
                f"could not connect to rosbridge at ws://{host}:{port} within {timeout}s "
                f"- is rosbridge_server running? ({exc})"
            ) from exc
        if not getattr(ros, "is_connected", False):
            raise TimeoutError(
                f"could not connect to rosbridge at ws://{host}:{port} within {timeout}s - is rosbridge_server running?"
            )
        self._connections[(host, port)] = ros
        return ros

    @property
    def lock(self) -> threading.RLock:
        return self._lock


_backend = _RosbridgeBackend()


def _ok(text: str) -> dict[str, Any]:
    return {"status": "success", "content": [{"text": text}]}


def _err(text: str) -> dict[str, Any]:
    return {"status": "error", "content": [{"text": f"use_rosbridge: {text}"}]}


@tool
def use_rosbridge(
    action: str,
    host: str = "localhost",
    port: int = 9090,
    topic: str | None = None,
    service: str | None = None,
    type: str | None = None,
    fields: dict[str, Any] | None = None,
    timeout: float = 5.0,
    count: int = 1,
    rate: float = 10.0,
) -> dict[str, Any]:
    """Universal rosbridge tool - ROS over a WebSocket, no ROS install needed.

    Args:
        action: One of ``status``, ``list_topics``, ``list_services``,
            ``echo``, ``publish``, ``service_call``.
        host: rosbridge server hostname or IP.
        port: rosbridge WebSocket port (default 9090).
        topic: Topic name (``echo``, ``publish``).
        service: Service name (``service_call``).
        type: ROS1 two-segment interface type, e.g. ``geometry_msgs/Twist``.
            Auto-resolved for ``echo`` when omitted.
        fields: JSON field dict (``publish`` message / ``service_call`` request).
        timeout: Seconds for connection, sample collection, or a service call.
        count: Messages to echo or publish.
        rate: Publish rate in Hz.

    Returns:
        A Strands tool result dict ``{"status": ..., "content": [{"text": ...}]}``.
    """
    fields = fields or {}

    if not host or not _HOST_RE.match(host):
        return _err(f"invalid host: {host!r}")
    if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
        return _err(f"invalid port: {port!r} (expected 1-65535)")
    if topic is not None and not _NAME_RE.match(topic):
        return _err(f"invalid topic name: {topic!r}")
    if service is not None and not _NAME_RE.match(service):
        return _err(f"invalid service name: {service!r}")
    if type is not None and not _TYPE_RE.match(type):
        return _err(f"invalid interface type: {type!r} (expected ROS1 pkg/Name like geometry_msgs/Twist)")

    if action == "status":
        if not _backend.available():
            return _ok("backend: none - " + _INSTALL_HINT)
        try:
            with _backend.lock:
                _backend.connect(host, port, timeout)
        except TimeoutError as exc:
            return _ok(f"backend: roslibpy; not connected - {exc}")
        return _ok(f"backend: roslibpy; connected to ws://{host}:{port}")

    if not _backend.available():
        return _err(_INSTALL_HINT)

    try:
        with _backend.lock:
            ros = _backend.connect(host, port, timeout)

            if action == "publish":
                if not topic or not type:
                    return _err("publish requires topic and type")
                _publish(ros, topic, type, fields, count, rate)
                return _ok(f"published {count} message(s) to {topic}")

            return _err(f"unknown action: {action}")
    except TimeoutError as exc:
        return _err(str(exc))
    except (ImportError, KeyError, AttributeError, ValueError, TypeError, OSError) as exc:
        return _err(f"{action} failed: {exc}")


def _publish(ros: Any, topic: str, msg_type: str, fields: dict[str, Any], count: int, rate: float) -> None:
    import roslibpy

    pub = roslibpy.Topic(ros, topic, msg_type)
    pub.advertise()
    try:
        time.sleep(0.2)  # settle so rosbridge registers the publisher before the first send
        period = 1.0 / rate if rate > 0 else 0.0
        for _ in range(count):
            pub.publish(roslibpy.Message(dict(fields)))
            if period:
                time.sleep(period)
    finally:
        pub.unadvertise()
