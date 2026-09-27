"""Capability vocabulary a :class:`~strands_robots.simulation.base.SimEngine` backend declares.

The names form a closed set; a backend may add ``"<vendor>:<name>"`` extras. A
missing capability is reported with :data:`UNSUPPORTED_BY_BACKEND`, which no
operator grant can lift, so it is not a continuable refusal code. Pure stdlib.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from strands_robots.simulation.base import SimEngine

__all__ = [
    "CAMERA_PARAMS",
    "CONTACTS",
    "DEFAULT_CAPABILITIES",
    "FRAMES",
    "JOINTS",
    "KNOWN_CAPABILITIES",
    "LOAD_SCENE",
    "OBJECTS",
    "OBSERVATION",
    "OBS_NOISE",
    "OPTIONAL_CAPABILITY_METHODS",
    "POLICY_ROLLOUT",
    "RANDOMIZE",
    "RENDER",
    "ROBOTS",
    "STEP",
    "UNSUPPORTED_BY_BACKEND",
    "WORLD",
    "CapabilityNotSupported",
    "check_capabilities",
    "unsupported_result",
]

WORLD = "world"
ROBOTS = "robots"
STEP = "step"
OBSERVATION = "observation"
JOINTS = "joints"  # robot_joint_names / robot_action_keys / numeric send_action
OBJECTS = "objects"  # add_object / remove_object
RENDER = "render"  # render and camera frames
POLICY_ROLLOUT = "policy_rollout"  # run_policy and its rollout siblings
LOAD_SCENE = "load_scene"
RANDOMIZE = "randomize"
OBS_NOISE = "obs_noise"
CONTACTS = "contacts"
FRAMES = "frames"
CAMERA_PARAMS = "camera_params"

_CORE = (WORLD, ROBOTS, STEP, OBSERVATION, JOINTS, OBJECTS, RENDER, POLICY_ROLLOUT)
KNOWN_CAPABILITIES: tuple[str, ...] = _CORE + (LOAD_SCENE, RANDOMIZE, OBS_NOISE, CONTACTS, FRAMES, CAMERA_PARAMS)

#: What a backend without a declaration supports: it implements every abstract member.
DEFAULT_CAPABILITIES: frozenset[str] = frozenset(_CORE)

#: Optional capability -> the ``SimEngine`` method (a raising stub on the base) that delivers it.
OPTIONAL_CAPABILITY_METHODS: dict[str, str] = {
    LOAD_SCENE: "load_scene",
    RANDOMIZE: "randomize",
    OBS_NOISE: "set_obs_noise",
    CONTACTS: "get_contacts",
    FRAMES: "get_frame",
    CAMERA_PARAMS: "get_camera_params",
}

UNSUPPORTED_BY_BACKEND = "unsupported_by_backend"


class CapabilityNotSupported(NotImplementedError):
    """A member was called on a backend that lacks the capability it needs.

    Raised only by list-returning members; others return :func:`unsupported_result`.

    Args:
        capability: The missing capability name.
        member: The ``SimEngine`` member that was called.
    """

    code: str = UNSUPPORTED_BY_BACKEND

    def __init__(self, capability: str, member: str) -> None:
        super().__init__(f"{member} needs the {capability!r} capability, which this backend does not support")
        self.capability = capability
        self.member = member

    def __reduce__(self) -> tuple[type[CapabilityNotSupported], tuple[str, str]]:
        return (type(self), (self.capability, self.member))


def unsupported_result(capability: str, member: str, backend: str) -> dict[str, Any]:
    """Build the standard error result for a member the backend does not support.

    Args:
        capability: The missing capability name.
        member: The ``SimEngine`` member that was called.
        backend: The backend's name, usually its class name.

    Returns:
        A tool-result dict with ``status="error"`` and a ``json`` block carrying
        ``code``, ``capability``, ``member`` and ``backend``.
    """
    text = f"{backend} does not support {member}: it lacks the {capability!r} capability."
    return _error(text, capability=capability, member=member, backend=backend)


def check_capabilities(sim: SimEngine, required: Iterable[str], *, caller: str) -> dict[str, Any] | None:
    """Check that ``sim`` has every capability in ``required``.

    Args:
        sim: The engine to check.
        required: Capability names the caller needs.
        caller: The member or tool doing the check, named in the result.

    Returns:
        ``None`` when every capability is present, otherwise an error result
        whose ``json`` block lists the absent names under ``missing``.

    Raises:
        TypeError: ``required`` is a single ``str`` rather than a collection.
    """
    if isinstance(required, str):
        raise TypeError(f"required must be a collection of capability names, not the str {required!r}")
    missing = sorted(set(required) - frozenset(sim.capabilities()))
    if not missing:
        return None
    backend = type(sim).__name__
    text = f"{caller} needs capabilities {missing} that {backend} does not support."
    return _error(text, member=caller, backend=backend, missing=missing)


def _error(text: str, **payload: Any) -> dict[str, Any]:
    """Wrap ``text`` and ``payload`` in an ``unsupported_by_backend`` error result."""
    return {"status": "error", "content": [{"text": text}, {"json": {"code": UNSUPPORTED_BY_BACKEND, **payload}}]}
