"""A backend's capability profile: what it declares, derives, validates and advertises."""

from __future__ import annotations

import ast
import functools
import importlib
import inspect
import pickle
import subprocess
import sys
import types
from pathlib import Path
from typing import Any

import pytest

from strands_robots.simulation import capabilities as caps
from strands_robots.simulation.base import SimEngine
from tests.tool_result_contract import assert_strands_tool_result, tool_json

_CORE = frozenset({caps.WORLD, caps.ROBOTS, caps.STEP, caps.OBSERVATION})


def _engine(**attrs: Any) -> type[SimEngine]:
    """Build a concrete SimEngine whose abstract members all return ``{}``."""
    body: dict[str, Any] = {m: (lambda self, *a, **k: {}) for m in SimEngine.__abstractmethods__}
    body["list_robots"] = lambda self: []
    return type("Engine", (SimEngine,), {**body, **attrs})


def test_backend_without_declaration_reports_full_manipulation_set() -> None:
    engine = _engine()()
    assert engine.capabilities() == caps.DEFAULT_CAPABILITIES
    assert engine.describe()["capabilities"] == sorted(caps.DEFAULT_CAPABILITIES)
    assert caps.DEFAULT_CAPABILITIES == _CORE | {caps.JOINTS, caps.OBJECTS, caps.RENDER, caps.POLICY_ROLLOUT}
    assert _engine(get_contacts=lambda self: {})().capabilities() == caps.DEFAULT_CAPABILITIES | {caps.CONTACTS}


@pytest.mark.parametrize(
    ("module", "cls", "extra"),
    [
        ("mujoco", "MuJoCoSimEngine", set(caps.KNOWN_CAPABILITIES)),
        ("newton", "NewtonSimEngine", {caps.CAMERA_PARAMS, caps.FRAMES, caps.OBS_NOISE, caps.RANDOMIZE}),
        ("isaac", "IsaacSimulation", set(caps.KNOWN_CAPABILITIES)),
    ],
)
def test_builtin_backends_report_their_exact_set(module: str, cls: str, extra: set[str]) -> None:
    engine_cls = getattr(importlib.import_module(f"strands_robots.simulation.{module}.simulation"), cls)
    assert engine_cls.__new__(engine_cls).capabilities() == caps.DEFAULT_CAPABILITIES | extra


_BAD_EXTRAS = (caps.LOAD_SCENE, "teleport", ":x", "acme:")


@pytest.mark.parametrize("declared", ["world", [*_CORE, 3], {caps.WORLD}, *(_CORE | {n} for n in _BAD_EXTRAS)])
def test_invalid_declaration_is_a_type_error_and_a_valid_one_is_frozen(declared: Any) -> None:
    with pytest.raises(TypeError):
        _engine(CAPABILITIES=declared)
    valid = {*_CORE, "acme:eclipse"}
    engine_cls = _engine(CAPABILITIES=valid)
    valid.add(caps.LOAD_SCENE)
    assert engine_cls.CAPABILITIES == frozenset(_CORE | {"acme:eclipse"})


def test_check_capabilities_names_the_missing_ones() -> None:
    needed = [caps.POLICY_ROLLOUT, caps.JOINTS]
    assert caps.check_capabilities(_engine()(), needed, caller="run_policy") is None
    result = caps.check_capabilities(_engine(CAPABILITIES=_CORE)(), needed, caller="run_policy")
    assert result is not None
    assert_strands_tool_result(result)
    payload = tool_json(result)
    assert (payload["code"], payload["member"]) == (caps.UNSUPPORTED_BY_BACKEND, "run_policy")
    assert payload["missing"] == sorted(needed)
    with pytest.raises(TypeError):
        caps.check_capabilities(_engine()(), caps.JOINTS, caller="run_policy")


def test_vocabulary_module_imports_no_heavy_dependency() -> None:
    # The package parent loads numpy through ``base``, so load the file standalone.
    script = (
        "import importlib.util, sys\n"
        f"spec = importlib.util.spec_from_file_location('_caps', {caps.__file__!r})\n"
        "spec.loader.exec_module(importlib.util.module_from_spec(spec))\n"
        "print(sorted(m for m in ('numpy', 'mujoco', 'torch', 'strands_robots') if m in sys.modules))\n"
    )
    out = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "[]"


def test_any_engine_like_object_is_checked_and_a_clashing_name_does_not_break_describe() -> None:
    class Reporter:
        def capabilities(self) -> frozenset[str]:
            return _CORE

    assert tool_json(caps.check_capabilities(Reporter(), [caps.JOINTS], caller="x"))["missing"] == [caps.JOINTS]
    assert _engine(capabilities=["legacy"])().describe()["capabilities"] is None
    tree = ast.parse(Path(caps.__file__).read_text(encoding="utf-8"))
    assert not any(
        isinstance(n, ast.ImportFrom) and (n.module or "").startswith("strands_robots") for n in ast.walk(tree)
    )


def test_unsupported_result_and_exception_carry_the_stable_code() -> None:
    result = caps.unsupported_result(caps.OBJECTS, "add_object", "OrbitSim")
    assert_strands_tool_result(result)
    assert result["status"] == "error"
    expected = {"code": caps.UNSUPPORTED_BY_BACKEND, "capability": caps.OBJECTS}
    assert tool_json(result) == {**expected, "member": "add_object", "backend": "OrbitSim"}
    err = caps.CapabilityNotSupported(caps.JOINTS, "robot_joint_names")
    assert isinstance(err, NotImplementedError)
    assert (err.code, err.capability, err.member) == (caps.UNSUPPORTED_BY_BACKEND, caps.JOINTS, "robot_joint_names")
    copy = pickle.loads(pickle.dumps(err))
    assert (type(copy), copy.capability, copy.member, str(copy)) == (type(err), err.capability, err.member, str(err))


class _Orbit(caps.ManipulationOptional, SimEngine):
    """The documented pattern, written statically so mypy checks it: the mixin supplies manipulation."""

    def create_world(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}

    def destroy(self) -> dict[str, Any]:
        return {}

    def reset(self) -> dict[str, Any]:
        return {}

    def get_state(self) -> dict[str, Any]:
        return {}

    def step(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}

    def add_robot(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}

    def remove_robot(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}

    def list_robots(self) -> list[str]:
        return []

    def get_observation(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}

    def send_action(self, *a: Any, **k: Any) -> dict[str, Any]:
        return {}


@pytest.mark.parametrize("call", [lambda s: s.add_object("rock"), lambda s: s.remove_object("rock"), _Orbit.render])
def test_mixin_refusals_are_error_results_with_stable_code(call: Any) -> None:
    engine = _Orbit()
    assert engine.capabilities() == _CORE
    result = call(engine)
    assert_strands_tool_result(result)
    assert result["status"] == "error"
    payload = tool_json(result)
    assert (payload["code"], payload["backend"]) == (caps.UNSUPPORTED_BY_BACKEND, "_Orbit")


def test_joint_names_without_joints_raises_rather_than_returning_empty() -> None:
    with pytest.raises(caps.CapabilityNotSupported) as info:
        _Orbit().robot_joint_names("sat0")
    assert (info.value.capability, info.value.member) == (caps.JOINTS, "robot_joint_names")


@pytest.mark.parametrize("member", ["add_object", "remove_object", "render", "robot_joint_names"])
def test_mixin_keeps_shared_parameter_order(member: str) -> None:
    mixin_member = getattr(caps.ManipulationOptional, member)
    assert inspect.signature(mixin_member) == inspect.signature(getattr(SimEngine, member))


@pytest.mark.parametrize("claimed", [caps.OBJECTS, caps.RENDER, caps.JOINTS])
def test_declared_capability_backed_by_refusal_is_a_type_error(claimed: str) -> None:
    with pytest.raises(TypeError, match=claimed):
        type("X", (_Orbit,), {"CAPABILITIES": _CORE | {claimed}})
    wrapped = functools.wraps(caps.ManipulationOptional.render)(lambda self, *a, **k: {})
    with pytest.raises(TypeError, match="render"):
        type("X", (_Orbit,), {"CAPABILITIES": _CORE | {caps.RENDER}, "render": wrapped})


def test_mixin_backend_that_overrides_render_may_declare_it() -> None:
    def render(self: Any, camera_name: str = "default", width: int | None = None, height: int | None = None) -> Any:
        return {"status": "success", "content": [{"text": camera_name}]}

    declared = (caps.ManipulationOptional.CAPABILITIES or frozenset()) | {caps.RENDER}
    engine = type("Imager", (_Orbit,), {"CAPABILITIES": declared, "render": render})()
    assert engine.capabilities() == _CORE | {caps.RENDER}
    assert engine.render()["status"] == "success"


#: Hand-written, independent of the source table: what describe() must drop per absent capability.
_GATED = {
    "joints": {"robot_joint_names", "robot_action_keys", "set_joint_positions", "set_joint_velocities"},
    "objects": {"add_object", "remove_object", "list_objects", "move_object"},
    "render": {"render", "render_depth", "render_all"},
    "policy_rollout": {"run_policy", "start_policy", "eval_policy", "evaluate_benchmark", "replay_episode"}
    | {"run_multi_policy"},
    "load_scene": {"load_scene"},
    "randomize": {"randomize"},
    "obs_noise": {"set_obs_noise"},
    "contacts": {"get_contacts"},
    "frames": {"get_frame"},
    "camera_params": {"get_camera_params"},
}
_ALL_GATED = set().union(*_GATED.values())
# The minimum each describe() reads on a ``__new__`` instance: no world, model, GPU or Kit app.
_SKELETONS: dict[str, tuple[str, dict[str, Any]]] = {
    "mujoco": ("MuJoCoSimEngine", {}),
    "newton": (
        "NewtonSimEngine",
        {"list_cameras": lambda self: [], "device": None, "_model": None, "_solver_name": "x"},
    ),
    "isaac": ("IsaacSimulation", {"_cameras": {}, "_world_created": False}),
}


def _skeleton(backend: str, declared: frozenset[str] | None) -> Any:
    name, attrs = _SKELETONS[backend]
    engine_cls = getattr(importlib.import_module(f"strands_robots.simulation.{backend}.simulation"), name)
    wp = types.SimpleNamespace(get_device=lambda *a: "cpu")
    body: dict[str, Any] = {"CAPABILITIES": declared, "list_robots": lambda self: [], "_wp": wp}
    sub: Any = type(f"Core{name}", (engine_cls,), {**body, "_world": None, "default_timestep": 0.01, **attrs})
    return sub.__new__(sub)


@pytest.mark.parametrize("absent", sorted(_GATED))
def test_describe_omits_methods_of_absent_capabilities(absent: str) -> None:
    full = _skeleton("mujoco", None).describe()["methods"].keys()
    methods = _skeleton("mujoco", frozenset(caps.KNOWN_CAPABILITIES) - {absent}).describe()["methods"].keys()
    assert not _GATED[absent] & methods
    assert (full & _ALL_GATED) - _GATED[absent] <= methods


@pytest.mark.parametrize("backend", ["base", *_SKELETONS])
def test_core_only_backend_advertises_no_gated_method(backend: str) -> None:
    def build(declared: frozenset[str] | None) -> Any:
        return _engine(CAPABILITIES=declared)() if backend == "base" else _skeleton(backend, declared)

    assert build(None).describe()["methods"].keys() & _ALL_GATED
    assert not build(_CORE).describe()["methods"].keys() & _ALL_GATED
