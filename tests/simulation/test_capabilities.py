"""A backend's capability profile: what it declares, derives, validates and advertises."""

from __future__ import annotations

import importlib
import pickle
import subprocess
import sys
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
