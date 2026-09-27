"""A backend's capability profile: what it declares, derives, validates and advertises."""

from __future__ import annotations

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
    assert _engine()().capabilities() == caps.DEFAULT_CAPABILITIES
    assert caps.DEFAULT_CAPABILITIES == _CORE | {caps.JOINTS, caps.OBJECTS, caps.RENDER, caps.POLICY_ROLLOUT}
    with_contacts = _engine(get_contacts=lambda self: {})()
    assert with_contacts.capabilities() == caps.DEFAULT_CAPABILITIES | {caps.CONTACTS}


@pytest.mark.parametrize(
    ("dependency", "module", "cls"),
    [
        ("mujoco", "strands_robots.simulation.mujoco.simulation", "MuJoCoSimEngine"),
        ("newton", "strands_robots.simulation.newton.simulation", "NewtonSimEngine"),
        ("isaacsim", "strands_robots.simulation.isaac.simulation", "IsaacSimulation"),
    ],
)
def test_builtin_backends_report_the_manipulation_set(dependency: str, module: str, cls: str) -> None:
    pytest.importorskip(dependency)
    engine_cls = getattr(pytest.importorskip(module), cls)
    reported = engine_cls.__new__(engine_cls).capabilities()
    assert caps.DEFAULT_CAPABILITIES <= reported
    if dependency == "mujoco":
        assert reported == frozenset(caps.KNOWN_CAPABILITIES)


def test_declared_capability_backed_by_stub_or_unknown_name_is_a_type_error() -> None:
    with pytest.raises(TypeError, match="load_scene"):
        _engine(CAPABILITIES=_CORE | {caps.LOAD_SCENE})
    with pytest.raises(TypeError, match="teleport"):
        _engine(CAPABILITIES=_CORE | {"teleport"})
    assert _engine(CAPABILITIES=_CORE | {"acme:eclipse"})().capabilities() == _CORE | {"acme:eclipse"}


def test_describe_omits_methods_of_absent_capabilities() -> None:
    full = _engine()().describe()
    assert full["capabilities"] == sorted(caps.DEFAULT_CAPABILITIES)
    assert {"add_object", "render", "run_policy"} <= set(full["methods"])

    desc = _engine(CAPABILITIES=_CORE)().describe()
    assert desc["capabilities"] == sorted(_CORE)
    assert not {"add_object", "remove_object", "render", "run_policy", "eval_policy", "load_scene"} & set(
        desc["methods"]
    )
    assert {"step", "stop_policy", "destroy", "reset"} <= set(desc["methods"])


def test_unsupported_result_and_exception_carry_the_stable_code() -> None:
    result = caps.unsupported_result(caps.OBJECTS, "add_object", "OrbitSim")
    assert_strands_tool_result(result)
    assert result["status"] == "error"
    assert tool_json(result) == {
        "code": caps.UNSUPPORTED_BY_BACKEND,
        "capability": caps.OBJECTS,
        "member": "add_object",
        "backend": "OrbitSim",
    }
    err = caps.CapabilityNotSupported(caps.JOINTS, "robot_joint_names")
    assert isinstance(err, NotImplementedError)
    assert (err.code, err.capability, err.member) == (caps.UNSUPPORTED_BY_BACKEND, caps.JOINTS, "robot_joint_names")


def test_check_capabilities_names_the_missing_ones() -> None:
    needed = [caps.POLICY_ROLLOUT, caps.JOINTS]
    assert caps.check_capabilities(_engine()(), needed, caller="run_policy") is None
    result = caps.check_capabilities(_engine(CAPABILITIES=_CORE)(), needed, caller="run_policy")
    assert result is not None
    assert_strands_tool_result(result)
    payload = tool_json(result)
    assert (payload["code"], payload["member"]) == (caps.UNSUPPORTED_BY_BACKEND, "run_policy")
    assert payload["missing"] == sorted(needed)


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
