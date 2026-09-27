"""``register_robot`` accepts a robot that has no simulation asset.

The package registry already ships hardware-only entries; the public writer can
now produce one. ``tests/registry/conftest.py`` isolates the user registry.
"""

from __future__ import annotations

import pytest

from strands_robots.registry import get_robot, has_hardware, has_sim, resolve_name
from strands_robots.registry._overlay import user_registry_path
from strands_robots.registry.user_registry import get_user_robots, register_robot

# Hardware an asset-less robot cannot be built from.
_UNBUILDABLE = [None, {}, {"lerobot_type": ""}, {"lerobot_type": "  "}, {"lerobot_type": 5}, {"port": 1}]
_UNBUILDABLE += [{"driver": "auto"}, {"driver": "lerobot"}, {"driver": "bogus"}]


def test_asset_less_robot_is_stored_with_no_asset_block():
    hardware = {"driver": "strands"}
    entry = register_robot("orbiter", description="Spacecraft with no mesh", category="aerial", hardware=hardware)
    assert entry["hardware"] is not hardware
    assert "asset" not in entry
    assert "_user_asset_path" not in entry
    assert entry["hardware"] == {"driver": "strands"}
    assert "asset" not in get_user_robots()["orbiter"]


def test_asset_less_robot_is_reachable_in_real_mode_only():
    register_robot("orbiter", hardware={"driver": "strands"})
    assert get_robot("orbiter") is not None
    assert has_hardware("orbiter")
    assert not has_sim("orbiter")


def test_asset_less_robot_may_declare_only_a_lerobot_type():
    entry = register_robot("remote_arm", hardware={"lerobot_type": "so101_follower"})
    assert "asset" not in entry
    assert get_robot("remote_arm")["hardware"]["lerobot_type"] == "so101_follower"


def test_asset_less_robot_is_reachable_by_its_alias():
    assert register_robot("orbiter", aliases=["sat_one"], hardware={"driver": "strands"})["aliases"] == ["sat_one"]
    assert resolve_name("sat_one") == "orbiter"


def test_asset_less_alias_colliding_with_a_shipped_robot_writes_nothing():
    with pytest.raises(ValueError, match="panda"):
        register_robot("orbiter", aliases=["panda"], hardware={"driver": "strands"})
    assert not user_registry_path().exists()


@pytest.mark.parametrize("hardware", _UNBUILDABLE)
def test_asset_less_robot_without_a_hardware_declaration_is_refused(hardware):
    with pytest.raises(ValueError, match="Robot 'orbiter' is registered without model_xml"):
        register_robot("orbiter", hardware=hardware)
    assert "orbiter" not in get_user_robots()


@pytest.mark.parametrize(
    "asset_only",
    [{"scene_xml": "scene.xml"}, {"asset_dir": "orbiter"}, {"robot_descriptions_module": "panda_mj_description"}],
)
def test_asset_arguments_without_model_xml_are_refused(asset_only):
    with pytest.raises(ValueError, match="only apply to a simulation asset and require model_xml; pass model_xml"):
        register_robot("orbiter", hardware={"driver": "strands"}, **asset_only)
    assert "orbiter" not in get_user_robots()


@pytest.mark.parametrize("model_xml", ["", "  "])
def test_an_empty_model_xml_is_refused(model_xml):
    with pytest.raises(ValueError, match="model_xml"):
        register_robot("orbiter", model_xml=model_xml, hardware={"driver": "strands"})
    assert "orbiter" not in get_user_robots()


@pytest.mark.parametrize("model_xml", [None, "orbiter.xml"])
@pytest.mark.parametrize("hardware", ["strands", ["driver"]])
def test_a_hardware_value_that_is_not_a_dict_is_refused(model_xml, hardware):
    with pytest.raises(TypeError, match="hardware must be a dict"):
        register_robot("orbiter", model_xml=model_xml, hardware=hardware)
    assert "orbiter" not in get_user_robots()


def test_sim_mode_on_an_asset_less_robot_fails_with_a_clear_error():
    from strands_robots import Robot

    register_robot("orbiter", hardware={"driver": "strands"})
    with pytest.raises(RuntimeError, match="'orbiter' is registered for real hardware only"):
        Robot("orbiter", mode="sim")


@pytest.mark.parametrize("model_xml", [5, pytest.param(__import__("pathlib").Path("m.xml"), id="path")])
def test_a_model_xml_that_is_not_a_str_is_a_type_error(model_xml):
    with pytest.raises(TypeError, match="model_xml must be a str"):
        register_robot("orbiter", model_xml=model_xml, hardware={"driver": "strands"})
    assert not user_registry_path().exists()


def test_a_str_subclass_cannot_pass_a_blank_lerobot_type():
    class Padded(str):
        def strip(self, chars=None):  # type: ignore[override]
            return "not blank"

    with pytest.raises(ValueError, match="hardware must declare"):
        register_robot("orbiter", hardware={"lerobot_type": Padded("   ")})


def test_a_str_subclass_cannot_pass_a_blank_model_xml():
    class Padded(str):
        def strip(self, chars=None):  # type: ignore[override]
            return "not blank"

    with pytest.raises(ValueError, match="model_xml must name a model file"):
        register_robot("orbiter", model_xml=Padded("   "), hardware={"driver": "strands"})
