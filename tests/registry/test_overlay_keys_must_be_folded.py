"""A robot key the lookup fold cannot produce is refused, not silently unreachable.

Every reader looks a robot up by :func:`~strands_robots.registry.loader.normalize_robot_name`
of the query - lowercase, trimmed, dashes as underscores. ``register_robot``
folds the name before it writes, so its entries are always reachable. A
``user_robots.json`` written by hand (or by any tool other than
``register_robot``) was merged verbatim, so a key like ``sat-001`` or ``My_Arm``
loaded without complaint and then answered no query at all - not even the
spelling it was declared in, because that query is folded before it reaches the
registry.

The loader does not fold such a key on the user's behalf: two overlay keys, or
an overlay key and a package key, could collapse onto one entry and the merge
would keep whichever came last. It refuses the load instead, the same way it
refuses an alias collision or an unknown driver, and the error names the
spelling to rename to.
"""

from __future__ import annotations

import json

import pytest

from strands_robots.registry import get_robot, list_robots, normalize_robot_name
from strands_robots.registry.loader import _REGISTRY_DIR
from strands_robots.utils import get_base_dir


def _write_overlay(robots: dict) -> None:
    """Write user_robots.json directly, bypassing register_robot()."""
    path = get_base_dir() / "user_robots.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"robots": robots}))


def _entry(description: str = "hand-written") -> dict:
    return {
        "description": description,
        "category": "arm",
        "joints": 6,
        "asset": {"dir": "x", "model_xml": "x.xml", "scene_xml": "x.xml"},
    }


@pytest.mark.parametrize(
    ("declared", "folded"),
    [("sat-001", "sat_001"), ("My_Arm", "my_arm"), (" padded ", "padded")],
    ids=["dashed", "mixed-case", "padded"],
)
def test_an_overlay_key_that_is_not_folded_is_refused_with_its_folded_spelling(declared: str, folded: str) -> None:
    """The load fails and says which spelling the key has to be renamed to."""
    _write_overlay({declared: _entry()})

    with pytest.raises(ValueError, match=f"rename '{declared}' to '{folded}'"):
        get_robot(declared)


def test_an_overlay_key_that_folds_onto_a_shipped_robot_names_the_robot_it_would_replace() -> None:
    """Renaming ``Panda`` to ``panda`` would override the shipped entry, so the error says so."""
    _write_overlay({"Panda": _entry()})

    with pytest.raises(ValueError, match="a robot named 'panda' already exists"):
        list_robots()


def test_a_folded_overlay_key_still_loads_and_answers_every_spelling() -> None:
    """Control: the refusal is about the key's spelling, not about hand-written overlays."""
    _write_overlay({"sat_001": _entry("folded")})

    for query in ("sat_001", "sat-001", "SAT-001"):
        entry = get_robot(query)
        assert entry is not None, f"{query!r} reached no robot"
        assert entry["description"] == "folded"


def test_every_shipped_robot_key_is_already_folded() -> None:
    """The check covers package keys too, so the package must already comply."""
    shipped = json.loads((_REGISTRY_DIR / "robots.json").read_text(encoding="utf-8"))["robots"]
    assert len(shipped) >= 50, f"premise: the shipped registry declares few robots ({len(shipped)})"
    assert [key for key in shipped if key != normalize_robot_name(key)] == []
