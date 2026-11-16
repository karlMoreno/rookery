"""Row 2.5: exit tray and crane rails placed and validated; output is byte-identical."""

import hashlib
import json

import mujoco
import pytest

from scene import SceneSpec, SpecError
from scene.builder import build_scene, spec_json
from scene.cli import main


def load(spec=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(spec))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def geom(m, name):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert i >= 0, f"no geom {name!r}"
    return i


def test_exit_tray_top_is_at_its_pose():
    s = SceneSpec()
    m, d = load(s)
    g = geom(m, "exit_tray")
    x, y, z = s.exit_pose
    assert d.geom_xpos[g][:2] == pytest.approx((x, y))
    assert d.geom_xpos[g][2] + m.geom_size[g][2] == pytest.approx(z)


def test_aisle_side_order_is_rail_then_exit_tray_then_tray():
    s = SceneSpec()
    m, d = load(s)
    rail = d.geom_xpos[geom(m, "floor_rail")][0]
    exit_x = d.geom_xpos[geom(m, "exit_tray")][0]
    tray_wall = d.geom_xpos[geom(m, "tray_0_wall_xneg")][0]
    assert rail < exit_x < tray_wall


def test_rails_are_contact_free_and_guide_is_up_high():
    s = SceneSpec()
    m, d = load(s)
    for n in ("floor_rail", "guide_rail", "guide_post_yneg", "guide_post_ypos"):
        g = geom(m, n)
        assert m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
    g = geom(m, "guide_rail")
    assert d.geom_xpos[g][2] - m.geom_size[g][2] == pytest.approx(s.guide_rail_height)


@pytest.mark.parametrize(
    "kwargs, needle",
    [
        ({"exit_tray_pose": (-0.30, 0.0, 0.10)}, "overlaps the tray"),
        ({"rail_x": -0.40}, "aisle side of the exit tray"),
        ({"exit_tray_pose": (-0.45, 0.6, 0.10)}, "rail's reach"),
        ({"exit_tray_pose": (-0.45, 0.0, 0.0)}, "above the tray floor"),
        ({"guide_rail_height": 0.25}, "guide_rail_height"),
        ({"guide_rail_height": 0.0}, "guide_rail_height must be positive"),
    ],
)
def test_bad_placement_raises(kwargs, needle):
    with pytest.raises(SpecError) as e:
        SceneSpec(**kwargs)
    assert any(needle in p for p in e.value.problems), e.value.problems


def test_auto_placement_follows_the_grid():
    for rows in (2, 4, 8):
        s = SceneSpec(rows=rows)  # would overlap with a fixed default pose
        assert s.exit_pose[0] + s.exit_tray_half < -s.tray_size[0] / 2
        assert s.rail_line_x < s.exit_pose[0] - s.exit_tray_half


def test_explicit_placement_wins():
    s = SceneSpec(exit_tray_pose=(-0.5, 0.1, 0.2), rail_x=-0.8)
    assert s.exit_pose == (-0.5, 0.1, 0.2) and s.rail_line_x == -0.8


def test_build_is_deterministic_in_process():
    assert build_scene(SceneSpec()) == build_scene(SceneSpec())


def test_no_float_escapes_the_six_decimal_format():
    xml = build_scene(SceneSpec())
    for value in __import__("re").findall(r'"([^"]*)"', xml):
        for tok in value.split():
            if any(ch.isdigit() for ch in tok) and "." in tok:
                assert len(tok.split(".")[1]) == 6, tok


def test_cli_writes_xml_and_json_byte_identically(tmp_path):
    args = ["--trays", "1", "--cells", "4x4", "--shelf-gap", "0.08"]
    digests = []
    for sub in ("a", "b"):
        out = tmp_path / sub / "rookery.xml"
        assert main([*args, "--out", str(out)]) == 0
        digests.append(tuple(hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (out, out.with_suffix(".json"))))
    assert digests[0] == digests[1]


def test_json_echoes_the_spec():
    s = SceneSpec(rows=3)
    data = json.loads(spec_json(s))
    assert data["rows"] == 3 and data["exit_tray_pose"] is None
    assert data["derived"]["exit_pose"] == pytest.approx(list(s.exit_pose))
    assert spec_json(s).endswith("}\n")
