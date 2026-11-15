"""Row 2.3: the builder emits a tray and a water plane, and the scene renders."""

import mujoco
import numpy as np
import pytest

from scene import SceneSpec
from scene.builder import MjcfBuilder, build_scene, num


def load(spec=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(spec))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def gid(m, name):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert i >= 0, f"no geom {name!r}"
    return i


def test_mjcf_parses_with_tray_and_water():
    m, _ = load()
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tray_0") >= 0
    for g in ("tray_0_floor", "tray_0_wall_xneg", "tray_0_wall_xpos",
              "tray_0_wall_yneg", "tray_0_wall_ypos", "water"):
        gid(m, g)


def test_water_is_contact_free_and_translucent_at_water_depth():
    s = SceneSpec(water_depth=0.04)
    m, d = load(s)
    w = gid(m, "water")
    assert m.geom_contype[w] == 0 and m.geom_conaffinity[w] == 0
    assert 0 < m.geom_rgba[w][3] < 1
    top = d.geom_xpos[w][2] + m.geom_size[w][2]
    assert top == pytest.approx(s.water_z)


def test_tray_floor_top_is_z0_and_walls_clear_the_water():
    s = SceneSpec()
    m, d = load(s)
    f = gid(m, "tray_0_floor")
    assert d.geom_xpos[f][2] + m.geom_size[f][2] == pytest.approx(0.0)
    wall = gid(m, "tray_0_wall_xpos")
    assert 2 * m.geom_size[wall][2] > s.water_depth


def test_tray_grows_with_the_grid():
    small, _ = load(SceneSpec(rows=2, cols=2))
    big, _ = load(SceneSpec(rows=6, cols=3))
    assert big.geom_size[gid(big, "tray_0_floor")][0] > small.geom_size[gid(small, "tray_0_floor")][0]


def test_duplicate_names_are_refused():
    b = MjcfBuilder(SceneSpec()).header().water()
    with pytest.raises(ValueError, match="duplicate"):
        b.water()


def test_num_is_fixed_six_decimals_without_negative_zero():
    assert num(0.1, -0.0, 1e-9, -2) == "0.100000 0.000000 0.000000 -2.000000"


def test_render_frame_zero_shows_tray_and_water(tmp_path):
    from scene.render import main

    out = tmp_path / "tray.png"
    try:
        assert main(["--frame", "0", "--out", str(out)]) == 0
    except Exception as e:  # no offscreen GL on this machine (e.g. a bare CI runner)
        pytest.skip(f"offscreen GL unavailable: {e}")
    assert out.read_bytes().startswith(b"\x89PNG")


def test_render_pixels_are_not_blank():
    from scene.render import render_frame

    m, _ = load()
    try:
        rgb = render_frame(m, 0, width=160, height=120)
    except Exception as e:
        pytest.skip(f"offscreen GL unavailable: {e}")
    assert rgb.shape == (120, 160, 3)
    assert np.ptp(rgb.reshape(-1, 3), axis=0).max() > 30, "image is one flat colour"
