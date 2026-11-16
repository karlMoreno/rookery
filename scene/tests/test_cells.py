"""Row 2.4: one mesh box per cell, handles above the water, boxes rest stably."""

import mujoco
import numpy as np
import pytest

from scene import SceneSpec
from scene.builder import build_scene


def load(spec=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(spec))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def box_bodies(m):
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or "" for i in range(m.nbody)]
    return [(i, n) for i, n in enumerate(names) if n.startswith("box_")]


def test_boxes_are_row_major_and_unique():
    s = SceneSpec(rows=3, cols=2, box_size=(0.12, 0.12, 0.08))
    m, _ = load(s)
    names = [n for _, n in box_bodies(m)]
    assert names == [f"box_{r}_{c}" for r in range(3) for c in range(2)]


def test_each_box_sits_at_its_cell_center():
    s = SceneSpec()
    m, d = load(s)
    for i, n in box_bodies(m):
        r, c = map(int, n.split("_")[1:])
        assert d.xpos[i][:2] == pytest.approx(s.cell_center(r, c))


def test_handle_tops_are_at_handle_top_z_and_above_water():
    s = SceneSpec()
    m, d = load(s)
    water = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "water")
    water_top = d.geom_xpos[water][2] + m.geom_size[water][2]
    for r in range(s.rows):
        for c in range(s.cols):
            g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"handle_{r}_{c}")
            top = d.geom_xpos[g][2] + m.geom_size[g][0]
            assert top == pytest.approx(s.handle_top_z)
            assert d.geom_xpos[g][2] > water_top


def test_boxes_rest_on_the_tray_without_tunnelling_or_drifting():
    s = SceneSpec()
    m, d = load(s)
    start = {i: d.xpos[i].copy() for i, _ in box_bodies(m)}
    for _ in range(1000):  # 1 s
        mujoco.mj_step(m, d)
    for i, n in box_bodies(m):
        moved = np.abs(d.xpos[i] - start[i])
        assert moved.max() < 0.002, f"{n} moved {moved}"
        assert d.xpos[i][2] > -0.001, f"{n} sank into the tray"


def test_box_mass_is_positive_and_reasonable():
    m, _ = load()
    i, _ = box_bodies(m)[0]
    assert 0.05 < m.body_subtreemass[i] < 0.5
