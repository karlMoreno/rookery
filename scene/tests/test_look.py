"""The crane's CAD look (``SceneSpec.crane_look``): the CAD meshes hang on the
sim's crane bodies, look-only, and sit where the collision shapes are.

Physics must be identical with and without the look: the gates were tuned on
the plain shapes, and the meshes are only for people watching.
"""
from __future__ import annotations

import mujoco
import numpy as np
import pytest

from scene.builder import GENERATED, HIDDEN_GROUP, LOOK_GROUP, TRAY_WALL, build_scene
from scene.cli import DEFAULT_LOOK
from scene.spec import SceneSpec

LOOK = str((GENERATED / DEFAULT_LOOK).resolve())
SCENES = [dict(), dict(trays=3, rows=1, cols=4, shelf_gap=0.18)]
CRANE = ["crane_truck", "crane_carriage", "crane_arm", "crane_hook", "crane_fork_mid"]


def model(look: bool, **kw) -> tuple[mujoco.MjModel, mujoco.MjData]:
    m = mujoco.MjModel.from_xml_string(build_scene(SceneSpec(crane_look=LOOK if look else None, **kw)))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    return m, d


def cad_box(m, d, body: str) -> tuple[np.ndarray, np.ndarray]:
    """World AABB of the CAD meshes on ``body`` (``world`` for the frame)."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != b or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        i = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[i]: m.mesh_vertadr[i] + m.mesh_vertnum[i]]
        w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    return lo, hi


def geom_box(m, d, name: str) -> tuple[np.ndarray, np.ndarray]:
    g = m.geom(name).id
    c, s = d.geom_xpos[g], m.geom_size[g]
    return c - s, c + s


@pytest.mark.parametrize("kw", SCENES)
def test_the_look_changes_no_mass_joint_or_contact(kw):
    a, _ = model(False, **kw)
    b, _ = model(True, **kw)
    assert (a.nbody, a.njnt, a.nq, a.nu) == (b.nbody, b.njnt, b.nq, b.nu)
    np.testing.assert_array_equal(a.body_mass, b.body_mass)
    np.testing.assert_array_equal(a.body_inertia, b.body_inertia)
    np.testing.assert_array_equal(a.jnt_range, b.jnt_range)
    for g in range(a.ngeom):                       # every plain geom keeps its contact settings
        n = mujoco.mj_id2name(a, mujoco.mjtObj.mjOBJ_GEOM, g)
        h = b.geom(n).id
        for f in ("geom_contype", "geom_conaffinity", "geom_size", "geom_friction", "geom_condim"):
            np.testing.assert_array_equal(getattr(a, f)[g], getattr(b, f)[h], err_msg=f"{n}.{f}")


def test_cad_meshes_are_look_only_and_the_plain_crane_is_hidden():
    m, _ = model(True)
    cad = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith("cad_")]
    assert len(cad) == m.nmesh > 10
    for g in cad:
        assert (m.geom_contype[g], m.geom_conaffinity[g], m.geom_group[g]) == (0, 0, LOOK_GROUP)
    for name in ("crane_mast", "crane_fork_top", "hook", "hook_tip", "floor_rail"):
        assert m.geom_group[m.geom(name).id] == HIDDEN_GROUP


@pytest.mark.parametrize("kw", SCENES)
def test_the_cad_crane_sits_on_its_collision_shapes(kw):
    """At home: the fork stages line up with the sim's within 4 cm along X and
    1 cm in height, the hook's bottom within 5 mm, and the frame stands on the
    ground (not under it)."""
    m, d = model(True, **kw)
    for body, stage in (("crane_arm", "crane_fork_top"), ("crane_fork_mid", "crane_fork_mid")):
        lo, hi = cad_box(m, d, body)
        slo, shi = geom_box(m, d, stage)
        assert abs(hi[0] - shi[0]) < 0.04 and abs(lo[0] - slo[0]) < 0.04, (body, lo, hi, slo, shi)
        assert slo[2] - 0.035 < lo[2] < shi[2] + 0.01, (body, lo, slo, shi)
    lo, _ = cad_box(m, d, "crane_hook")
    hlo, _ = geom_box(m, d, "hook")
    assert abs(lo[2] - hlo[2]) < 0.005
    lo, _ = cad_box(m, d, "world")
    assert -TRAY_WALL <= lo[2] < -TRAY_WALL + 0.01


def test_the_cad_crane_follows_the_joints():
    """Meshes ride their bodies: move every joint and the hook mesh moves with the hook."""
    m, d = model(True)
    before = cad_box(m, d, "crane_hook")[0] - d.body("crane_hook").xpos
    for j, v in (("rail_y", 0.2), ("mast_z", 0.3), ("arm_x", 0.6)):
        d.joint(j).qpos[0] = v
    d.joint("arm_mid").qpos[0] = 0.07 + (0.6 - 0.07) / 2
    mujoco.mj_forward(m, d)
    np.testing.assert_allclose(cad_box(m, d, "crane_hook")[0] - d.body("crane_hook").xpos, before, atol=1e-9)


def test_the_committed_scenes_use_the_look():
    import json
    for name in ("rookery", "rack3"):
        assert json.loads((GENERATED / f"{name}.json").read_text())["crane_look"] == DEFAULT_LOOK
