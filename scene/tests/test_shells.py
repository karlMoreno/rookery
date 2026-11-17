"""Row 7.4: a shed shell per box, parked under the tray until a molt."""

import mujoco
import numpy as np
import pytest

from scene import SceneSpec, SpecError
from scene import builder, spec
from scene.builder import build_scene


def load(s=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(s))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def bid(m, n):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    assert i >= 0, n
    return i


def place(m, d, body, pos, gravity=True):
    """What the shell mover does: Sim::set_gravcomp (gravity on in a box, off
    when parked), then Sim::set_body_pose (write the free joint, stop it, forward)."""
    m.body_gravcomp[bid(m, body)] = 0.0 if gravity else 1.0
    j = m.body_jntadr[bid(m, body)]
    q, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
    d.qpos[q:q + 7] = [*pos, 1, 0, 0, 0]
    d.qvel[v:v + 6] = 0
    mujoco.mj_forward(m, d)


def run(m, d, seconds):
    for _ in range(round(seconds / m.opt.timestep)):
        mujoco.mj_step(m, d)


def test_the_spec_mirrors_the_builder_constants():
    assert spec.BOX_FLOOR_FOR_SPEC == builder.BOX_FLOOR
    assert spec.BOX_WALL_FOR_SPEC == builder.BOX_WALL
    assert spec.GROUND_Z == -builder.TRAY_WALL


def test_every_cell_has_a_shell_parked_under_it_and_it_stays_there():
    s = SceneSpec()
    m, d = load(s)
    names = [f"shell_{r}_{c}" for r in range(s.rows) for c in range(s.cols)]
    start = {n: d.xpos[bid(m, n)].copy() for n in [*names, "box_1_1", "crawfish_1_1"]}
    run(m, d, 1.0)
    for n, p0 in start.items():
        moved = abs(d.xpos[bid(m, n)] - p0).max()
        assert moved < 0.001, f"{n} moved {moved}"
    for r in range(s.rows):
        for c in range(s.cols):
            assert d.xpos[bid(m, f"shell_{r}_{c}")] == pytest.approx(s.shell_parked(r, c), abs=1e-3)


def test_a_parked_shell_moves_into_its_box_and_back():
    s = SceneSpec()
    m, d = load(s)
    run(m, d, 0.2)
    others = {n: d.xpos[bid(m, n)].copy() for n in ("box_1_2", "crawfish_1_2", "shell_1_1")}

    place(m, d, "shell_1_2", s.shell_in_box(1, 2))
    run(m, d, 1.0)
    p = d.xpos[bid(m, "shell_1_2")]
    assert p == pytest.approx(s.shell_in_box(1, 2), abs=2e-3), "rests on the box floor beside the crawfish"
    x, y = s.cell_center(1, 2)
    inner = s.box_size[0] / 2 - builder.BOX_WALL
    assert abs(p[0] - x) + s.shell_length / 2 <= inner + 1e-6
    assert abs(p[1] - y) + s.shell_radius <= inner + 1e-6
    for n, p0 in others.items():
        moved = abs(d.xpos[bid(m, n)] - p0).max()
        assert moved < 0.002, f"{n} moved {moved} when the shell arrived"

    place(m, d, "shell_1_2", s.shell_parked(1, 2), gravity=False)
    run(m, d, 1.0)
    assert d.xpos[bid(m, "shell_1_2")] == pytest.approx(s.shell_parked(1, 2), abs=1e-3)


def shell_contacts(m, d):
    shells = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"shell_{r}_{c}_body") for r in range(4) for c in range(4)}
    return sum(1 for i in range(d.ncon) if d.contact[i].geom1 in shells or d.contact[i].geom2 in shells)


def test_parked_shells_hover_and_touch_nothing():
    m, d = load()
    run(m, d, 0.5)
    assert shell_contacts(m, d) == 0
    place(m, d, "shell_0_3", SceneSpec().shell_in_box(0, 3))
    run(m, d, 0.5)
    assert shell_contacts(m, d) >= 1, "a shell in a box rests on its floor"
    place(m, d, "shell_0_3", SceneSpec().shell_parked(0, 3), gravity=False)
    run(m, d, 0.5)
    assert shell_contacts(m, d) == 0


def test_a_shell_whose_gravity_is_left_on_lands_on_the_pad():
    s = SceneSpec()
    m, d = load(s)
    m.body_gravcomp[bid(m, "shell_2_1")] = 0.0
    run(m, d, 1.0)
    p = d.xpos[bid(m, "shell_2_1")]
    assert p[2] == pytest.approx(s.shell_park_z + s.shell_radius, abs=1e-3), "caught by the pad"


def test_a_shell_in_a_box_weighs_on_the_box_not_the_crawfish():
    s = SceneSpec()
    m, d = load(s)
    place(m, d, "shell_2_2", s.shell_in_box(2, 2))
    run(m, d, 0.5)
    adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_2_2")]
    assert d.sensordata[adr] == pytest.approx(s.crawfish_mass * 9.81, rel=0.2)


def test_the_park_pad_is_hidden_and_only_shells_touch_it():
    m, _ = load()
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "shell_park")
    assert m.geom_group[g] == 3, "groups 0-2 are drawn by default"
    box_floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "box_0_0_floor")
    for other, touches in ((box_floor, False), (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "shell_0_0_body"), True)):
        hit = bool(m.geom_contype[g] & m.geom_conaffinity[other]) or bool(m.geom_contype[other] & m.geom_conaffinity[g])
        assert hit == touches


def test_no_shells_option_keeps_the_phase_6_ground_and_presets_have_none():
    m, _ = load(SceneSpec(shells=False))
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shell_0_0") == -1
    ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    assert (m.geom_contype[ground], m.geom_conaffinity[ground]) == (1, 1)
    m, _ = load(SceneSpec(preset="drop"))
    assert m.nbody == 2


@pytest.mark.parametrize("kw", [{"shell_length": 0.2}, {"shell_length": 0.01}, {"shell_offset_y": 0.055},
                                {"shell_offset_y": 0.01}, {"shell_park_z": -0.05}])
def test_a_shell_that_does_not_fit_is_refused(kw):
    with pytest.raises(SpecError):
        SceneSpec(**kw)


def test_parked_shells_sit_where_the_spec_says():
    s = SceneSpec()
    _, d = load(s)
    m, _ = load(s)
    q = np.array(d.qpos)
    j = m.body_jntadr[bid(m, "shell_3_1")]
    assert q[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 3] == pytest.approx(s.shell_parked(3, 1))
