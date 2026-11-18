"""Row 8.4: a row of exit slots, so harvested boxes can wait side by side."""

import json

import mujoco
import pytest

from scene import SceneSpec, SpecError
from scene import builder, spec
from scene.builder import build_scene, spec_json


def load(s=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(s))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def test_six_slots_nearest_the_middle_first_all_within_the_rail():
    s = SceneSpec()
    ys = [round(y, 6) for _, y, _ in s.exit_slot_poses]
    assert ys == [-0.075, 0.075, -0.225, 0.225, -0.375, 0.375]
    assert all(x == s.exit_pose[0] and z == s.exit_pose[2] for x, _, z in s.exit_slot_poses)
    assert spec.RAIL_Y_REACH == builder.GANTRY_RANGES["rail_y"][1] == -builder.GANTRY_RANGES["rail_y"][0]


def test_the_plate_covers_every_slot_and_rookery_json_lists_them():
    s = SceneSpec()
    m, d = load(s)
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "exit_tray")
    half_y = m.geom_size[g][1]
    for _, y, _ in s.exit_slot_poses:
        assert abs(y) + s.box_size[1] / 2 <= half_y + 1e-9
    derived = json.loads(spec_json(s))["derived"]
    assert derived["exit_slots"] == [list(p) for p in s.exit_slot_poses]


def test_a_box_set_in_each_slot_rests_on_the_plate():
    s = SceneSpec()
    m, d = load(s)
    # Move the first six boxes onto the slots and let them settle for 1 s.
    for k, (x, y, z) in enumerate(s.exit_slot_poses):
        j = m.body_jntadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"box_0_{k % 4}" if k < 4 else f"box_1_{k - 4}")]
        a = m.jnt_qposadr[j]
        d.qpos[a:a + 7] = [x, y, z, 1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    for _ in range(1000):
        mujoco.mj_step(m, d)
    for k, (x, y, z) in enumerate(s.exit_slot_poses):
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"box_0_{k % 4}" if k < 4 else f"box_1_{k - 4}")
        assert d.xpos[b] == pytest.approx((x, y, z), abs=2e-3), k


def test_no_slots_is_the_phase_6_tray():
    m, _ = load(SceneSpec(exit_slots=0))
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "exit_tray")
    assert m.geom_size[g][0] == pytest.approx(m.geom_size[g][1])
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "exit_tray_leg_2") == -1
    assert SceneSpec(exit_slots=0).exit_slot_poses == []


@pytest.mark.parametrize("kw,msg", [({"exit_slots": 7}, "rail travel"), ({"exit_slots": -1}, "whole number"),
                                    ({"exit_slots": 2.5}, "whole number")])
def test_slots_out_of_reach_or_malformed_are_refused(kw, msg):
    with pytest.raises(SpecError, match=msg):
        SceneSpec(**kw)
