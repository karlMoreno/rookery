"""Row 10.2: three stacked trays on a rack, each one box deep (#53: under a
shelf a box can't be lifted over the handles of the boxes in front of it)."""

import json

import mujoco
import pytest

from scene import SceneSpec, SpecError
from scene import builder, spec
from scene.builder import build_scene, spec_json

RACK = SceneSpec(trays=3, rows=1, cols=4, shelf_gap=0.15)


def load(s=RACK):
    m = mujoco.MjModel.from_xml_string(build_scene(s))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def name(m, kind, i):
    return mujoco.mj_id2name(m, kind, i) or ""


def test_the_spec_mirrors_the_builder_and_limits():
    assert spec.TRAY_FLOOR_FOR_SPEC == builder.TRAY_WALL
    assert spec.TRAY_FREEBOARD_FOR_SPEC == builder.TRAY_FREEBOARD
    assert spec.MAST_Z_TOP == builder.GANTRY_RANGES["mast_z"][1]


def test_tray_0_keeps_its_names_and_the_others_add_the_tray():
    m, _ = load()
    for n in ("box_0_2", "crawfish_0_2", "shell_0_2", "box_1_0_2", "crawfish_2_0_3", "shell_2_0_0",
              "tray_0", "tray_1", "tray_2"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) >= 0, n
    assert m.nbody == 1 + 3 + 3 * 4 * 3 + 4  # world, trays, boxes/crawfish/shells, crane


def test_free_joints_are_boxes_then_crawfish_then_shells_tray_major():
    m, _ = load()
    free = [name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j]) for j in range(m.njnt)
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    assert free[0] == "box_0_0" and free[4] == "box_1_0_0" and free[11] == "box_2_0_3"
    assert free[12] == "crawfish_0_0" and free[24] == "shell_0_0" and len(free) == 36


def test_no_box_touches_the_shelf_above_and_everything_rests():
    m, d = load()
    for i in range(d.ncon):
        a = name(m, mujoco.mjtObj.mjOBJ_GEOM, d.contact[i].geom1)
        b = name(m, mujoco.mjtObj.mjOBJ_GEOM, d.contact[i].geom2)
        pair = {a.split("_")[0], b.split("_")[0]}
        assert pair <= {"box", "tray", "crawfish", "post", "handle"}, (a, b)
        if "tray" in pair:
            box = a if a.startswith("box") else b
            tray = int((b if box == a else a).split("_")[1])
            box_tray = 0 if len(box.split("_")) == 4 else int(box.split("_")[1])
            assert tray == box_tray, f"{box} touches tray {tray}"
    start = d.xpos.copy()
    for _ in range(1000):
        mujoco.mj_step(m, d)
    for b in ("box_0_0", "box_1_0_1", "box_2_0_3", "crawfish_2_0_1"):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)
        assert abs(d.xpos[i][2] - start[i][2]) < 2e-3, b


def test_shelves_are_a_pitch_apart_with_the_gap_clear_and_listed_in_the_json():
    s = RACK
    assert s.shelf_pitch == pytest.approx(0.01 + s.handle_top_z + s.shelf_gap)
    assert json.loads(spec_json(s))["derived"]["shelf_bases"] == [0.0, 0.285, 0.57]
    # Clear space between tray 1's handle tops and tray 2's floor underside.
    assert s.shelf_base_z(2) - 0.01 - (s.shelf_base_z(1) + s.handle_top_z) == pytest.approx(s.shelf_gap)
    assert "shelf_bases" not in json.loads(spec_json(SceneSpec()))["derived"], "one tray: unchanged"


@pytest.mark.parametrize("kw,msg", [({"trays": 3, "rows": 1}, "too small for stacked trays"),
                                    ({"trays": 4, "rows": 1, "shelf_gap": 0.15}, "above its 0.80 m reach"),
                                    ({"trays": 3, "shelf_gap": 0.15}, "one box deep")])
def test_a_rack_that_does_not_fit_is_refused(kw, msg):
    with pytest.raises(SpecError, match=msg):
        SceneSpec(**kw)
