"""Row 5.2: a crawfish in every box, resting inside the walls, with a touch sensor."""

import mujoco
import pytest

from scene import SceneSpec, SpecError
from scene.builder import build_scene


def load(spec=SceneSpec()):
    m = mujoco.MjModel.from_xml_string(build_scene(spec))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def bid(m, n):
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    assert i >= 0, n
    return i


def test_every_box_has_a_crawfish_and_a_touch_sensor():
    s = SceneSpec()
    m, _ = load(s)
    for r in range(s.rows):
        for c in range(s.cols):
            bid(m, f"crawfish_{r}_{c}")
            assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, f"touch_{r}_{c}") >= 0
    assert m.nsensor == s.cells


def test_crawfish_rest_inside_the_walls_and_boxes_still_rest():
    s = SceneSpec()
    m, d = load(s)
    start = {n: d.xpos[bid(m, n)].copy() for n in ("box_1_1", "crawfish_1_1", "box_3_0")}
    for _ in range(1000):  # 1 s
        mujoco.mj_step(m, d)
    for n, p0 in start.items():
        moved = abs(d.xpos[bid(m, n)] - p0).max()
        assert moved < 0.002, f"{n} moved {moved}"
    # The capsule's footprint stays inside the box's inner walls.
    x, y = s.cell_center(1, 1)
    p = d.xpos[bid(m, "crawfish_1_1")]
    inner = s.box_size[0] / 2 - 0.003
    assert abs(p[0] - x) + s.crawfish_length / 2 <= inner + 1e-6
    assert abs(p[1] - y) + s.crawfish_radius <= inner + 1e-6
    assert p[2] == pytest.approx(0.004 + s.crawfish_radius, abs=1e-3)


def test_touch_reads_the_crawfish_weight_at_rest():
    s = SceneSpec()
    m, d = load(s)
    for _ in range(500):
        mujoco.mj_step(m, d)
    adr = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_2_2")]
    assert d.sensordata[adr] == pytest.approx(s.crawfish_mass * 9.81, rel=0.2)


def test_no_crawfish_option_and_presets_have_none():
    m, _ = load(SceneSpec(crawfish=False))
    assert m.nsensor == 0
    m, _ = load(SceneSpec(preset="drop"))
    assert m.nsensor == 0


@pytest.mark.parametrize("kw", [{"crawfish_length": 0.2}, {"crawfish_length": 0.01},
                                {"crawfish_radius": 0.05}])
def test_a_crawfish_that_does_not_fit_is_refused(kw):
    with pytest.raises(SpecError):
        SceneSpec(**kw)
