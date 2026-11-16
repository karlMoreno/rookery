"""Row 3.2: the crane in the scene matches limits.rs, holds against gravity
and settles a 0.3 m step in under 1 s with no overshoot."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import mujoco
import pytest

from scene import SceneSpec
from scene.builder import (
    GANTRY_HOME,
    GANTRY_RANGES,
    HANDLE_RADIUS,
    HOOK_FINGER,
    HOOK_RADIUS,
    HOOK_TIP_RISE,
    build_scene,
)

REPO = Path(__file__).resolve().parents[2]
JOINTS = ["rail_y", "mast_z", "arm_x", "hook_lift"]
DONE_TOL = 0.002  # mast::DONE_TOL


def load():
    m = mujoco.MjModel.from_xml_string(build_scene(SceneSpec()))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home"))
    mujoco.mj_forward(m, d)
    return m, d


def q(m, d, name):
    return d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]]


def find_cargo():
    for c in (shutil.which("cargo"), str(Path.home() / ".cargo/bin/cargo"),
              "/opt/homebrew/opt/rustup/bin/cargo"):
        if c and Path(c).exists():
            return c
    return None


def dump_limits():
    """`cargo run -p mast --example dump-limits` as JSON."""
    cargo = find_cargo()
    if cargo is None:
        if os.environ.get("CI"):
            pytest.fail("cargo not found in CI; the drift check must run there")
        pytest.skip("cargo not found")
    out = subprocess.run([cargo, "run", "-q", "-p", "mast", "--example", "dump-limits"],
                         cwd=REPO, check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def test_hook_geometry_and_home_match_mast():
    """Drift check (row 4.1): the builder's hook and home equal mast's constants."""
    d = dump_limits()
    hook = d["hook"]
    assert HOOK_FINGER == pytest.approx(hook["finger"])
    assert HOOK_RADIUS == pytest.approx(hook["radius"])
    assert HOOK_TIP_RISE == pytest.approx(hook["tip_rise"])
    assert 2 * HANDLE_RADIUS == pytest.approx(hook["handle_diameter"])
    assert [GANTRY_HOME[j] for j in JOINTS] == pytest.approx(d["home"])


def test_mjcf_ranges_match_limits_rs():
    """Drift check: the scene's joint ranges equal the Rust constants."""
    rust = dump_limits()["ranges"]
    m, _ = load()
    for name in JOINTS:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert j >= 0, f"no joint {name}"
        assert list(m.jnt_range[j]) == pytest.approx(rust[name], abs=1e-6), name
        assert list(GANTRY_RANGES[name]) == pytest.approx(rust[name], abs=1e-6), name
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        assert list(m.actuator_ctrlrange[a]) == pytest.approx(rust[name], abs=1e-6), name


def test_joint_values_are_the_limits_coordinates():
    s = SceneSpec()
    m, d = load()
    for name in JOINTS:
        assert q(m, d, name) == pytest.approx(GANTRY_HOME[name])
    hook = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "crane_hook")
    # Hook x = rail_line_x + arm_x; hook z = mast_z + hook_lift; hook y = rail_y.
    assert d.xpos[hook] == pytest.approx((s.rail_line_x + GANTRY_HOME["arm_x"], GANTRY_HOME["rail_y"],
                                          GANTRY_HOME["mast_z"] + GANTRY_HOME["hook_lift"]))


def test_home_holds_without_sag():
    m, d = load()
    for _ in range(1000):
        mujoco.mj_step(m, d)
    for name in JOINTS:
        assert abs(q(m, d, name) - GANTRY_HOME[name]) < DONE_TOL, name


@pytest.mark.parametrize("name, step", [("rail_y", 0.3), ("mast_z", 0.3), ("arm_x", 0.3), ("hook_lift", 0.03)])
def test_step_settles_in_one_second_without_overshoot(name, step):
    m, d = load()
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    start = q(m, d, name)
    goal = start + step
    d.ctrl[a] = goal
    peak = start
    for _ in range(1000):  # 1 s
        mujoco.mj_step(m, d)
        peak = max(peak, q(m, d, name))
    assert abs(q(m, d, name) - goal) < DONE_TOL, f"{name} at {q(m, d, name)} after 1 s, goal {goal}"
    assert peak - goal < 0.001, f"{name} overshot by {peak - goal}"


def test_only_the_hook_collides():
    m, _ = load()
    for i in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or ""
        if n.startswith("crane_") or n == "hook_stem":
            assert m.geom_contype[i] == 0 and m.geom_conaffinity[i] == 0, n
    for n in ("hook", "hook_tip"):
        assert m.geom_contype[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)] == 1


def test_boxes_still_rest_with_the_crane_present():
    m, d = load()
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box_0_0")
    z0 = d.xpos[b][2]
    for _ in range(500):
        mujoco.mj_step(m, d)
    assert abs(d.xpos[b][2] - z0) < 0.002
