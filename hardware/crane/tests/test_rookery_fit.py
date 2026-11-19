"""The Rookery presets against the scenes Rookery generates (row 2.1 of the crane
swap plan). If the sim's rack changes, these fail until the presets follow.

    presets/rookery_rack3.toml  <->  scene/generated/rack3.json
    presets/rookery_tray.toml   <->  scene/generated/rookery.json

Both sides use metres. The crane's frame has the bottom tray floor at
z = 0.10, Rookery's at z = 0. Reach is measured from the mast line, which is
Rookery's floor rail (`derived.rail_line_x`).
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from crane.spec import CraneSpec

ROOT = Path(__file__).resolve().parent.parent
REPO = ROOT.parents[1]
GENERATED = REPO / "scene" / "generated"
#: The old straight arm stuck out about this far behind the mast (Karl's figure).
OLD_ARM_OVERHANG = 0.52
TOL = 1e-6

CASES = [("rookery_rack3.toml", "rack3.json"), ("rookery_tray.toml", "rookery.json")]


def load(preset: str, scene: str) -> tuple[CraneSpec, dict]:
    if not (GENERATED / scene).exists():
        pytest.skip(f"{scene} not found: run from a Rookery checkout")
    return CraneSpec.from_toml(ROOT / "presets" / preset), json.loads((GENERATED / scene).read_text())


def needed_reach(meta: dict) -> tuple[float, float]:
    """The shortest and longest arm extension Rookery's routes ask for, from the
    rail line: the exit slot with the hook backed off by PRE, and the far row
    with the handle over mid-finger (crates/mast kinematics.rs and route.rs)."""
    rail = meta["derived"]["rail_line_x"]
    finger = meta["hook_finger"]
    pre = finger / 2 + 0.015
    rows, pitch = meta["rows"], meta["cell_pitch"]
    far_row_x = (rows - 1) / 2 * pitch
    exit_x = meta["derived"]["exit_pose"][0]
    return exit_x - rail - finger / 2 - pre, far_row_x - rail - finger / 2


@pytest.mark.parametrize("preset,scene", CASES)
def test_the_preset_is_a_valid_crane(preset, scene):
    load(preset, scene)  # CraneSpec refuses an impossible preset, listing every reason


@pytest.mark.parametrize("preset,scene", CASES)
def test_boxes_handles_and_grid_match_the_scene(preset, scene):
    s, m = load(preset, scene)
    assert m["box_size"][0] == m["box_size"][1] == pytest.approx(s.box_size)
    assert m["box_size"][2] == pytest.approx(s.box_height)
    assert m["handle_height"] == pytest.approx(s.handle_height)
    assert m["handle_dia"] == pytest.approx(s.handle_dia)
    assert m["cell_pitch"] == pytest.approx(s.cell_pitch)
    assert (m["rows"], m["cols"]) == (s.cells_x, s.cells_y)
    assert m["trays"] == len(s.tier_floors)
    assert m["water_depth"] == pytest.approx(s.water_depth)


@pytest.mark.parametrize("preset,scene", CASES)
def test_the_fork_reaches_every_cell_and_the_exit(preset, scene):
    s, m = load(preset, scene)
    lo, hi = (x - s.mast_x for x in s.arm_x_range)
    near, far = needed_reach(m)
    assert lo <= near + TOL, f"folded, the hook is {lo:.3f} m out; the exit needs {near:.3f} m"
    assert hi >= far - TOL, f"the fork reaches {hi:.3f} m; the far row needs {far:.3f} m"


@pytest.mark.parametrize("preset,scene", CASES)
def test_the_folded_fork_overhangs_less_than_the_old_arm(preset, scene):
    s, _ = load(preset, scene)
    assert s.fork_overhang < OLD_ARM_OVERHANG


@pytest.mark.xfail(strict=True, reason="rack3 is regenerated with shelf_gap 0.17 in row 2.4 (fit study D4)")
def test_rack3_shelf_pitch_matches_the_scene():
    s, m = load("rookery_rack3.toml", "rack3.json")
    floors = [f - s.tier_floors[0] for f in s.tier_floors]
    assert m["derived"]["shelf_bases"] == pytest.approx(floors)


@pytest.mark.xfail(strict=True, reason=(
    "with Rookery's 120 mm boxes and 12 mm handles the J-hook sets the box 8-33 mm off "
    "the exit slot (it slides along the handle); fit study D5: give the crane Rookery's 30 mm saddle"))
@pytest.mark.parametrize("preset,scene,cell", [
    ("rookery_rack3.toml", "rack3.json", (0, 0, 0)),
    ("rookery_rack3.toml", "rack3.json", (2, 0, 3)),
    ("rookery_tray.toml", "rookery.json", (0, 3, 3)),
])
def test_the_hook_places_a_rookery_box_within_5_mm(preset, scene, cell, tmp_path):
    import mujoco

    from crane.builder import build_crane
    from crane.motion import pick, place_on_exit
    from crane.simcheck import reset_home, run
    from crane.visitors.mjcf import write_models

    s, _ = load(preset, scene)
    world, reg = build_crane(s)
    m = mujoco.MjModel.from_xml_path(str(write_models(s, world, reg, tmp_path)["preview"]))
    d = mujoco.MjData(m)
    reset_home(m, d)
    speed = {"rail_y": s.vmax_rail, "mast_z": s.vmax_mast, "arm_x": s.vmax_arm}
    t, r, c = cell
    run(m, d, pick(s, t, r, c), speed)
    run(m, d, place_on_exit(s, 0, t), speed)
    end = d.body(f"box_{t}_{r}_{c}").xpos
    goal = (s.exit_x(0), s.exit_y, s.exit_z)
    assert max(abs(a - b) for a, b in zip(end, goal)) <= 0.005
