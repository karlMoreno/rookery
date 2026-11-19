"""Tests: the spec refuses bad numbers, every printed part fits the bed, the
MuJoCo model loads with Rookery's joint names, and the hook really lifts a box.

Run:  python -m pytest -q
"""
from __future__ import annotations

import dataclasses
from pathlib import Path

import mujoco
import pytest

import crane.parts  # noqa: F401
from crane.builder import build_crane
from crane.motion import pick, place_on_exit
from crane.parts.factory import PartFactory
from crane.simcheck import reset_home, run
from crane.spec import CraneSpec, SpecError
from crane.tree import PartNode
from crane.visitors.bom import BomVisitor
from crane.visitors.klipper import KlipperVisitor
from crane.visitors.mjcf import write_models

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def spec() -> CraneSpec:
    return CraneSpec.from_toml(ROOT / "crane.toml")


@pytest.fixture(scope="module")
def built(spec):
    return build_crane(spec)


@pytest.fixture(scope="module")
def models(spec, built, tmp_path_factory):
    world, reg = built
    return write_models(spec, world, reg, tmp_path_factory.mktemp("mj"))


# ------------------------------------------------------------------ spec
def test_toml_matches_defaults(spec):
    assert spec == CraneSpec(), "crane.toml and CraneSpec defaults have drifted apart"


def test_spec_reports_every_problem_at_once():
    with pytest.raises(SpecError) as e:
        CraneSpec(mast_length=0.6, fork_length=0.2, vmax_mast=0.5)
    msg = str(e.value)
    assert "mast only reaches" in msg and "arm reach" in msg and "lead screw" in msg


def test_unknown_toml_key_is_an_error(tmp_path):
    p = tmp_path / "bad.toml"
    p.write_text("[crane]\nmast_lenght = 0.9\n")
    with pytest.raises(SpecError, match="mast_lenght"):
        CraneSpec.from_toml(p)


def test_every_cell_and_exit_slot_is_reachable(spec):
    (ylo, yhi), (zlo, zhi), (xlo, xhi) = spec.rail_y_range, spec.mast_z_range, spec.arm_x_range
    for t in range(len(spec.tier_floors)):
        assert zlo <= spec.engage_z(t) and spec.carry_z(t) <= zhi
    for r in range(spec.cells_x):
        assert xlo <= spec.cell_x(r) <= xhi
    for c in range(spec.cells_y):
        assert ylo <= spec.cell_y(c) <= yhi
    assert ylo <= spec.exit_y <= yhi


# ------------------------------------------------------------------ parts
@pytest.mark.parametrize("key", PartFactory.printed_keys())
def test_printed_part_is_valid_and_fits_the_bed(spec, key):
    p = PartFactory(spec).printed(key)
    assert p.shape().is_valid
    fx, fy, fz = p.footprint_mm
    assert fx <= spec.printable_mm and fy <= spec.printable_mm and fz <= spec.bed_z_mm


def test_small_bed_is_enforced():
    from crane.parts.base import BedFitError
    small = dataclasses.replace(CraneSpec(), bed_x_mm=120, bed_y_mm=120)
    with pytest.raises(BedFitError):
        PartFactory(small).printed("truck_plate").shape()


def test_mesh_boxes_share_one_solid(built):
    world, _ = built
    boxes = [n.part for n in world.walk() if isinstance(n, PartNode) and n.part.key == "mesh_box"]
    assert len(boxes) == 16 and len({id(b) for b in boxes}) == 1      # Flyweight


# ------------------------------------------------------------------ outputs
def test_mjcf_loads_with_rookery_joint_names(models, spec):
    m = mujoco.MjModel.from_xml_path(str(models["crane"]))
    names = [m.joint(i).name for i in range(m.njnt)]
    assert names[:4] == ["rail_y", "mast_z", "arm_x", "lift"]      # Rookery's indices unchanged
    assert names[4:] == ["arm_mid"]                                 # fork middle stage, not driven
    assert m.nu == 3
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    for j, v in spec.home.items():
        assert d.joint(j).qpos[0] == pytest.approx(v)                # ref = home


def test_hook_cup_is_at_the_joint_values(models, spec):
    m = mujoco.MjModel.from_xml_path(str(models["crane"]))
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    foot = d.geom("hook_foot")
    h = spec.home
    assert foot.xpos[1] == pytest.approx(h["rail_y"], abs=1e-6)
    assert foot.xpos[2] + spec.hook_section / 2 == pytest.approx(h["mast_z"], abs=1e-6)


def test_registry_has_the_trimmed_sensor_set(built):
    _, reg = built
    assert {m.name for m in reg.of_kind("endstop")} == {"x_home", "y_home", "z_home"}
    assert sorted(m.name for m in reg.of_kind("camera")) == ["cam_hook", "cam_tier_0"]
    assert reg["cam_hook"].body == "arm"                       # rides out with the hook
    assert [m.name for m in reg.of_kind("sensor_node")] == ["water_node_0"]


def test_bom_and_klipper(spec, built):
    world, reg = built
    bom = BomVisitor()
    world.accept(bom)
    keys = {r["key"] for r in bom.rows()}
    assert {"nema17", "nema14", "skr_pico", "pi5", "pi_camera3_wide", "esp32c3", "petg"} <= keys
    k = KlipperVisitor(spec, reg)
    world.accept(k)
    cfg = k.text()
    assert f"position_max: {spec.mast_z_range[1] * 1000:.2f}" in cfg


# ------------------------------------------------------------------ physics
def test_hook_lifts_a_box_and_sets_it_on_the_exit_tray(models, spec):
    m = mujoco.MjModel.from_xml_path(str(models["preview"]))
    d = mujoco.MjData(m)
    reset_home(m, d)
    box = "box_0_1_2"
    z0 = float(d.body(box).xpos[2])
    speed = {"rail_y": spec.vmax_rail, "mast_z": spec.vmax_mast, "arm_x": spec.vmax_arm}
    run(m, d, pick(spec, 0, 1, 2), speed)
    assert d.body(box).xpos[2] - z0 > 0.07, "box didn't leave the tub"
    run(m, d, place_on_exit(spec, 0, 0), speed)
    x, y, z = d.body(box).xpos
    assert abs(x - spec.exit_x(0)) < 0.008 and abs(y - spec.exit_y) < 0.008
    assert z == pytest.approx(spec.exit_z, abs=0.003)


# ---- parts kit: one CAD file per piece ----

def test_parts_kit_matches_the_assembly(built, tmp_path):
    """Every printed part in the print set is in the kit, counts agree with the
    STL visitor, and every STEP file reads back with the part's own volume."""
    from build123d import import_step
    from crane.visitors.parts_kit import PartsKitVisitor
    from crane.visitors.stl import StlVisitor

    world, _ = built
    kit = PartsKitVisitor(tmp_path, log=lambda *_: None)
    world.accept(kit)
    stl = StlVisitor(tmp_path / "stl")
    world.accept(stl)
    printed = {k: r.qty for k, r in kit.records.items() if r.kind == "printed"}
    assert printed == dict(stl.counts)
    assert sum(1 for r in kit.records.values() if r.kind == "bought") > 20
    kit.finish()
    for key in ("truck_cheek", "nema17"):
        rec = kit.records[key]
        back = import_step(str(tmp_path / rec.files["step"]))
        assert back.volume == pytest.approx(rec.part.shape().volume, rel=1e-3)
    assert (tmp_path / "index.html").exists() and (tmp_path / "parts.json").exists()


# ------------------------------------------------------------------ telescopic fork
def test_folded_fork_is_compact_and_clear_of_the_rack(spec):
    assert spec.fork_front_home + 0.016 < spec.rack_x0 - 0.020          # folds clear of the rack posts
    assert spec.fork_overhang < 0.25                                   # was about 0.52 m with the 0.6 m arm
    assert spec.arm_x_range[1] - spec.arm_x_range[0] > 1.35 * spec.fork_length   # reach well beyond its length


def test_middle_stage_follows_at_half_speed(models, spec):
    m = mujoco.MjModel.from_xml_path(str(models["preview"]))
    d = mujoco.MjData(m)
    reset_home(m, d)
    speed = {"rail_y": spec.vmax_rail, "mast_z": spec.vmax_mast, "arm_x": spec.vmax_arm}
    far = spec.arm_x_range[1] - 0.002
    from crane.motion import MacroCommand, MoveCommand
    run(m, d, MacroCommand("reach", [MoveCommand("out", {"arm_x": far}, dwell=0.5)]), speed)
    h = spec.home["arm_x"]
    assert d.joint("arm_mid").qpos[0] - h == pytest.approx((far - h) / 2, abs=0.002)


def test_klipper_x_counts_the_2_to_1_belts(spec, built):
    world, reg = built
    k = KlipperVisitor(spec, reg)
    world.accept(k)
    assert f"rotation_distance: {4 * spec.belt_pulley_teeth}" in k.text().split("[stepper_x]")[1].split("[")[0]


# ------------------------------------------------------------------ hook camera
def test_hook_camera_passes_over_handles_and_is_in_the_model(spec, models):
    mount = PartFactory(spec).printed("hook_camera_mount")
    lowest_above_cup = mount.shape().bounding_box().min.Z / 1000 + spec.hook_shank
    handle_top_at_engage = spec.engage_drop + spec.handle_dia + 0.003     # bar plus its posts
    assert lowest_above_cup - handle_top_at_engage > 0.003
    m = mujoco.MjModel.from_xml_path(str(models["crane"]))
    cam = m.camera("cam_hook")
    assert m.body(cam.bodyid[0]).name == "arm"


# ------------------------------------------------------------------ committed BOM
def test_committed_bom_matches_the_spec(built, tmp_path):
    """bom.csv is committed so the parts list is readable on GitHub. It must be
    what crane.toml builds today: rerun `python -m crane bom` after a change."""
    world, _ = built
    bom = BomVisitor()
    world.accept(bom)
    fresh = bom.write(tmp_path / "bom.csv").read_text()
    assert fresh == (ROOT / "bom.csv").read_text(), (
        "bom.csv is stale: run `python -m crane bom` and commit it"
    )
