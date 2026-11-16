"""Row 2.7: --preset drop reproduces the Phase 1 scene, and --check enforces rule 7."""

from pathlib import Path

import mujoco
import pytest

from scene import SceneSpec
from scene.builder import build_scene, spec_from_json, spec_json
from scene.cli import main

REPO = Path(__file__).resolve().parents[2]


def test_drop_preset_matches_the_phase1_scene():
    m = mujoco.MjModel.from_xml_string(build_scene(SceneSpec(preset="drop")))
    assert m.nbody == 2
    assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, 1) == "box"
    assert m.opt.timestep == pytest.approx(0.001)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert d.xpos[1][2] == pytest.approx(0.5)
    for _ in range(2000):  # the Phase 1 gate, in Python
        mujoco.mj_step(m, d)
    assert abs(d.xpos[1][2] - 0.05) < 0.001


def test_drop_preset_is_physically_the_same_as_the_hand_written_file():
    hand = REPO / "examples" / "drop.xml"
    if not hand.is_file():
        pytest.skip("examples/drop.xml already retired")
    a = mujoco.MjModel.from_xml_path(str(hand))
    b = mujoco.MjModel.from_xml_string(build_scene(SceneSpec(preset="drop")))
    for field in ("nbody", "ngeom", "njnt", "nq"):
        assert getattr(a, field) == getattr(b, field), field
    assert a.geom_size.ravel().tolist() == pytest.approx(b.geom_size.ravel().tolist())
    assert a.body_pos.ravel().tolist() == pytest.approx(b.body_pos.ravel().tolist())
    assert a.body_mass.tolist() == pytest.approx(b.body_mass.tolist())


def test_cli_preset_defaults_to_its_own_file_name(tmp_path, monkeypatch):
    out = tmp_path / "drop.xml"
    assert main(["--preset", "drop", "--out", str(out)]) == 0
    assert out.is_file() and out.with_suffix(".json").is_file()


def test_json_round_trips_to_the_same_spec():
    for spec in (SceneSpec(), SceneSpec(rows=3, cols=5), SceneSpec(preset="drop"),
                 SceneSpec(exit_tray_pose=(-0.5, 0.1, 0.2), rail_x=-0.8)):
        assert spec_from_json(spec_json(spec)) == spec


@pytest.fixture
def generated(tmp_path):
    for args in (["--trays", "1", "--cells", "4x4", "--shelf-gap", "0.08",
                  "--out", str(tmp_path / "rookery.xml")],
                 ["--preset", "drop", "--out", str(tmp_path / "drop.xml")]):
        assert main(args) == 0
    return tmp_path


def test_check_passes_on_fresh_output(generated, capsys):
    assert main(["--check", "--dir", str(generated)]) == 0
    assert "2 scene(s) up to date" in capsys.readouterr().out


def test_check_catches_a_hand_edit(generated, capsys):
    xml = generated / "rookery.xml"
    xml.write_text(xml.read_text().replace("0.050000", "0.060000", 1))
    assert main(["--check", "--dir", str(generated)]) == 1
    assert "rookery.xml: differs" in capsys.readouterr().err


def test_check_catches_a_missing_xml_and_a_bad_recipe(generated, capsys):
    (generated / "drop.xml").unlink()
    (generated / "rookery.json").write_text('{"rows": 0}')
    assert main(["--check", "--dir", str(generated)]) == 1
    err = capsys.readouterr().err
    assert "drop.xml: differs" in err and "rookery.json: not a valid recipe" in err


def test_check_fails_on_an_empty_directory(tmp_path):
    assert main(["--check", "--dir", str(tmp_path)]) == 1


def test_committed_generated_scenes_are_up_to_date():
    """The real thing: what is in scene/generated/ matches the builder."""
    gen = REPO / "scene" / "generated"
    if not any(gen.glob("*.json")):
        pytest.skip("nothing generated yet")
    assert main(["--check"]) == 0
