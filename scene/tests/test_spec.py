"""Row 2.1: ``SceneSpec`` defaults construct and every invalid case raises."""

import math

import pytest

from scene import SceneSpec, SpecError
from scene.cli import main, parse_cells


def test_defaults_construct():
    s = SceneSpec()
    assert (s.trays, s.rows, s.cols) == (1, 4, 4)
    assert s.cells == 16
    assert s.handle_top_z > s.water_z


def test_spec_is_frozen_and_normalises_sequences():
    s = SceneSpec(box_size=[0.1, 0.1, 0.08])
    assert s.box_size == (0.1, 0.1, 0.08)
    assert s == SceneSpec(box_size=(0.1, 0.1, 0.08))
    with pytest.raises(AttributeError):
        s.rows = 5  # type: ignore[misc]


def test_cell_centers_are_grid_ordered_and_centred():
    s = SceneSpec(rows=2, cols=3, cell_pitch=0.2, box_size=(0.1, 0.1, 0.08))
    assert s.cell_center(0, 0) == pytest.approx((-0.1, -0.2))
    assert s.cell_center(1, 2) == pytest.approx((0.1, 0.2))
    with pytest.raises(IndexError):
        s.cell_center(2, 0)


@pytest.mark.parametrize(
    "kwargs, needle",
    [
        # Plan checks.
        ({"box_size": (0.15, 0.10, 0.08)}, "footprint"),          # equals the pitch
        ({"box_size": (0.10, 0.20, 0.08)}, "footprint"),          # above the pitch
        ({"water_depth": 0.125}, "water plane"),                   # handle top 0.125 == water
        ({"water_depth": 0.20}, "water plane"),                    # handle under water
        # Non-positive dimensions.
        ({"cell_pitch": 0.0}, "cell_pitch"),
        ({"water_depth": -0.01}, "water_depth"),
        ({"handle_height": 0.0}, "handle_height"),
        ({"shelf_gap": -1.0}, "shelf_gap"),
        ({"box_size": (0.1, 0.0, 0.08)}, "box_size[1]"),
        ({"cell_pitch": math.nan}, "cell_pitch"),
        # Counts and shapes.
        ({"rows": 0}, "rows"),
        ({"cols": 2.5}, "cols"),
        ({"trays": True}, "trays"),
        ({"box_size": (0.1, 0.1)}, "box_size"),
        ({"exit_tray_pose": (0.0, 0.0)}, "exit_tray_pose"),
        ({"preset": "farm"}, "preset"),
    ],
)
def test_invalid_spec_raises(kwargs, needle):
    with pytest.raises(SpecError) as e:
        SceneSpec(**kwargs)
    assert any(needle in p for p in e.value.problems), e.value.problems


def test_every_problem_is_reported_at_once():
    with pytest.raises(SpecError) as e:
        SceneSpec(rows=0, cell_pitch=-1.0, shelf_gap=0.0)
    assert len(e.value.problems) == 3


def test_parse_cells():
    assert parse_cells("4x4") == (4, 4)
    assert parse_cells("3X5") == (3, 5)


def test_cli_accepts_the_phase2_demo_args(tmp_path, capsys):
    out = tmp_path / "rookery.xml"
    assert main(["--trays", "1", "--cells", "4x4", "--shelf-gap", "0.08", "--out", str(out)]) == 0
    assert "(4x4 cells, 1 tray(s))" in capsys.readouterr().out
    assert out.read_text().startswith("<mujoco")


def test_cli_rejects_a_bad_spec(capsys):
    assert main(["--cells", "0x4", "--out", "/nonexistent/never-written.xml"]) == 2
    assert "rows must be" in capsys.readouterr().err
