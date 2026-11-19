"""Row 12.3: the sweep files and the bench driver's plumbing (the runs
themselves are the clean-clone CI job)."""

import tomllib
from pathlib import Path

from scene import bench
from scene.spec import SceneSpec

REPO = Path(__file__).resolve().parents[2]


def test_the_small_sweep_has_ten_points_and_every_scene_builds(tmp_path):
    sweep = tomllib.loads((REPO / "bench/small.toml").read_text())
    pts = bench.points(sweep)
    assert len(pts) == 10 and all(p["days"] == 7.0 and p["seed"] == 7 for p in pts)
    names = {bench.scene_name(p) for p in pts}
    assert names == {"tray_1x4", "tray_2x4", "rack3_1x4_gap0.15"}
    for p in pts:
        xml = bench.build(p, tmp_path)
        assert xml.exists() and xml.with_suffix(".json").exists()


def test_the_large_sweep_runs_every_point_on_four_seeds():
    sweep = tomllib.loads((REPO / "bench/large.toml").read_text())
    pts = bench.points(sweep)
    assert len(pts) == 4 * len(sweep["point"]) and {p["seed"] for p in pts} == {7, 11, 23, 42}
    for p in sweep["point"]:
        rows, cols = (int(v) for v in p["cells"].split("x"))
        SceneSpec(trays=p.get("trays", 1), rows=rows, cols=cols, shelf_gap=p.get("shelf_gap", 0.08))
