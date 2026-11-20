"""The committed look for Rookery's scene (meshes/rookery/) must be what the
preset builds today. Rookery's scene builder hangs these meshes on its crane;
rerun `python -m crane --spec presets/rookery_tray.toml look` after a change
to the crane or the preset, then regenerate Rookery's scenes."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from crane.builder import build_crane
from crane.look import LOOK_PRESET, write_look
from crane.spec import CraneSpec

ROOT = Path(__file__).resolve().parent.parent
COMMITTED = ROOT / "meshes" / "rookery"


def test_committed_look_matches_the_preset(tmp_path):
    s = CraneSpec.from_toml(ROOT / "presets" / LOOK_PRESET)
    world, reg = build_crane(s)
    write_look(s, world, reg, tmp_path)
    fresh = sorted(p.relative_to(tmp_path) for p in tmp_path.rglob("*") if p.is_file())
    kept = sorted(p.relative_to(COMMITTED) for p in COMMITTED.rglob("*") if p.is_file())
    assert fresh == kept, "meshes/rookery/ has different files: rerun `python -m crane look` and commit"
    # look.xml and anchor.json byte for byte. OBJ bytes differ between macOS
    # and Linux (OpenCascade tessellates with different float rounding), so a
    # mesh only has to keep its extent to 0.1 mm and its size to within 10 %.
    stale = []
    for p in fresh:
        a, b = (tmp_path / p).read_bytes(), (COMMITTED / p).read_bytes()
        if p.suffix != ".obj":
            stale += [] if a == b else [str(p)]
            continue
        (alo, ahi, an), (blo, bhi, bn) = _extent(a), _extent(b)
        if np.abs(alo - blo).max() > 0.1 or np.abs(ahi - bhi).max() > 0.1 or abs(an - bn) > 0.1 * bn:
            stale.append(str(p))
    assert not stale, f"meshes/rookery/ is stale ({', '.join(stale)}): rerun `python -m crane look` and commit"


def _extent(obj: bytes) -> tuple[np.ndarray, np.ndarray, int]:
    """Bounding box (mm) and vertex count of an OBJ."""
    v = np.array([[float(x) for x in line.split()[1:4]] for line in obj.decode().splitlines() if line.startswith("v ")])
    return v.min(0), v.max(0), len(v)


def test_the_anchor_names_the_preset_and_a_seat_on_the_hook():
    a = json.loads((COMMITTED / "anchor.json").read_text())
    assert a["preset"] == LOOK_PRESET
    assert len(a["hook_seat"]) == 3
    assert abs(a["hook_seat"][0]) < 0.02 and abs(a["hook_seat"][2]) < 0.02
