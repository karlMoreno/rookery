"""Phase 2 gate: the Scene Builder's output is a sound, deterministic MJCF.

Row 2.2. CI runs it from
``scene/`` with ``uv run pytest ../tests/gates -q``.

Generate ``--trays 1 --cells 4x4 --shelf-gap 0.08`` into a temp dir, then assert:
  1. ``mujoco.MjModel.from_xml_path`` loads it;
  2. bodies named ``box_*`` number 16, exactly one ``box_r_c`` per cell;
  3. every ``handle_*`` geom's world z is above the water plane's top;
  4. generating twice gives byte-identical files (same sha256).

It drives the builder only through its CLI (``python -m scene.cli``), the
same entry point a person or CI uses. So passing it proves the shipped
command, not some test-only path into the builder.

```text
  phase2_scene ──subprocess──▶ python -m scene.cli --out tmp/a/rookery.xml
                                               (again into tmp/b)
        │
        ├─ MjModel.from_xml_path(a)       → 1. parses
        ├─ names box_*                    → 2. == {box_r_c : r, c in 0..3}
        ├─ mj_forward; geom_xpos[handle_*] → 3. z > water top
        └─ sha256(a) == sha256(b)         → 4. deterministic
```

Contract this gate fixes for rows 2.3–2.5: body ``box_{r}_{c}``, geom
``handle_{r}_{c}``, and one geom named ``water`` (a box whose top face is
the water plane).

Red on ``main`` from row 2.2 (the CLI writes nothing yet). Green once rows
2.3–2.5 land.
"""

from __future__ import annotations

import hashlib
import re
import subprocess
import sys
from pathlib import Path

import mujoco
import pytest

# Absolute, so the gate works from any cwd.
SCENE_DIR = Path(__file__).resolve().parents[2] / "scene"
ARGS = ["--trays", "1", "--cells", "4x4", "--shelf-gap", "0.08"]
ROWS, COLS = 4, 4


def generate(out_dir: Path) -> Path:
    """Run the scene CLI into ``out_dir`` and return the MJCF path."""
    out = out_dir / "rookery.xml"
    subprocess.run(
        [sys.executable, "-m", "scene.cli", *ARGS, "--out", str(out)],
        cwd=SCENE_DIR,
        check=True,
    )
    assert out.is_file(), f"scene.cli exited 0 but wrote no {out}"
    return out


@pytest.fixture(scope="module")
def scene(tmp_path_factory) -> Path:
    return generate(tmp_path_factory.mktemp("a"))


@pytest.fixture(scope="module")
def model(scene) -> mujoco.MjModel:
    # Assertion 1: the generated MJCF parses.
    return mujoco.MjModel.from_xml_path(str(scene))


def names(model: mujoco.MjModel, obj: mujoco.mjtObj, count: int) -> list[str]:
    return [mujoco.mj_id2name(model, obj, i) or "" for i in range(count)]


def test_mjcf_parses(model):
    assert model.nbody > 1


def test_one_box_per_cell(model):
    boxes = [n for n in names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody) if n.startswith("box_")]
    expected = {f"box_{r}_{c}" for r in range(ROWS) for c in range(COLS)}
    assert len(boxes) == ROWS * COLS, f"{len(boxes)} box_* bodies: {sorted(boxes)}"
    assert set(boxes) == expected, f"missing {expected - set(boxes)}, extra {set(boxes) - expected}"


def test_every_handle_is_above_the_water(model):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # fills geom_xpos at the initial pose

    water = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "water")
    assert water >= 0, "no geom named `water`"
    water_top = data.geom_xpos[water][2] + model.geom_size[water][2]

    geom_names = names(model, mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    handles = {n: i for i, n in enumerate(geom_names) if re.fullmatch(r"handle_\d+_\d+", n)}
    assert len(handles) == ROWS * COLS, f"{len(handles)} handle_* geoms"
    low = {n: float(data.geom_xpos[i][2]) for n, i in handles.items() if data.geom_xpos[i][2] <= water_top}
    assert not low, f"handles at or below the water top {water_top:.6f}: {low}"


def test_regeneration_is_byte_identical(scene, tmp_path):
    again = generate(tmp_path)
    a = hashlib.sha256(scene.read_bytes()).hexdigest()
    b = hashlib.sha256(again.read_bytes()).hexdigest()
    assert a == b, f"same arguments, different output: {a} vs {b}"
