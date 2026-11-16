"""Render one frame of a scene to a PNG, offscreen, with no window.

    uv run python -m scene.render --frame 0 --out tray.png
    uv run python -m scene.render --scene path/to/scene.xml --frame 100 --out f100.png

``--frame N`` steps the physics N times from the initial state and renders
camera ``overview`` (the free camera if the scene has none). Headless: on
macOS it uses CGL (``MUJOCO_GL=cgl``), so no window ever opens. Elsewhere set
``MUJOCO_GL`` (``egl`` or ``osmesa``) for a headless machine. Every path is resolved to an absolute
path first, because GLFW on macOS changes the cwd if a window does open.

With no ``--scene``, it builds the default ``SceneSpec`` in memory, so you
can render without writing anything to ``scene/generated/``.

Replaying a run logged by ``rookery run`` (row 3.5):

    uv run python -m scene.render --run tour-1763262000 --frame -1 --out last.png   # headless
    uv run mjpython -m scene.render --run tour-1763262000 --live                     # passive viewer

``--run`` reads ``<runs>/<id>/summary.json`` (for the scene) and ``qpos.csv``
(one row per 50 Hz tick). ``--frame`` then indexes the ticks (``-1`` is the
last one). ``--live`` plays them in the MuJoCo passive viewer at real time,
which needs ``mjpython`` on macOS and opens a window.
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import zlib
from pathlib import Path

# Pick the offscreen GL backend before mujoco is imported. Only macOS gets a
# default: forcing `egl` where libEGL is missing breaks `import mujoco` itself.
if sys.platform == "darwin":
    os.environ.setdefault("MUJOCO_GL", "cgl")

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from .builder import build_scene  # noqa: E402
from .spec import SceneSpec  # noqa: E402


def write_png(path: Path, rgb: np.ndarray) -> None:
    """Minimal PNG writer (8-bit RGB) so rendering needs only numpy."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def render_frame(model: mujoco.MjModel, frame: int, width: int = 640, height: int = 480) -> np.ndarray:
    """RGB array of the scene after ``frame`` steps."""
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for _ in range(frame):
        mujoco.mj_step(model, data)
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    with mujoco.Renderer(model, height=height, width=width) as r:
        r.update_scene(data, camera=cam if cam >= 0 else -1)
        return r.render()


#: Where ``rookery run`` writes logs by default (relative paths resolve from the repo root).
REPO = Path(__file__).resolve().parents[2]


def load_run(run: str, runs: Path) -> tuple[mujoco.MjModel, list[float], list[list[float]]]:
    """Model, tick times and qpos rows of a logged run."""
    import csv
    import json

    d = (runs if runs.is_absolute() else REPO / runs) / run
    summary = json.loads((d / "summary.json").read_text())
    scene = Path(summary["scene"])
    scene = scene if scene.is_absolute() else REPO / scene
    model = mujoco.MjModel.from_xml_path(str(scene.resolve()))
    with open(d / "qpos.csv", newline="") as f:
        rows = [list(map(float, r)) for i, r in enumerate(csv.reader(f)) if i > 0]
    if not rows or len(rows[0]) - 1 != model.nq:
        raise SystemExit(f"render: {d}/qpos.csv does not match the scene's nq = {model.nq}")
    return model, [r[0] for r in rows], [r[1:] for r in rows]


def render_qpos(model: mujoco.MjModel, qpos: list[float], width: int = 640, height: int = 480) -> np.ndarray:
    """RGB array of the scene posed at ``qpos`` (no physics step)."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "overview")
    with mujoco.Renderer(model, height=height, width=width) as r:
        r.update_scene(data, camera=cam if cam >= 0 else -1)
        return r.render()


def replay_live(model: mujoco.MjModel, times: list[float], rows: list[list[float]]) -> None:
    """Play the logged ticks in the passive viewer at real time (opens a window)."""
    import time

    import mujoco.viewer

    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as v:
        t0 = time.monotonic()
        for t, q in zip(times, rows, strict=True):
            if not v.is_running():
                break
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            v.sync()
            time.sleep(max(0.0, t - times[0] - (time.monotonic() - t0)))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="scene.render", description=__doc__.splitlines()[0])
    ap.add_argument("--scene", type=Path, help="MJCF to load (default: build the default SceneSpec)")
    ap.add_argument("--frame", type=int, default=0, help="physics steps before rendering")
    ap.add_argument("--out", type=Path, help="PNG to write")
    ap.add_argument("--run", help="replay a run id logged by `rookery run`")
    ap.add_argument("--runs", type=Path, default=Path("runs"), help="run log directory (default runs/)")
    ap.add_argument("--live", action="store_true", help="with --run: play it in the passive viewer")
    args = ap.parse_args(argv)

    if args.run is not None:
        model, times, rows = load_run(args.run, args.runs)
        if args.live:
            replay_live(model, times, rows)
            return 0
        if args.out is None:
            ap.error("--run needs --out (or --live)")
        out = args.out.resolve()
        write_png(out, render_qpos(model, rows[args.frame]))
        print(f"render: wrote {out} (run {args.run}, tick {args.frame % len(rows)} of {len(rows)})")
        return 0
    if args.out is None:
        ap.error("--out is required")
    out = args.out.resolve()
    if args.scene is not None:
        model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    else:
        model = mujoco.MjModel.from_xml_string(build_scene(SceneSpec()))
    write_png(out, render_frame(model, args.frame))
    print(f"render: wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
