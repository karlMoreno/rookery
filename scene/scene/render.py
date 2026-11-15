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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="scene.render", description=__doc__.splitlines()[0])
    ap.add_argument("--scene", type=Path, help="MJCF to load (default: build the default SceneSpec)")
    ap.add_argument("--frame", type=int, default=0, help="physics steps before rendering")
    ap.add_argument("--out", type=Path, required=True, help="PNG to write")
    args = ap.parse_args(argv)

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
