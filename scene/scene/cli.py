"""Command line for the scene Builder: ``uv run python -m scene.cli --help``.

The CLI only turns flags into a ``SceneSpec`` and hands it to the builder,
so every check lives in the spec, and the CLI and the tests can't disagree.
From row 2.3 it writes the MJCF to ``--out``. The scene so far is the tray
and the water; boxes come in 2.4 and the exit tray in 2.5.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .builder import build_scene
from .spec import PRESETS, SceneSpec, SpecError

#: Where the farm scene goes by default. Resolved against this file rather
#: than the cwd, because GLFW on macOS changes the cwd once a window opens.
DEFAULT_OUT = (Path(__file__).resolve().parent.parent / "generated" / "rookery.xml")


def parse_cells(text: str) -> tuple[int, int]:
    """``"4x4"`` → ``(4, 4)`` (rows x cols)."""
    try:
        r, c = text.lower().split("x")
        return int(r), int(c)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--cells wants ROWSxCOLS like 4x4, got {text!r}") from None


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="scene", description="Generate a Rookery MJCF scene.")
    ap.add_argument("--trays", type=int, default=SceneSpec.trays, help="stacked trays (default %(default)s)")
    ap.add_argument("--cells", type=parse_cells, default=(SceneSpec.rows, SceneSpec.cols),
                    metavar="RxC", help="cells per tray, rows x cols (default 4x4)")
    ap.add_argument("--shelf-gap", type=float, default=SceneSpec.shelf_gap,
                    help="clear space above the handles, in metres (default %(default)s)")
    ap.add_argument("--preset", choices=PRESETS, default=None, help="build a named scene instead")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="output MJCF path")
    return ap


def spec_from_args(args: argparse.Namespace) -> SceneSpec:
    rows, cols = args.cells
    return SceneSpec(trays=args.trays, rows=rows, cols=cols, shelf_gap=args.shelf_gap, preset=args.preset)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        spec = spec_from_args(args)
    except SpecError as e:
        print(f"scene: {e}", file=sys.stderr)
        return 2
    if spec.preset is not None:
        print(f"scene: preset {spec.preset!r} arrives in row 2.7", file=sys.stderr)
        return 2
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    # newline="\n": the same bytes on every OS (gate assertion 4).
    out.write_text(build_scene(spec), encoding="utf-8", newline="\n")
    print(f"scene: wrote {out} ({spec.rows}x{spec.cols} cells, {spec.trays} tray(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
