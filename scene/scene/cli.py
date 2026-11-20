"""Command line for the scene Builder: ``uv run python -m scene.cli --help``.

The CLI only turns flags into a ``SceneSpec`` and hands it to the builder,
so every check lives in the spec, and the CLI and the tests can't disagree.
It writes the MJCF to ``--out`` and the spec as JSON beside it
(``rookery.xml`` → ``rookery.json``). The JSON is the recipe: ``--check``
rebuilds every ``generated/*.xml`` from it and fails on any difference.

    uv run python -m scene.cli --trays 1 --cells 4x4 --shelf-gap 0.08 --crane-look   # generated/rookery.xml
    uv run python -m scene.cli --preset drop                            # generated/drop.xml
    uv run python -m scene.cli --check
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .builder import build_scene, spec_from_json, spec_json
from .spec import PRESETS, SceneSpec, SpecError

#: Where the farm scene goes by default. Resolved against this file rather
#: than the cwd, because GLFW on macOS changes the cwd once a window opens.
GENERATED = Path(__file__).resolve().parent.parent / "generated"
DEFAULT_OUT = GENERATED / "rookery.xml"
#: The CAD crane's look (hardware/crane, `python -m crane look`), relative to GENERATED.
DEFAULT_LOOK = "../../hardware/crane/meshes/rookery"


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
    ap.add_argument("--crane-look", nargs="?", const=DEFAULT_LOOK, default=None, metavar="DIR",
                    help=f"draw the crane with its CAD meshes (default DIR {DEFAULT_LOOK}, relative to generated/)")
    ap.add_argument("--preset", choices=PRESETS, default=None, help="build a named scene instead")
    ap.add_argument("--out", type=Path, default=None,
                    help="output MJCF path (default generated/rookery.xml, or generated/<preset>.xml)")
    ap.add_argument("--check", action="store_true",
                    help="rebuild every generated/*.xml from the .json beside it; exit 1 on any difference")
    ap.add_argument("--dir", type=Path, default=GENERATED, help=argparse.SUPPRESS)  # --check target, for tests
    return ap


def spec_from_args(args: argparse.Namespace) -> SceneSpec:
    rows, cols = args.cells
    return SceneSpec(trays=args.trays, rows=rows, cols=cols, shelf_gap=args.shelf_gap, preset=args.preset,
                     crane_look=None if args.preset else args.crane_look)


def write_scene(spec: SceneSpec, out: Path) -> Path:
    """Write the MJCF and the JSON recipe beside it, and return the JSON path."""
    out.parent.mkdir(parents=True, exist_ok=True)
    # newline="\n": the same bytes on every OS (gate assertion 4).
    out.write_text(build_scene(spec), encoding="utf-8", newline="\n")
    meta = out.with_suffix(".json")
    meta.write_text(spec_json(spec), encoding="utf-8", newline="\n")
    return meta


def check(directory: Path) -> int:
    """Rule 7, enforced: each committed ``X.json`` must rebuild ``X.xml`` and
    itself byte for byte. A hand edit, or a builder change without a
    regenerate, shows up here as a named file."""
    recipes = sorted(directory.resolve().glob("*.json"))
    if not recipes:
        print(f"scene --check: no *.json recipes in {directory}", file=sys.stderr)
        return 1
    stale: list[str] = []
    for meta in recipes:
        xml = meta.with_suffix(".xml")
        try:
            spec = spec_from_json(meta.read_text(encoding="utf-8"))
        except (ValueError, TypeError) as e:
            stale.append(f"{meta.name}: not a valid recipe ({e})")
            continue
        if not xml.is_file() or xml.read_bytes() != build_scene(spec).encode("utf-8"):
            stale.append(f"{xml.name}: differs from what {meta.name} builds")
        if meta.read_bytes() != spec_json(spec).encode("utf-8"):
            stale.append(f"{meta.name}: not in canonical form")
    if stale:
        print("scene --check: regenerate and commit (rule 7):\n  " + "\n  ".join(stale), file=sys.stderr)
        return 1
    print(f"scene --check: {len(recipes)} scene(s) up to date in {directory.resolve()}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.check:
        return check(args.dir)
    try:
        spec = spec_from_args(args)
    except SpecError as e:
        print(f"scene: {e}", file=sys.stderr)
        return 2
    out = args.out or (GENERATED / f"{spec.preset}.xml" if spec.preset else DEFAULT_OUT)
    out = out.resolve()
    meta = write_scene(spec, out)
    what = f"preset {spec.preset}" if spec.preset else f"{spec.rows}x{spec.cols} cells, {spec.trays} tray(s)"
    print(f"scene: wrote {out} and {meta.name} ({what})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
