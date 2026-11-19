"""Show the crane in the OCP CAD Viewer (the usual build123d viewer).

Pattern: Facade. `show_crane()` hides the spec loading, the builder, the pose
lookup and the ViewerVisitor behind one call, so show.py stays three lines.

Poses:
    home                 (default) the pose after homing
    pick:T,R,C           box at tier T, row R, column C lifted clear of the tub
    engage:T,R,C         hook under that box's handle, before the lift
    rail_y=0.3 mast_z=0.5 arm_x=0.2    any joints, in metres (world frame)
"""
from __future__ import annotations

from pathlib import Path

import crane.parts  # noqa: F401
from crane.builder import build_crane
from crane.motion import pick
from crane.spec import CraneSpec
from crane.visitors.viewer import ViewerVisitor


def parse_pose(spec: CraneSpec, args: list[str] | None) -> dict[str, float]:
    pose = dict(spec.home)
    for a in args or []:
        if a == "home":
            continue
        if ":" in a:
            kind, cell = a.split(":", 1)
            t, r, c = (int(v) for v in cell.split(","))
            moves = list(pick(spec, t, r, c))
            upto = {"pick": len(moves), "engage": 4}[kind]
            for m in moves[:upto]:
                pose.update(m.targets)
        else:
            k, v = a.split("=")
            pose[k] = float(v)
    return pose


def assembly(spec_path: str | Path, pose_args: list[str] | None = None, rack: bool = True):
    spec = CraneSpec.from_toml(spec_path)
    world, _ = build_crane(spec, with_rack=rack)
    v = ViewerVisitor(pose=parse_pose(spec, pose_args))
    world.accept(v)
    return v.result


def _viewer_running(port: int = 3939) -> bool:
    """The OCP CAD Viewer listens on localhost:3939 by default."""
    import os
    import socket
    port = int(os.environ.get("OCP_PORT", port))
    with socket.socket() as sock:
        sock.settimeout(0.5)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def show_crane(spec_path: str | Path, pose_args: list[str] | None = None, rack: bool = True) -> None:
    comp = assembly(spec_path, pose_args, rack)
    try:
        if not _viewer_running():
            raise ConnectionError("nothing is listening on port 3939")
        from ocp_vscode import show
        show(comp)
        print("Sent to the OCP CAD Viewer.")
    except Exception as e:  # viewer not installed or not running
        from build123d import export_gltf
        out = Path(spec_path).resolve().parent / "out" / "rookery_crane.glb"
        out.parent.mkdir(parents=True, exist_ok=True)
        export_gltf(comp, str(out), binary=True)
        print(f"OCP CAD Viewer not reachable ({e.__class__.__name__}: {e}).")
        print(f"Wrote {out} instead; open it in any glTF viewer, or start the viewer and rerun.")


# ---------------------------------------------------------------- one part
#
# Pattern: Facade (again). `show_part()` hides where a single part comes from
# behind one call. It walks the assembly with PartsKitVisitor (no files
# written) to find the part by its key, so the name you type is the same name
# as the STEP file in out/parts/ and the row in index.html. Building it live
# from the code (instead of reading the STEP) means an edit to crane.toml
# shows up straight away. `from_step=True` opens the exported file instead,
# which proves the file itself is good.
#
#   show_part.py ──► show_part() «Facade»
#                        ├── build_crane() + PartsKitVisitor  find the Part by key
#                        ├── import_step()                    (from_step=True)
#                        └── _send()  OCP CAD Viewer, or a .glb next to the STEP

def part_records(spec_path: str | Path):
    from crane.visitors.parts_kit import PartsKitVisitor
    spec = CraneSpec.from_toml(spec_path)
    world, _ = build_crane(spec)
    kit = PartsKitVisitor(Path("."), exporters=[])    # collect only, writes nothing
    world.accept(kit)
    return kit.sorted_records()


def _send(shape, label: str, fallback: Path) -> None:
    try:
        if not _viewer_running():
            raise ConnectionError("nothing is listening on port 3939")
        from ocp_vscode import show
        show(shape, names=[label])
        print(f"Sent {label} to the OCP CAD Viewer.")
    except Exception as e:
        from build123d import export_gltf
        fallback.parent.mkdir(parents=True, exist_ok=True)
        export_gltf(shape, str(fallback), binary=True)
        print(f"OCP CAD Viewer not reachable ({e.__class__.__name__}: {e}).")
        print(f"Wrote {fallback}; open it in any glTF viewer, or start the viewer and rerun.")


def show_part(spec_path: str | Path, key: str, from_step: bool = False, print_pose: bool = False) -> None:
    from difflib import get_close_matches

    from build123d import Color, import_step

    recs = {r.key: r for r in part_records(spec_path)}
    if key not in recs:
        near = get_close_matches(key, recs, n=5, cutoff=0.4)
        hint = f" Did you mean: {', '.join(near)}?" if near else ""
        raise SystemExit(f"No part called {key!r}.{hint} Run `python show_part.py --list` to see them all.")
    rec = recs[key]
    out = Path(spec_path).resolve().parent / "out" / "parts" / rec.kind
    if from_step:
        step = out / f"{key}.step"
        if not step.exists():
            raise SystemExit(f"{step} is missing. Run `python -m crane parts` first.")
        shape = import_step(str(step))
    elif print_pose and rec.kind == "printed":
        shape = rec.part.print_shape()
    else:
        shape = rec.part.shape()
    shape.color = Color(*rec.part.color)
    _send(shape, key, out / f"{key}.glb")
