"""Command line: python -m crane <command>

    check    validate crane.toml and print the derived joint ranges
    build    write everything to out/: STLs, BOM, printer.cfg, MuJoCo model, crane.json, STEP, parts kit
    parts    write out/parts/: one STEP + STL + picture per part, and index.html
    bom      rewrite the committed bom.csv from crane.toml (a test fails if it is stale)
    pick     run a pick-and-place in MuJoCo to prove the hook geometry
    show     send the assembly to the OCP CAD Viewer (same as `python show.py`)
    view     open MuJoCo's viewer to drive the joints, or `view --pick` to watch a pick
    look     write meshes/rookery/: the crane meshes Rookery's scene hangs on its crane (presets/rookery_tray.toml)

Pattern: Command (for the CLI itself). Each subcommand is one small function
registered in COMMANDS, so adding a command is adding a function.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import crane.parts  # noqa: F401  (registers printed parts)
from crane.builder import build_crane
from crane.spec import CraneSpec, SpecError

ROOT = Path(__file__).resolve().parent.parent
#: How far the placed box may sit from its exit slot before `pick` fails (CI uses the exit code), metres.
PICK_TOL = 0.005


def _spec(args) -> CraneSpec:
    try:
        return CraneSpec.from_toml(args.spec)
    except SpecError as e:
        print(e, file=sys.stderr)
        raise SystemExit(2)


def cmd_check(args) -> None:
    s = _spec(args)
    print("crane.toml is valid.")
    for name, j in s.summary()["joints"].items():
        lo, hi = j["range"]
        print(f"  {name:7s} {lo * 1000:8.1f} .. {hi * 1000:8.1f} mm   home {j['home'] * 1000:8.1f} mm")


def cmd_build(args) -> None:
    from crane.exporters import StepExporter
    from crane.visitors.bom import BomVisitor
    from crane.visitors.klipper import KlipperVisitor
    from crane.visitors.mjcf import write_models
    from crane.visitors.stl import StlVisitor
    from crane.visitors.viewer import ViewerVisitor

    s = _spec(args)
    out = Path(args.out)
    world, reg = build_crane(s)
    print("building print set ...")
    stl = StlVisitor(out / "stl")
    world.accept(stl)
    stl.finish()
    print("building BOM ...")
    bom = BomVisitor()
    world.accept(bom)
    bom.write(out / "bom.csv")
    print(f"  BOM total about ${bom.total_usd():.0f}")
    print("writing printer.cfg ...")
    k = KlipperVisitor(s, reg)
    world.accept(k)
    k.write(out / "klipper" / "printer.cfg")
    print("writing MuJoCo model ...")
    write_models(s, world, reg, out / "mujoco")
    (out / "crane.json").write_text(json.dumps(s.summary(), indent=2))
    if not args.no_step:
        print("writing STEP assembly ...")
        v = ViewerVisitor()
        world.accept(v)
        StepExporter().export(v.result, out / "rookery_crane")
    if not args.no_parts:
        _write_parts(world, out / "parts", pictures=True)
    print(f"done: {out.resolve()}")


def _write_parts(world, out: Path, pictures: bool) -> Path:
    from crane.preview import MatplotlibRenderer, NullRenderer
    from crane.visitors.parts_kit import PartsKitVisitor
    print("writing parts kit (one STEP per part) ...")
    if pictures:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            # Null Object instead of a crash: the STEP and STL files still get
            # written, only the preview pictures are skipped.
            print("  matplotlib isn't installed, so no preview pictures "
                  "(fix: pip install -e .  inside the .venv)")
            pictures = False
    kit = PartsKitVisitor(out, renderer=MatplotlibRenderer() if pictures else NullRenderer())
    world.accept(kit)
    index = kit.finish()
    print(f"  {len(kit.records)} parts -> {index}")
    return index


def cmd_bom(args) -> None:
    from crane.visitors.bom import BomVisitor

    world, _ = build_crane(_spec(args))
    bom = BomVisitor()
    world.accept(bom)
    path = bom.write(Path(args.out))
    print(f"wrote {path}: about ${bom.total_usd():.0f}")


def cmd_parts(args) -> None:
    world, _ = build_crane(_spec(args))
    _write_parts(world, Path(args.out) / "parts", pictures=not args.no_pictures)


def cmd_pick(args) -> None:
    import mujoco
    from crane.motion import pick, place_on_exit
    from crane.simcheck import pose, reset_home, run
    from crane.visitors.mjcf import write_models

    s = _spec(args)
    world, reg = build_crane(s)
    paths = write_models(s, world, reg, Path(args.out) / "mujoco")
    m = mujoco.MjModel.from_xml_path(str(paths["preview"]))
    d = mujoco.MjData(m)
    reset_home(m, d)
    t, r, c = args.cell
    box = f"box_{t}_{r}_{c}"
    z0 = float(d.body(box).xpos[2])
    speed = {"rail_y": s.vmax_rail, "mast_z": s.vmax_mast, "arm_x": s.vmax_arm}
    for macro in (pick(s, t, r, c), place_on_exit(s, args.slot, t)):
        print(macro.label, run(m, d, macro, speed))
    end = d.body(box).xpos
    print(f"{box}: started at z={z0:.3f} m, now at ({end[0]:.3f}, {end[1]:.3f}, {end[2]:.3f}) m;"
          f" exit slot {args.slot} is at ({s.exit_x(args.slot):.3f}, {s.exit_y:.3f}, {s.exit_z:.3f}) m")
    goal = (s.exit_x(args.slot), s.exit_y, s.exit_z)
    miss = max(abs(a - b) for a, b in zip(end, goal))
    if miss > PICK_TOL:
        print(f"the box missed exit slot {args.slot} by {miss * 1000:.1f} mm (limit {PICK_TOL * 1000:.0f} mm)",
              file=sys.stderr)
        raise SystemExit(1)


def cmd_show(args) -> None:
    from crane.show import show_crane
    show_crane(args.spec, pose_args=args.pose, rack=not args.no_rack)


def cmd_look(args) -> None:
    from crane.look import write_look

    s = _spec(args)
    world, reg = build_crane(s)
    paths = write_look(s, world, reg, Path(args.out))
    print(f"wrote {paths['look']} and {paths['anchor']}")


def cmd_view(args) -> None:
    from crane.live import view
    view(args.spec, args.out, play=args.pick, cell=args.cell, slot=args.slot, speed=args.speed,
         rebuild=args.rebuild, inset=None if args.inset == "none" else args.inset, classic=args.classic)


COMMANDS = {"check": cmd_check, "build": cmd_build, "bom": cmd_bom, "parts": cmd_parts, "pick": cmd_pick, "show": cmd_show,
            "view": cmd_view, "look": cmd_look}


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="python -m crane", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--spec", default=str(ROOT / "crane.toml"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    b = sub.add_parser("build")
    b.add_argument("--out", default=str(ROOT / "out"))
    b.add_argument("--no-step", action="store_true", help="skip the STEP assembly (the slowest output)")
    b.add_argument("--no-parts", action="store_true", help="skip the per-part kit in out/parts/")
    bm = sub.add_parser("bom")
    bm.add_argument("--out", default=str(ROOT / "bom.csv"))
    pk = sub.add_parser("parts")
    pk.add_argument("--out", default=str(ROOT / "out"))
    pk.add_argument("--no-pictures", action="store_true", help="skip the preview PNGs")
    p = sub.add_parser("pick")
    p.add_argument("--cell", type=int, nargs=3, default=(0, 1, 2), metavar=("TIER", "ROW", "COL"))
    p.add_argument("--slot", type=int, default=0)
    p.add_argument("--out", default=str(ROOT / "out"))
    sh = sub.add_parser("show")
    sh.add_argument("pose", nargs="*", help="e.g. pick:0,1,2  or  mast_z=0.6 arm_x=0.3")
    sh.add_argument("--no-rack", action="store_true")
    vw = sub.add_parser("view", help="open MuJoCo's viewer: drive the joints by hand, or --pick to watch one")
    vw.add_argument("--pick", action="store_true", help="play a pick and place first")
    vw.add_argument("--cell", type=int, nargs=3, default=(0, 1, 2), metavar=("TIER", "ROW", "COL"))
    vw.add_argument("--slot", type=int, default=0)
    vw.add_argument("--speed", type=float, default=1.0, help="playback speed, 2 = twice real time")
    vw.add_argument("--rebuild", action="store_true", help="rewrite out/mujoco/ first")
    vw.add_argument("--inset", default="cam_hook", help="camera shown bottom-right (cam_hook, cam_tier_0, none)")
    vw.add_argument("--classic", action="store_true", help="MuJoCo's own app instead (plain python, no inset)")
    vw.add_argument("--out", default=str(ROOT / "out"))
    lk = sub.add_parser("look", help="the crane's meshes for Rookery's scene (use --spec presets/rookery_tray.toml)")
    lk.add_argument("--out", default=str(ROOT / "meshes" / "rookery"))
    args = ap.parse_args(argv)
    COMMANDS[args.cmd](args)


if __name__ == "__main__":
    main()
