"""Contact sweep for the Phase 5 lift (row 5.6).

    uv run python -m scene.sweep --seeds 20 --out ../runs/golden/lift_sweep.csv

For every point of a small grid (hook width × mast/hook kp × lift speed),
build the farm scene into a temp dir and run `rookery lift`, which runs the
Phase 5 gate's trial on the first N seeds and reports margins. The CSV has
one row per point: pass rate, worst roll, peak crawfish touch, smallest
water clearance. The defaults should sit inside a region where every
neighbour passes as well.

The sweep regenerates the scene per point, which a Lua scenario cannot do,
so it lives here (decided in #25). It needs a release build of rookery:
`cargo build --release -p rookery`.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from . import builder
from .cli import write_scene
from .spec import SceneSpec

REPO = Path(__file__).resolve().parents[2]

#: The grid, centred on today's defaults (hook_width 0.03, kp 2000, lift speed 1.0).
HOOK_WIDTHS = (0.02, 0.03, 0.04)
KPS = (1000.0, 2000.0, 4000.0)
LIFT_SPEEDS = (0.5, 1.0)


def rookery_bin() -> Path:
    meta = json.loads(subprocess.run(["cargo", "metadata", "--format-version", "1", "--no-deps"],
                                     cwd=REPO, check=True, capture_output=True, text=True).stdout)
    return Path(meta["target_directory"]) / "release" / "rookery"


def build_point(out: Path, hook_width: float, kp: float) -> Path:
    """Write the farm scene with this hook width and mast/hook kp."""
    saved = dict(builder.GANTRY_KP)
    try:
        builder.GANTRY_KP.update({"mast_z": kp, "hook_lift": kp})
        xml = out / f"w{hook_width:.3f}_kp{kp:.0f}.xml"
        write_scene(SceneSpec(hook_width=hook_width), xml)
        return xml
    finally:
        builder.GANTRY_KP.clear()
        builder.GANTRY_KP.update(saved)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="scene.sweep", description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--out", type=Path, default=REPO / "runs" / "golden" / "lift_sweep.csv")
    args = ap.parse_args(argv)
    binary = rookery_bin()
    if not binary.exists():
        print(f"sweep: build it first: cargo build --release -p rookery ({binary})", file=sys.stderr)
        return 2
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        for w, kp in itertools.product(HOOK_WIDTHS, KPS):
            xml = build_point(Path(tmp), w, kp)
            for speed in LIFT_SPEEDS:
                r = subprocess.run([str(binary), "lift", "--scene", str(xml), "--seeds", str(args.seeds),
                                    "--lift-speed", str(speed)], capture_output=True, text=True)
                if r.returncode != 0:
                    print(f"sweep: rookery lift failed at w={w} kp={kp} speed={speed}: {r.stderr}", file=sys.stderr)
                    return 1
                s = json.loads(r.stdout)
                row = {
                    "hook_width": w, "kp": kp, "lift_speed": speed, "trials": s["trials"],
                    "passed": s["passed"], "max_roll_deg": round(s["max_roll_deg"], 3),
                    "max_touch_n": round(s["max_touch_n"], 3),
                    "min_clearance_m": round(s["min_clearance_m"], 4),
                }
                rows.append(row)
                print(f"w={w:.3f} kp={kp:5.0f} speed={speed:.1f}: {s['passed']}/{s['trials']} "
                      f"roll {row['max_roll_deg']:.2f}° touch {row['max_touch_n']:.3f} N "
                      f"clear {row['min_clearance_m']:.4f} m", flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", newline="\n") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"sweep: wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
