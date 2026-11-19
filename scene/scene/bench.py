"""Benchmark sweep (row 12.3): every point of a sweep file run unattended,
one CSV row each.

    uv run python -m scene.bench ../bench/small.toml --out ../results.csv
    uv run python -m scene.bench ../bench/large.toml --out ../runs/large.csv --jobs 2

For each point the scene is built from the point's fields into a temp dir
(never ``scene/generated/``, so nothing needs approving), then
``rookery bench-row`` runs it with ``lua/trees/autonomy.lua`` and prints its
row. Rows come back in the file's order, whatever order the jobs finish in,
so the same file and seed give the same CSV byte for byte.

```text
  bench/*.toml ─ point {trays, cells, shelf_gap, molt_rate, detector} × seeds
     ─▶ SceneSpec ─▶ tmp/<name>.xml ─▶ rookery bench-row ─▶ row ─▶ results.csv (fixed header)
```

It needs a release build of rookery: ``cargo build --release -p rookery``.
Pattern from the Phase 5 lift sweep (``sweep.py``), which also had to build
scenes a Lua scenario can't.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

from .cli import write_scene
from .spec import SceneSpec
from .sweep import rookery_bin


def points(sweep: dict) -> list[dict]:
    """Every (point, seed) the sweep file asks for, in order."""
    seeds = sweep.get("seeds", [sweep.get("seed", 7)])
    out = []
    for p in sweep["point"]:
        for seed in seeds:
            out.append({**p, "seed": seed, "days": p.get("days", sweep.get("days", 7.0))})
    return out


def scene_name(p: dict) -> str:
    """A readable stem for the point's scene, which lands in the CSV."""
    trays = p.get("trays", 1)
    name = f"tray_{p['cells']}" if trays == 1 else f"rack{trays}_{p['cells']}"
    if "shelf_gap" in p and trays > 1:
        name += f"_gap{p['shelf_gap']:.2f}"
    return name


def build(p: dict, out: Path) -> Path:
    """Write the point's scene into ``out`` and return its XML path."""
    rows, cols = (int(v) for v in p["cells"].lower().split("x"))
    kw = {"trays": p.get("trays", 1), "rows": rows, "cols": cols}
    if "shelf_gap" in p:
        kw["shelf_gap"] = p["shelf_gap"]
    xml = out / f"{scene_name(p)}.xml"
    if not xml.exists():
        write_scene(SceneSpec(**kw), xml)
    return xml


def run(binary: Path, xml: Path, p: dict) -> str:
    """One point's CSV row from ``rookery bench-row``."""
    cmd = [str(binary), "bench-row", "--scene", str(xml), "--molt-rate", str(p["molt_rate"]),
           "--detector", p["detector"], "--seed", str(p["seed"]), "--days", str(p["days"])]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)}\n{r.stderr}")
    return r.stdout.strip()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="scene.bench", description=__doc__.splitlines()[0])
    ap.add_argument("sweep", type=Path, help="a sweep file, bench/small.toml or bench/large.toml")
    ap.add_argument("--out", type=Path, required=True, help="the results CSV")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1, help="points run at once")
    args = ap.parse_args(argv)
    binary = rookery_bin()
    if not binary.exists():
        print(f"bench: build it first: cargo build --release -p rookery ({binary})", file=sys.stderr)
        return 2
    sweep = tomllib.loads(args.sweep.read_text(encoding="utf-8"))
    todo = points(sweep)
    header = subprocess.run([str(binary), "bench-row", "--header"], check=True, capture_output=True,
                            text=True).stdout.strip()
    with tempfile.TemporaryDirectory() as tmp:
        xmls = [build(p, Path(tmp)) for p in todo]
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            futures = [pool.submit(run, binary, x, p) for x, p in zip(xmls, todo)]
            rows = []
            for f, p in zip(futures, todo):
                try:
                    rows.append(f.result())
                except RuntimeError as e:
                    print(f"bench: {e}", file=sys.stderr)
                    return 1
                print(f"bench: {rows[-1]}", flush=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(header + "\n" + "\n".join(rows) + "\n", encoding="utf-8", newline="\n")
    print(f"bench: {len(rows)} rows -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
