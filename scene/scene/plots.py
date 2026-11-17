"""Figures from run logs (row 7.7; Phase 12 adds more).

    uv run python -m scene.plots molt_hist runs/<id>

``molt_hist`` reads ``runs/<id>/summary.json`` (written by ``rookery run``),
takes every animal's molt-to-molt intervals from its ``events``, and draws
their histogram over the configured distribution: a lognormal with the
interval mean and sigma that ``rookery-core`` reports in ``lifecycle``. It
writes ``runs/<id>/molt_hist.png`` and prints the empirical mean and sigma
against the configured ones.

```text
  summary.json ─ events [{t, kind, r, c}] ─▶ per cell: Molted times ─▶ diffs (days)
               └ lifecycle {interval_mean, interval_sigma} ─▶ lognormal pdf
                                       ▼
                     molt_hist.png  (bars = runs, line = configured)
```

The interval is a sum of five lognormal stages, so a lognormal with the same
mean and sigma is an approximation: close at the default 13% coefficient of
variation, which is what gate A checks.

Drawn with Pillow alone (no matplotlib): same input, same bytes, so a
figure can be committed and regenerated (rule 7, and 12.4's byte-identical
figures).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

DAY = 86_400.0
W, H = 800, 500
MARGIN_L, MARGIN_R, MARGIN_T, MARGIN_B = 70, 20, 40, 60
BAR = (214, 120, 60)
LINE = (40, 90, 160)
INK = (30, 30, 30)
GRID = (225, 225, 225)


def intervals(summary: dict) -> list[float]:
    """Every molt-to-molt interval in the run, in seconds, cell by cell."""
    times: dict[tuple[int, int, int], list[float]] = defaultdict(list)
    for e in summary.get("events", []):
        if e["kind"] == "Molted":
            times[(e["tray"], e["r"], e["c"])].append(float(e["t"]))
    out: list[float] = []
    for cell in sorted(times):
        ts = sorted(times[cell])
        out += [b - a for a, b in zip(ts, ts[1:])]
    return out


def lognormal_pdf(x: float, mean: float, sigma: float) -> float:
    """The lognormal density with the given mean and standard deviation."""
    if x <= 0:
        return 0.0
    s2 = math.log(1 + (sigma / mean) ** 2)
    mu = math.log(mean) - s2 / 2
    return math.exp(-((math.log(x) - mu) ** 2) / (2 * s2)) / (x * math.sqrt(2 * math.pi * s2))


def stats(xs: list[float]) -> tuple[float, float]:
    """Mean and sample standard deviation."""
    m = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else 0.0
    return m, sd


def molt_hist(run_dir: Path, bins: int = 20) -> tuple[Path, str]:
    """Write ``molt_hist.png`` into ``run_dir``; return its path and a one-line summary."""
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    life = summary.get("lifecycle")
    if not life:
        raise ValueError(f"{run_dir}/summary.json has no lifecycle (run it with a newer rookery)")
    xs = [x / DAY for x in intervals(summary)]
    if len(xs) < 2:
        raise ValueError(
            f"{run_dir}: {len(xs)} molt interval(s); run longer (a sim-year gives about 180 with 16 cells)"
        )
    mean, sigma = life["interval_mean"] / DAY, life["interval_sigma"] / DAY
    m, sd = stats(xs)

    lo = max(0.0, min(min(xs), mean - 4 * sigma))
    hi = max(max(xs), mean + 4 * sigma)
    width = (hi - lo) / bins
    counts = [0] * bins
    for x in xs:
        counts[min(bins - 1, int((x - lo) / width))] += 1
    density = [c / (len(xs) * width) for c in counts]
    curve = [(lo + (hi - lo) * i / 200, lognormal_pdf(lo + (hi - lo) * i / 200, mean, sigma)) for i in range(201)]
    ymax = max(max(density), max(y for _, y in curve)) * 1.1

    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    font = ImageFont.load_default()
    px = lambda x: MARGIN_L + (x - lo) / (hi - lo) * (W - MARGIN_L - MARGIN_R)  # noqa: E731
    py = lambda y: H - MARGIN_B - y / ymax * (H - MARGIN_T - MARGIN_B)  # noqa: E731

    for k in range(6):  # horizontal grid and y labels
        y = ymax * k / 5
        d.line([(MARGIN_L, py(y)), (W - MARGIN_R, py(y))], fill=GRID)
        d.text((8, py(y) - 6), f"{y:.3f}", fill=INK, font=font)
    for i, v in enumerate(density):
        x0, x1 = px(lo + i * width), px(lo + (i + 1) * width)
        if v > 0:
            d.rectangle([x0 + 1, py(v), x1 - 1, py(0)], fill=BAR)
    d.line([(px(x), py(y)) for x, y in curve], fill=LINE, width=3)
    d.line([(MARGIN_L, py(0)), (W - MARGIN_R, py(0))], fill=INK)
    d.line([(MARGIN_L, MARGIN_T), (MARGIN_L, py(0))], fill=INK)
    for k in range(6):  # x labels, in days
        x = lo + (hi - lo) * k / 5
        d.text((px(x) - 10, H - MARGIN_B + 8), f"{x:.1f}", fill=INK, font=font)
    d.text((W // 2 - 90, H - 25), "molt-to-molt interval (days)", fill=INK, font=font)
    d.text((8, MARGIN_T - 22), "density (1/day)", fill=INK, font=font)
    d.text((MARGIN_L + 100, 12), f"{len(xs)} intervals: mean {m:.2f} d, sd {sd:.2f} d", fill=BAR, font=font)
    d.text((W // 2 + 40, 12), f"configured: mean {mean:.2f} d, sd {sigma:.2f} d", fill=LINE, font=font)

    out = run_dir / "molt_hist.png"
    img.save(out, format="PNG", optimize=False)
    line = (
        f"molt_hist: {len(xs)} intervals, mean {m:.2f} d (configured {mean:.2f}, {100 * (m - mean) / mean:+.1f}%), "
        f"sd {sd:.2f} d (configured {sigma:.2f}, {100 * (sd - sigma) / sigma:+.1f}%) -> {out}"
    )
    return out, line


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="scene.plots", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="figure", required=True)
    mh = sub.add_parser("molt_hist", help="molt-interval histogram over the configured lognormal")
    mh.add_argument("run", type=Path, help="a run directory, runs/<id>")
    mh.add_argument("--bins", type=int, default=20)
    args = ap.parse_args(argv)
    try:
        _, line = molt_hist(args.run, args.bins)
    except (OSError, ValueError, KeyError) as e:
        print(f"scene.plots: {e}", file=sys.stderr)
        return 1
    print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
