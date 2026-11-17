"""The README's demo gifs, rendered offscreen from logged runs (no window).

Make the four runs first (release build), into one log directory:

    rookery run --scenario lua/scenarios/patrol_4x4.lua   --log runs/demo --id patrol
    rookery run --scenario lua/scenarios/transfer_1_2.lua --log runs/demo --id transfer
    rookery run --tree lua/trees/transfer.lua --seconds 40 --fault-at 700 --log runs/demo --id fault
    rookery run --scenario lua/scenarios/week.lua         --log runs/demo --id week

then, from the repo root:

    uv run --project scene python scripts/demo_gifs.py runs/demo docs [patrol lift transfer fault molt]

On macOS it renders with CGL, so nothing opens on screen. Each frame is
quantised to 128 colours; identical frames are merged, so held captions
cost nothing.
"""
import csv
import json
import os
import sys

os.environ.setdefault("MUJOCO_GL", "cgl")
import mujoco  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

RUNS = sys.argv[1]
OUT = sys.argv[2]
os.makedirs(OUT, exist_ok=True)


def load(run):
    d = os.path.join(RUNS, run)
    s = json.load(open(os.path.join(d, "summary.json")))
    m = mujoco.MjModel.from_xml_path(os.path.abspath(s["scene"]))
    rows = [list(map(float, r)) for i, r in enumerate(csv.reader(open(os.path.join(d, "qpos.csv")))) if i > 0]
    return m, s, [r[0] for r in rows], [r[1:] for r in rows]


def gif(name, run, start, end, speed, fps=12, size=(560, 420), cam=None, label=None, picks=None, hold=0):
    m, s, times, rows = load(run)
    if picks is None:
        picks, t, i = [], times[0] + start, 0
        t1 = min(times[-1], times[0] + end)
        while t <= t1 + 1e-9:
            while i + 1 < len(times) and times[i + 1] <= t:
                i += 1
            picks.append(i)
            t += speed / fps
    d = mujoco.MjData(m)
    font = ImageFont.load_default(size=16)
    frames = []
    with mujoco.Renderer(m, height=size[1], width=size[0]) as r:
        for k in picks:
            d.qpos[:] = rows[k]
            mujoco.mj_forward(m, d)
            if cam is None:
                r.update_scene(d, camera="overview")
            else:
                c = mujoco.MjvCamera()
                c.type = mujoco.mjtCamera.mjCAMERA_FREE
                c.lookat[:], c.distance, c.azimuth, c.elevation = cam
                r.update_scene(d, camera=c)
            img = Image.fromarray(r.render().copy())
            if label:
                dr = ImageDraw.Draw(img)
                text = label(k, times[k]) if callable(label) else label
                dr.rectangle([0, 0, size[0], 26], fill=(255, 255, 255))
                dr.text((8, 4), text, fill=(30, 30, 30), font=font)
            frames.append(img)
    frames += [frames[-1]] * hold
    path = os.path.join(OUT, name)
    pal = [f.quantize(colors=128, method=Image.Quantize.MEDIANCUT) for f in frames]
    pal[0].save(path, save_all=True, append_images=pal[1:], duration=round(1000.0 / fps), loop=0, optimize=True)
    print(name, len(frames), "frames", os.path.getsize(path) // 1024, "KB")
    return m


which = sys.argv[3:] or ["patrol", "lift", "transfer", "fault", "molt"]
if "patrol" in which:
    gif("patrol.gif", "patrol", 0, 60, 4.0, label=lambda k, t: f"patrol: visiting every cell (4x speed)  t = {t:5.1f} s")
if "lift" in which:
    m, s, times, rows = load("transfer")
    gif("lift.gif", "transfer", 3, 16, 1.0, cam=([-0.075, 0.075, 0.12], 0.55, 200, -15),
        label=lambda k, t: f"lift: hook slides under the handle, lifts box (1, 2)  t = {t:4.1f} s")
if "transfer" in which:
    gif("transfer.gif", "transfer", 0, 31.5, 2.0, label=lambda k, t: f"transfer: box (1, 2) to the exit tray (2x speed)  t = {t:4.1f} s")
if "fault" in which:
    gif("fault_undo.gif", "fault", 0, 28.5, 2.0,
        label=lambda k, t: ("fault injected at 14 s: the transfer is undone" if t >= 14.0 else "transfer under way...") + f"  t = {t:4.1f} s")
if "molt" in which:
    m, s, times, rows = load("week")
    ev = s["events"]
    first = next(e for e in ev if e["kind"] == "Molted")
    r, c = first["r"], first["c"]
    k = r * 4 + c
    zi = 7 * (32 + k) + 2
    inbox = [i for i, q in enumerate(rows) if q[zi] > -0.05]
    a, b = inbox[0], inbox[-1]
    eaten = next(e for e in ev if e["kind"] == "ShellEaten" and (e["r"], e["c"]) == (r, c))
    win = next(e for e in ev if e["kind"] == "WindowClosing" and (e["r"], e["c"]) == (r, c))
    x = -0.225 + 0.15 * r
    y = -0.225 + 0.15 * c
    day = lambda t: t / 86400
    picks = [max(0, a - 1)] * 24 + [a] * 30 + [min(b, a + 3)] * 24 + [b + 1 if b + 1 < len(rows) else b] * 30
    texts = ([f"box ({r}, {c}): crawfish in premolt"] * 24
             + [f"day {day(first['t']):.2f}: molt! its shed shell lands beside it"] * 30
             + [f"day {day(win['t']):.2f}: soft-shell window closes (harvest is Phase 8)"] * 24
             + [f"day {day(eaten['t']):.2f}: the shell is eaten and parked again"] * 30)
    it = iter(texts)
    gif("molt.gif", "week", 0, 0, 1.0, fps=12, cam=([x, y, 0.03], 0.32, 135, -50), picks=picks, label=lambda kk, t: next(it))
