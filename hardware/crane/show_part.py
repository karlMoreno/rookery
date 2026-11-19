"""See one crane part by itself in the OCP CAD Viewer.

    python show_part.py --list                 every part name, printed first
    python show_part.py truck_cheek            one part, the way it sits on the crane
    python show_part.py truck_cheek --flat     a printed part turned the way it prints
    python show_part.py nema17 --step          open the exported STEP file instead

Start the OCP CAD Viewer in VS Code first (the viewer icon in the sidebar).
If it isn't running, the part is written as out/parts/<kind>/<name>.glb instead.

The names are the same as the files in out/parts/ and the cards in
out/parts/index.html. Make those with `python -m crane parts`.

Pattern: Facade. All the work is in crane.show.show_part(); this file only
reads the command line, so it stays short enough to read at a glance.
"""
import argparse
from pathlib import Path

from crane.show import part_records, show_part

SPEC = Path(__file__).with_name("crane.toml")

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("name", nargs="?", help="part name, e.g. truck_cheek or nema17")
ap.add_argument("--list", action="store_true", help="list every part name")
ap.add_argument("--step", action="store_true", help="open out/parts/.../<name>.step instead of building it")
ap.add_argument("--flat", action="store_true", help="printed parts: show the print orientation")
a = ap.parse_args()

if a.list or not a.name:
    for r in part_records(SPEC):
        print(f"  {r.kind:7s}  x{r.qty:<3d} {r.key:28s} {r.title}")
else:
    show_part(SPEC, a.name, from_step=a.step, print_pose=a.flat)
