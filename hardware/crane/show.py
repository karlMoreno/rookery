"""Open this file in VS Code with the OCP CAD Viewer extension and press Run.

Or from a terminal:   python show.py                 (home pose)
                      python show.py pick:0,1,2      (box tier 0, row 1, column 2 lifted)
                      python show.py mast_z=0.7 arm_x=0.3

Edit crane.toml and rerun to see the change. The object tree on the right of
the viewer follows the crane's bodies (crane_frame, truck > carriage > arm >
hook, fixtures, rack), so you can hide or isolate any of them.
"""
import sys
from pathlib import Path

from crane.show import show_crane

show_crane(Path(__file__).with_name("crane.toml"), pose_args=sys.argv[1:])
