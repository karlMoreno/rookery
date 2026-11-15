"""Pick an offscreen GL backend before any test imports mujoco.

macOS: CGL, so no window (not even a hidden GLFW one) appears while tests run.
Linux: EGL, unless MUJOCO_GL is already set. Render tests skip if it's missing.
"""

import os
import sys

os.environ.setdefault("MUJOCO_GL", "cgl" if sys.platform == "darwin" else "egl")
