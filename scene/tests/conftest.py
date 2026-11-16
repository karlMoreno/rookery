"""Pick an offscreen GL backend before any test imports mujoco.

macOS: CGL, so no window (not even a hidden GLFW one) appears while tests run.
Elsewhere MUJOCO_GL is left alone. Setting it to `egl` on a runner without
libEGL breaks `import mujoco` itself (seen in CI), whereas the default only
fails when a Renderer is created, and the render tests skip on that.
"""

import os
import sys

if sys.platform == "darwin":
    os.environ.setdefault("MUJOCO_GL", "cgl")
