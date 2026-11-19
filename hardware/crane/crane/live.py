"""Open the crane in MuJoCo's own viewer, to drive it by hand or watch a pick.

    python -m crane view                 drive the joints with the sliders (Control panel, right side)
    python -m crane view --pick          play a pick and place, then hand the controls back
    python -m crane view --pick --cell 0 3 1 --slot 1 --speed 2
    python -m crane view --inset cam_tier_0     inset a different camera (--inset none for no inset)
    python -m crane view --classic       MuJoCo's own app, no inset (cameras in the Rendering panel)

The window shows the hook camera (cam_hook, under the fork's top stage) as a
live picture-in-picture in its bottom-right corner.

Pattern: Facade. `view()` hides the three things that make MuJoCo's viewer
awkward on a Mac: which viewer to use, which Python launcher it needs, and
whether out/mujoco/ is up to date. The caller only says "by hand" or "play".

Pattern: Strategy (ViewerMode) + Observer (FrameObserver).

    «interface» ViewerMode                         «interface» FrameObserver
        + open(model, data)                            + on_frame(viewer, m, d)
          ▲                 ▲                              ▲              ▲
    ClassicViewer       LiveViewer ◇──────────────── CameraInset     StatusText
    viewer.launch()     viewer.launch_passive()     offscreen render  joint values
    MuJoCo steps        we step + notify observers  of cam_hook,
    plain python        mjpython on macOS           pasted bottom-right
                            ▲
                        PickPlayback (script() plays the MoveCommands first)

Why two: on macOS the managed viewer (launch) must own the main thread, so it
fails under `mjpython` with "Caught an unknown exception!", while the passive
viewer (launch_passive) only works under `mjpython`. That is exactly the error
`mjpython -m mujoco.viewer` gives. `view()` picks the mode, and if the passive
viewer needs `mjpython` it re-runs this same command under it, so you never
have to remember which launcher goes with which viewer.
"""
from __future__ import annotations

import os
import shutil
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from crane.spec import CraneSpec


class ViewerClosed(Exception):
    """Raised from inside the playback loop when the window is closed."""


# ---------------------------------------------------------------- frame observers
class FrameObserver:
    """Pattern: Observer. Anything that wants to draw on the viewer every
    frame (the hook camera inset, a status line) subscribes to the session and
    gets `on_frame(viewer, model, data)` just before each sync. The session
    never needs to know what its observers draw."""

    def on_frame(self, viewer, model, data) -> None: ...

    def close(self) -> None: ...


class CameraInset(FrameObserver):
    """Picture-in-picture of a named MJCF camera (cam_hook by default) in the
    bottom-right corner of the viewer window: what the camera on the fork sees.

    MuJoCo renders the camera offscreen (mujoco.Renderer) and the passive
    viewer pastes the pixels over its own view (Handle.set_images)."""

    def __init__(self, model, camera: str = "cam_hook", width: int = 400, every: int = 2):
        self.camera = camera
        self.every = every                       # render every 2nd frame (30 Hz) to keep the sim fast
        self._n = 0
        h = int(width * 3 / 4)
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, h)
        self.renderer = mujoco.Renderer(model, h, width)
        self.size = (width, h)

    def on_frame(self, viewer, model, data):
        self._n += 1
        if self._n % self.every:
            return
        vp = viewer.viewport
        if vp is None or vp.width < self.size[0] * 2:
            return
        self.renderer.update_scene(data, self.camera)
        img = self.renderer.render()
        w, h = self.size
        viewer.set_images((mujoco.MjrRect(vp.width - w - 12, 12, w, h), img))

    def close(self):
        self.renderer.close()


class StatusText(FrameObserver):
    """One line of joint values in the top-left corner."""

    def on_frame(self, viewer, model, data):
        q = {j: data.joint(j).qpos[0] for j in ("rail_y", "mast_z", "arm_x", "arm_mid")}
        viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, "joints",
                          "  ".join(f"{k} {v * 1000:.0f} mm" for k, v in q.items())))


# ---------------------------------------------------------------- viewer modes
class ViewerMode:
    def open(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        raise NotImplementedError


class ClassicViewer(ViewerMode):
    """MuJoCo's full app (mujoco.viewer.launch), no inset. Needs plain
    `python` on macOS. Pick cam_hook from the Rendering panel's Camera list."""

    def open(self, model, data):
        mujoco.viewer.launch(model, data)


class LiveViewer(ViewerMode):
    """The passive viewer with observers: we step the physics ourselves, so we
    can draw the camera inset, and optionally play a pick first. The sliders
    in the Control panel still drive the joints.

    Pattern: Template Method in miniature. `open()` fixes the order (open the
    window, run the script, then idle with live sliders); `script()` is the
    part a subclass changes."""

    frame = 1 / 60

    def __init__(self, observers_factory=lambda m: [], speed: float = 1.0):
        self.observers_factory = observers_factory
        self.speed = speed

    def script(self, model, data, pace) -> None:
        """Nothing to play: go straight to driving by hand."""

    def open(self, model, data):
        observers = self.observers_factory(model)
        with mujoco.viewer.launch_passive(model, data) as v:
            clock = {"wall": time.perf_counter(), "sim": data.time}

            def pace(m, d, cmd=None):
                # Draw about 60 times a second and keep sim time in step with
                # the wall clock (times `speed`).
                if not v.is_running():
                    raise ViewerClosed
                if d.time - clock["sim"] >= self.frame * self.speed:
                    for o in observers:
                        o.on_frame(v, m, d)
                    v.sync()
                    ahead = (d.time - clock["sim"]) / self.speed - (time.perf_counter() - clock["wall"])
                    if ahead > 0:
                        time.sleep(ahead)
                    clock["wall"], clock["sim"] = time.perf_counter(), d.time

            try:
                self.script(model, data, pace)
                print("Drive it with the sliders in the Control panel (right side); close the window to quit.")
                while True:                                # idle: keep simulating, sliders live
                    mujoco.mj_step(model, data)
                    pace(model, data)
            except ViewerClosed:
                pass
            finally:
                for o in observers:
                    o.close()


class PickPlayback(LiveViewer):
    """Plays pick(cell) then place_on_exit(slot), then hands over the sliders."""

    def __init__(self, spec: CraneSpec, cell, slot: int, **kw):
        super().__init__(**kw)
        self.s, self.cell, self.slot = spec, tuple(cell), slot

    def script(self, model, data, pace):
        from crane.motion import pick, place_on_exit
        from crane.simcheck import run
        s = self.s
        vmax = {"rail_y": s.vmax_rail, "mast_z": s.vmax_mast, "arm_x": s.vmax_arm}
        t, r, c = self.cell
        for macro in (pick(s, t, r, c), place_on_exit(s, self.slot, t)):
            print(f"  {macro.label}")
            run(model, data, macro, vmax, on_step=pace, max_time=1e9)


def _ensure_models(spec_path: str | Path, out: Path, rebuild: bool) -> Path:
    preview = out / "mujoco" / "preview.xml"
    # Rebuild when crane.toml or any crane/*.py is newer than the model, so
    # a code update (a new camera, say) shows up without a manual build.
    sources = [Path(spec_path), *Path(__file__).parent.rglob("*.py")]
    newest = max(p.stat().st_mtime for p in sources)
    if rebuild or not preview.exists() or preview.stat().st_mtime < newest:
        import crane.parts  # noqa: F401
        from crane.builder import build_crane
        from crane.visitors.mjcf import write_models
        print("writing the MuJoCo model ...")
        spec = CraneSpec.from_toml(spec_path)
        world, reg = build_crane(spec)
        write_models(spec, world, reg, out / "mujoco")
    return preview


def _rerun_under_mjpython() -> None:
    """macOS only: the passive viewer needs Apple's main thread, which only
    the `mjpython` launcher (installed with mujoco, next to python) gives."""
    exe = Path(sys.executable).with_name("mjpython")
    mjpython = str(exe) if exe.exists() else shutil.which("mjpython")
    if not mjpython:
        sys.exit("The pick playback needs `mjpython` on macOS, and it isn't next to this python. "
                 "Run it inside the .venv where mujoco is installed.")
    print("re-running under mjpython (MuJoCo's playback viewer needs it on macOS) ...")
    os.execv(mjpython, [mjpython, "-m", "crane", *sys.argv[1:]])


def view(spec_path: str | Path, out: str | Path, play: bool = False, cell=(0, 1, 2), slot: int = 0,
         speed: float = 1.0, rebuild: bool = False, inset: str | None = "cam_hook", classic: bool = False) -> None:
    """Facade: one call opens the viewer the right way for this machine."""
    from crane.simcheck import reset_home
    preview = _ensure_models(spec_path, Path(out), rebuild)
    model = mujoco.MjModel.from_xml_path(str(preview))
    data = mujoco.MjData(model)
    reset_home(model, data)                       # actuators hold the home pose instead of dropping
    under_mjpython = mujoco.viewer._MJPYTHON is not None
    if classic:
        if sys.platform == "darwin" and under_mjpython:
            sys.exit("Use plain `python -m crane view --classic` (not mjpython).")
        ClassicViewer().open(model, data)
        return
    if sys.platform == "darwin" and not under_mjpython:
        _rerun_under_mjpython()
    cams = {model.camera(i).name for i in range(model.ncam)}
    if inset and inset not in cams:
        print(f"no camera {inset!r} in the model (have: {', '.join(sorted(cams))}); no inset")
        inset = None

    def observers(m):
        obs = [StatusText()]
        if inset:
            try:
                obs.append(CameraInset(m, inset))
            except Exception as e:                # no offscreen GL here: still open the viewer
                print(f"camera inset unavailable ({e.__class__.__name__}: {e}); "
                      f"pick {inset} from the Rendering panel's Camera list instead")
        return obs

    mode = (PickPlayback(CraneSpec.from_toml(spec_path), cell, slot, observers_factory=observers, speed=speed)
            if play else LiveViewer(observers_factory=observers, speed=speed))
    mode.open(model, data)
