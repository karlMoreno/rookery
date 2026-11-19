"""Run MoveCommands against the MuJoCo preview model.

Used by the tests and by `python -m crane pick` to prove the hook geometry:
the hook really gets under a handle and lifts a box clear of the tub.
"""
from __future__ import annotations

import mujoco
import numpy as np

from crane.motion import MacroCommand

JOINTS = ("rail_y", "mast_z", "arm_x")


def pose(data: mujoco.MjData) -> dict[str, float]:
    return {j: float(data.joint(j).qpos[0]) for j in JOINTS}


def run(model: mujoco.MjModel, data: mujoco.MjData, macro: MacroCommand, speed: dict[str, float],
        tol: float = 0.001, on_step=None, max_time: float = 60.0) -> list[str]:
    """Move each joint at no more than `speed` (m/s) toward each command's
    targets, by ramping the position actuator setpoints. Returns the labels
    of commands that finished."""
    done = []
    dt = model.opt.timestep
    for cmd in macro:
        start = {j: float(data.ctrl[model.actuator(f"{j}_act").id]) for j in JOINTS}
        T = max(abs(cmd.targets.get(j, start[j]) - start[j]) / speed[j] for j in JOINTS)
        steps = int(T / dt) + 1
        for k in range(steps + int(cmd.dwell / dt)):
            a = min(1.0, (k + 1) / steps)
            for j in JOINTS:
                tgt = cmd.targets.get(j, start[j])
                data.ctrl[model.actuator(f"{j}_act").id] = start[j] + (tgt - start[j]) * a
            mujoco.mj_step(model, data)
            if on_step:
                on_step(model, data, cmd)
            if data.time > max_time:
                return done
        err = max(abs(cmd.targets.get(j, pose(data)[j]) - pose(data)[j]) for j in JOINTS)
        if err > tol * 5:
            raise RuntimeError(f"{cmd.label}: stuck {err * 1000:.1f} mm from target")
        done.append(cmd.label)
    return done


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    mujoco.mj_resetData(model, data)
    for j in JOINTS:
        data.ctrl[model.actuator(f"{j}_act").id] = model.qpos0[model.jnt_qposadr[model.joint(j).id]]
    mujoco.mj_forward(model, data)
