"""Pick and place as Commands, for checking the geometry in simulation.

Pattern: Command (and Macro Command).
    Each MoveCommand is one straight move of some joints to targets. A pick is
    a MacroCommand: an ordered list of moves built from the spec's derived
    numbers. Why: this mirrors Rookery's MoveCommand / Transfer design, so the
    same sequence that proves the hook geometry here can be ported to the
    Rust side unchanged, and `undo()` gives the reverse path for free.

    pick(t, r, c):
      1 rail to the column, arm retracted, hook above the handles
      2 arm in, hook just in front of the bar
      3 mast down: hook below the bar           (the "lift" joint is replaced
      4 arm forward: cup under the bar           by this small mast move)
      5 mast up: box clears the tub rim
"""
from __future__ import annotations

from dataclasses import dataclass, field

from crane.spec import CraneSpec


@dataclass(frozen=True)
class MoveCommand:
    label: str
    targets: dict[str, float]
    dwell: float = 0.3      # seconds to wait after arriving (lets a hanging box stop swinging)

    def undo(self, before: dict[str, float]) -> "MoveCommand":
        return MoveCommand(f"undo {self.label}", {k: before[k] for k in self.targets})


@dataclass
class MacroCommand:
    label: str
    moves: list[MoveCommand] = field(default_factory=list)

    def __iter__(self):
        return iter(self.moves)

    def undo(self, start: dict[str, float]) -> "MacroCommand":
        """Reverse order, each move undone against the pose before it ran."""
        poses = [dict(start)]
        for m in self.moves:
            poses.append({**poses[-1], **m.targets})
        return MacroCommand(f"undo {self.label}",
                            [m.undo(poses[i]) for i, m in reversed(list(enumerate(self.moves)))])


def pick(spec: CraneSpec, t: int, r: int, c: int) -> MacroCommand:
    s = spec
    bar_x, y = s.cell_x(r), s.cell_y(c)
    above = s.handle_z(t) + s.handle_dia / 2 + 0.010 + s.hook_section
    front = bar_x - (s.hook_foot / 2 + s.hook_section + s.handle_dia / 2 + 0.004)
    return MacroCommand(f"pick {t}/{r}/{c}", [
        MoveCommand("to column", {"arm_x": s.arm_x_range[0], "rail_y": y, "mast_z": above}, dwell=0.5),
        MoveCommand("arm in", {"arm_x": front}),
        MoveCommand("hook down", {"mast_z": s.engage_z(t)}),
        MoveCommand("under the bar", {"arm_x": bar_x}),
        MoveCommand("lift", {"mast_z": s.carry_z(t)}),
    ])


def place_on_exit(spec: CraneSpec, slot: int, from_tier: int) -> MacroCommand:
    s = spec
    lift = s.carry_z(from_tier)
    # Cup height at which the box bottom touches the cradle floor. Going a
    # further engage_drop lower frees the bar so the hook can slide out.
    set_down = s.exit_z + s.box_height + s.handle_height - s.handle_dia / 2
    return MacroCommand(f"place on exit {slot}", [
        # A carried box swings like a pendulum. The dwells let it settle after
        # the fork stops and after the long rail move, before it is set down.
        MoveCommand("retract", {"arm_x": s.arm_x_range[0]}, dwell=1.0),
        MoveCommand("to exit", {"rail_y": s.exit_y, "mast_z": max(lift, set_down + 0.03)}, dwell=1.5),
        MoveCommand("arm out", {"arm_x": s.exit_x(slot)}, dwell=2.0),
        MoveCommand("set down", {"mast_z": set_down - s.engage_drop}),
        MoveCommand("hook out", {"arm_x": s.exit_x(slot) - (s.hook_foot / 2 + s.hook_section + s.handle_dia / 2 + 0.004)}),
        MoveCommand("hook up", {"mast_z": set_down + 0.03}),
    ])
