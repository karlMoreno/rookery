"""MountRegistry: every sensor mount, switch and cable port, declared once.

Pattern: Registry.
    A part declares a mount where it builds it ("there is a microswitch here").
    When the builder places that part on a body, the declaration is copied into
    the registry in world coordinates, tagged with the body it rides on.
    Visitors then read the registry: the MJCF writer makes <site> and <camera>
    elements, the BOM counts switches, the Klipper writer finds the endstops.
    Why: a sensor's position is written in exactly one place, next to the
    geometry that holds it, so the sim, the config and the parts list can't
    disagree about where (or whether) it exists.

The registry is created per build and passed in, not a global singleton, so
two builds (for example a 1-tier and a 3-tier crane in one test run) never
share state.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from build123d import Location, Vector


@dataclass(frozen=True)
class MountDecl:
    """Declared by a part in its own local frame (mm)."""
    name: str
    kind: str            # "endstop", "camera", "cable_port", "exit_slot", "sensor_node", "estop"
    local: Location
    meta: dict = field(default_factory=dict, hash=False, compare=False)


@dataclass(frozen=True)
class Mount:
    """A declaration resolved to world mm and the body it rides on."""
    name: str
    kind: str
    body: str
    world: Location
    meta: dict = field(default_factory=dict, hash=False, compare=False)

    @property
    def position_mm(self) -> Vector:
        return self.world.position


class MountRegistry:
    def __init__(self) -> None:
        self._mounts: dict[str, Mount] = {}

    def register(self, decl: MountDecl, body: str, placement: Location, prefix: str = "",
                 rename: str | None = None) -> Mount:
        name = rename or f"{prefix}{decl.name}"
        if name in self._mounts:
            raise ValueError(f"mount {name!r} declared twice")
        m = Mount(name, decl.kind, body, placement * decl.local, decl.meta)
        self._mounts[name] = m
        return m

    def of_kind(self, kind: str) -> list[Mount]:
        return [m for m in self._mounts.values() if m.kind == kind]

    def on_body(self, body: str) -> list[Mount]:
        return [m for m in self._mounts.values() if m.body == body]

    def __getitem__(self, name: str) -> Mount:
        return self._mounts[name]

    def __iter__(self):
        return iter(self._mounts.values())

    def __len__(self) -> int:
        return len(self._mounts)
