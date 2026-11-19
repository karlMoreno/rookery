"""ViewerVisitor: turns the assembly tree into a labelled build123d Compound.

Pattern: Visitor. It walks the same tree as every other exporter, keeping a
stack of open bodies (enter_body / exit_body) and a running joint offset so a
pose can be shown: moving `mast_z` moves the carriage, arm and hook together,
exactly as in MuJoCo.

The result is a Compound whose children mirror the tree (world > truck >
carriage > arm > hook), so the OCP CAD Viewer's object tree lets you hide or
isolate any body or part.
"""
from __future__ import annotations

from build123d import Color, Compound, Location

from crane.tree import BodyNode, PartNode, Visitor


class ViewerVisitor(Visitor):
    def __init__(self, pose: dict[str, float] | None = None, groups: set[str] | None = None):
        self.pose = pose or {}
        self.groups = groups
        self._stack: list[tuple[BodyNode, list, tuple[float, float, float]]] = []
        self.result: Compound | None = None
        self._q: dict[str, float] = {}       # clamped values of the driven joints seen so far

    def enter_body(self, node: BodyNode) -> bool:
        if self.groups is not None and node.name != "world" and node.group not in self.groups:
            return False
        parent_off = self._stack[-1][2] if self._stack else (0.0, 0.0, 0.0)
        off = parent_off
        j = node.joint
        q = None
        if j is not None and j.driven and j.name in self.pose:
            lo, hi = j.range
            q = min(max(self.pose[j.name], lo), hi)
            self._q[j.name] = q
        elif j is not None and j.follows is not None and j.follows in self.pose:
            # Follower (the fork's middle stage): placed from its leader's
            # value, so one arm_x number poses the whole telescope.
            lead = self._q.get(j.follows, self.pose[j.follows])
            q = j.follow(lead)
        if q is not None:
            d = (q - j.ref) * 1000.0
            off = (off[0] + j.axis[0] * d, off[1] + j.axis[1] * d, off[2] + j.axis[2] * d)
        self._stack.append((node, [], off))
        return True

    def visit_part(self, node: PartNode) -> None:
        body, children, off = self._stack[-1]
        shape = Location(off) * (node.placement * node.part.shape())
        shape.label = node.name
        shape.color = Color(*node.part.color)
        children.append(shape)

    def exit_body(self, node: BodyNode) -> None:
        _, children, _ = self._stack.pop()
        comp = Compound(children=children, label=node.name)
        if self._stack:
            self._stack[-1][1].append(comp)
        else:
            self.result = comp
