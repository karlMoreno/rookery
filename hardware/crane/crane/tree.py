"""The assembly tree and the Visitor interface that walks it.

Pattern: Composite.
    The crane is a tree. A BodyNode is one rigid MuJoCo body (frame, truck,
    carriage, arm, hook) and may carry a joint; it contains PartNodes (placed
    geometry) and ItemNodes (things you buy that have no geometry, like belt or
    wire). Why: the tree has exactly the shape of the MJCF body tree, so every
    exporter is a plain walk, and BodyNodes nest the same way joints chain
    (rail_y -> mast_z -> arm_x -> lift).

Pattern: Visitor (with enter/exit hooks for nesting).
    Each output (MJCF, STL, BOM, Klipper config, viewer) is a Visitor class.
    Nodes only know `accept(visitor)`; they never learn about file formats.
    Why: adding a new output means adding one Visitor, without touching any
    node or part. The enter/exit pair lets a visitor keep its own stack
    (an XML parent, a running joint offset) while it descends.

    Node.accept ──► visitor.enter_body ──► children.accept ──► visitor.exit_body
                                 └────► visitor.visit_part / visit_item

All placements are world coordinates in millimetres at the HOME pose. Bodies
only translate along their joint axis, so no rotations are needed between
bodies, which keeps the MJCF conversion exact.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Iterator

from build123d import Location

if TYPE_CHECKING:
    from crane.catalog import CatalogItem
    from crane.parts.base import Part


@dataclass(frozen=True)
class Joint:
    """A prismatic joint. `ref` is the joint value at the home pose, in world
    metres, so MuJoCo's reset puts the crane at home (Rookery's convention)."""
    name: str
    axis: tuple[float, float, float]
    range: tuple[float, float]
    ref: float
    vmax: float = 0.0
    locked: bool = False
    # A follower joint is not driven. It is tied to another joint by
    #     q = ref + ratio * (q_leader - ref)
    # (the telescopic fork's middle stage follows arm_x at half speed). Its ref
    # equals the leader's ref, so both read the same value at home.
    follows: str | None = None
    ratio: float = 1.0

    @property
    def driven(self) -> bool:
        """Has its own motor and actuator (not locked, not a follower)."""
        return not self.locked and self.follows is None

    def follow(self, leader_q: float) -> float:
        lo, hi = self.range
        return min(max(self.ref + self.ratio * (leader_q - self.ref), lo), hi)


@dataclass(frozen=True)
class CollisionBox:
    """A box collision geom in world mm at home. Only the hook gets these:
    Rookery's rule is that only the hook collides."""
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]


class Node(ABC):
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def accept(self, visitor: "Visitor") -> None: ...


class BodyNode(Node):
    """Composite node: one rigid body. `origin` is the body frame in world mm."""

    def __init__(self, name: str, origin: tuple[float, float, float], joint: Joint | None = None,
                 group: str = "crane"):
        super().__init__(name)
        self.origin = origin
        self.joint = joint
        self.group = group            # "crane", "fixtures" or "rack" (rack is preview-only)
        self.children: list[Node] = []
        self.collisions: list[CollisionBox] = []

    def add(self, node: Node) -> Node:
        self.children.append(node)
        return node

    def accept(self, visitor: "Visitor") -> None:
        if not visitor.enter_body(self):
            return
        for child in self.children:
            child.accept(visitor)
        visitor.exit_body(self)

    def walk(self) -> Iterator[Node]:
        yield self
        for c in self.children:
            if isinstance(c, BodyNode):
                yield from c.walk()
            else:
                yield c


class PartNode(Node):
    """Leaf: one placement of a Part (printed or bought) in world mm."""

    def __init__(self, name: str, part: "Part", placement: Location):
        super().__init__(name)
        self.part = part
        self.placement = placement

    def accept(self, visitor: "Visitor") -> None:
        visitor.visit_part(self)


class ItemNode(Node):
    """Leaf: something on the BOM with no geometry (belt, wire, fasteners)."""

    def __init__(self, item: "CatalogItem", qty: float):
        super().__init__(item.key)
        self.item = item
        self.qty = qty

    def accept(self, visitor: "Visitor") -> None:
        visitor.visit_item(self)


class Visitor:
    """Base Visitor. Subclasses override only what they need. enter_body
    returns False to skip a subtree (used to leave preview-only groups out)."""

    def enter_body(self, node: BodyNode) -> bool:
        return True

    def exit_body(self, node: BodyNode) -> None:
        pass

    def visit_part(self, node: PartNode) -> None:
        pass

    def visit_item(self, node: ItemNode) -> None:
        pass
