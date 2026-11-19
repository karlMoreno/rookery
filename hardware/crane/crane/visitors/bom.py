"""BomVisitor: the bill of materials.

Pattern: Visitor. Every bought PartNode adds its catalogue item (in metres for
extrusion, rail, screw and drag chain; each for everything else). Every ItemNode
adds its quantity. Printed parts add filament and heat-set inserts, counted
from the fasteners each part declares, so the BOM can't drift from the CAD.
"""
from __future__ import annotations

import csv
import math
from collections import defaultdict
from pathlib import Path

from crane.catalog import item
from crane.tree import BodyNode, ItemNode, PartNode, Visitor


class BomVisitor(Visitor):
    def __init__(self, groups: set[str] | None = None):
        self.groups = groups
        self.qty: dict[str, float] = defaultdict(float)
        self.filament_kg = 0.0
        self.inserts = 0

    def enter_body(self, node: BodyNode) -> bool:
        return self.groups is None or node.name == "world" or node.group in self.groups

    def visit_part(self, node: PartNode) -> None:
        p = node.part
        if p.process == "printed":
            self.filament_kg += p.mass_kg
            self.inserts += p.insert_count
        else:
            self.qty[p.item.key] += p.qty_per_use

    def visit_item(self, node: ItemNode) -> None:
        self.qty[node.item.key] += node.qty

    def rows(self) -> list[dict]:
        q = dict(self.qty)
        q["petg"] = q.get("petg", 0) + self.filament_kg * 1.25            # 25 % for supports and failed prints
        q["heat_insert_m3"] = q.get("heat_insert_m3", 0) + self.inserts
        out = []
        for key, n in q.items():
            it = item(key)
            if it.unit == "m":
                n = math.ceil(n * 10) / 10                                 # buy to the next 0.1 m
            elif it.unit == "kg":
                n = math.ceil(n * 4) / 4
            else:
                n = math.ceil(n)
            out.append({"section": it.section, "item": it.description, "key": key, "qty": n, "unit": it.unit,
                        "unit_usd": it.price_usd, "total_usd": round(n * it.price_usd, 2)})
        order = {"Structure and motion": 0, "Compute, electronics, power": 1, "Sensors": 2}
        return sorted(out, key=lambda r: (order.get(r["section"], 9), r["item"]))

    def write(self, path: Path) -> Path:
        rows = self.rows()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        return path

    def total_usd(self) -> float:
        return round(sum(r["total_usd"] for r in self.rows()), 2)
