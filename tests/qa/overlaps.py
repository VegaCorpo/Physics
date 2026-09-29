"""Scene diagnostics: which pairs of bodies are in contact at t = 0?

The module reports a contact when the distance between two centers is at most
the sum of the two radii. This tool applies the same rule to a scene file, so
one can tell whether "no collision detected" is a bug or simply the truth for
that scene. The engine's benchmark scenes (bodies ~1e7 km apart, radii below
1e5 km) contain no contact at all.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Body:
    index: int
    name: str
    x: float
    y: float
    z: float
    radius: float


@dataclass
class Contact:
    a: int
    b: int
    distance: float
    contact_distance: float


def load(path: Path) -> list[Body]:
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    entities = data["entities"] if isinstance(data, dict) else data
    bodies = []
    for index, entity in enumerate(entities):
        comps = entity.get("components", entity)
        pos = comps.get("Position", {})
        radius = comps.get("Radius", {}).get("value", 0.0)
        bodies.append(Body(index, entity.get("name", f"entity_{index}"), float(pos.get("x", 0.0)),
                           float(pos.get("y", 0.0)), float(pos.get("z", 0.0)), float(radius)))
    return bodies


def _dist(a: Body, b: Body) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def contacts(bodies: list[Body]) -> list[Contact]:
    """Exact list of pairs with center distance <= r1 + r2, via a uniform hash grid."""
    if len(bodies) < 2:
        return []
    max_radius = max(b.radius for b in bodies)
    cell = max(2.0 * max_radius, 1.0)
    grid: dict[tuple[int, int, int], list[Body]] = defaultdict(list)
    for b in bodies:
        grid[(math.floor(b.x / cell), math.floor(b.y / cell), math.floor(b.z / cell))].append(b)

    found: list[Contact] = []
    for (cx, cy, cz), cell_bodies in grid.items():
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    other = grid.get((cx + dx, cy + dy, cz + dz))
                    if not other:
                        continue
                    for a in cell_bodies:
                        for b in other:
                            if b.index <= a.index:
                                continue
                            d = _dist(a, b)
                            if d <= a.radius + b.radius:
                                found.append(Contact(a.index, b.index, d, a.radius + b.radius))
    found.sort(key=lambda c: (c.a, c.b))
    return found


def closest_pair(bodies: list[Body]) -> tuple[int, int, float] | None:
    """Closest pair of centers (sweep on x), independent of the radii."""
    if len(bodies) < 2:
        return None
    order = sorted(bodies, key=lambda b: b.x)
    best = (order[0].index, order[1].index, math.inf)
    for i, a in enumerate(order):
        for b in order[i + 1:]:
            if b.x - a.x >= best[2]:
                break
            d = _dist(a, b)
            if d < best[2]:
                best = (a.index, b.index, d)
    return best


def describe(path: Path, limit: int = 20) -> tuple[str, int]:
    bodies = load(path)
    lines = [f"{path}: {len(bodies)} bodies"]
    if not bodies:
        return "\n".join(lines), 0
    radii = [b.radius for b in bodies]
    non_positive = sum(1 for r in radii if r <= 0.0)
    lines.append(f"  radius: min {min(radii):g}, max {max(radii):g}"
                 + (f" ({non_positive} bodies with radius <= 0, they can only touch when coincident)"
                    if non_positive else ""))
    reach = 2.0 * max(radii)
    lines.append(f"  largest possible contact distance (2 x max radius): {reach:g}")
    closest = closest_pair(bodies)
    if closest:
        a, b, d = closest
        ratio = f" = {d / reach:.1f} x the largest contact distance" if reach > 0 else ""
        lines.append(f"  closest centers: {bodies[a].name} - {bodies[b].name}, distance {d:.6g}{ratio}")
    pairs = contacts(bodies)
    lines.append(f"  pairs in contact at t=0: {len(pairs)}")
    for c in pairs[:limit]:
        lines.append(f"    {bodies[c.a].name} - {bodies[c.b].name}: distance {c.distance:.6g}"
                     f" <= r1 + r2 = {c.contact_distance:.6g}")
    if len(pairs) > limit:
        lines.append(f"    ... ({len(pairs) - limit} more)")
    return "\n".join(lines), len(pairs)
