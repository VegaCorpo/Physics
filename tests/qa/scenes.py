"""Deterministic scene generators.

Scenes use the same JSON layout as the engine's scenes/*.json so they can also be
loaded in the real application. Units: km, km/s, kg, seconds.
"""

from __future__ import annotations

import json
import math
import random
import struct
from pathlib import Path

G_DEFAULT = 6.67430e-20


def float32(value: float) -> float:
    """Round like the module does: Mass::mantissa is a float."""
    return struct.unpack("f", struct.pack("f", value))[0]


def effective_mass(mantissa: float, exponent: int) -> float:
    """Mass as the module sees it after the float mantissa round trip."""
    return float32(mantissa) * 10.0**exponent


def split_mass(mass: float) -> tuple[float, int]:
    """Express a mass in kg as (mantissa, exponent) with mantissa in [1, 10)."""
    if mass <= 0:
        return 0.0, 0
    exponent = int(math.floor(math.log10(mass)))
    mantissa = mass / 10.0**exponent
    if mantissa >= 10.0:
        mantissa /= 10.0
        exponent += 1
    return mantissa, exponent


def body(name: str, mass: tuple[float, int], pos, vel) -> dict:
    return {
        "name": name,
        "components": {
            "Name": {"value": name},
            "Mass": {"mantissa": mass[0], "exponent": mass[1]},
            "Acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
            "Position": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "Velocity": {"x": vel[0], "y": vel[1], "z": vel[2]},
            "Radius": {"value": 1000.0},
        },
    }


def write_scene(path: Path, bodies: list[dict]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump({"entities": bodies}, handle)
    return path


def two_body(primary: tuple[float, int], secondary: tuple[float, int], a: float, e: float,
             G: float = G_DEFAULT) -> list[dict]:
    """Two bodies at periapsis in the barycentric frame. Orbit in the XY plane."""
    m1 = effective_mass(*primary)
    m2 = effective_mass(*secondary)
    total = m1 + m2
    mu = G * total
    r = a * (1.0 - e)
    v = math.sqrt(mu * (1.0 + e) / (a * (1.0 - e)))
    return [
        body("primary", primary, (-m2 / total * r, 0.0, 0.0), (0.0, -m2 / total * v, 0.0)),
        body("secondary", secondary, (m1 / total * r, 0.0, 0.0), (0.0, m1 / total * v, 0.0)),
    ]


def cluster(n: int, seed: int, extent: float = 1e9, speed: float = 20.0,
            mass_exponents: tuple[int, int] = (27, 29)) -> list[dict]:
    """Random cluster mirroring scenes/benchmark_*.json statistics."""
    rng = random.Random(seed)
    bodies = []
    for i in range(n):
        mantissa = round(rng.uniform(1.0, 9.99), 2)
        exponent = rng.randint(*mass_exponents)
        pos = tuple(round(rng.uniform(-extent, extent), 2) for _ in range(3))
        vel = tuple(round(rng.uniform(-speed, speed), 2) for _ in range(3))
        bodies.append(body(f"entity_{i}", (mantissa, exponent), pos, vel))
    return bodies


def planetary(planets: int, seed: int, star: tuple[float, int] = (1.989, 30),
              r_min: float = 5e7, r_max: float = 5e9, G: float = G_DEFAULT) -> list[dict]:
    """A star with planets on near-circular, slightly inclined orbits."""
    rng = random.Random(seed)
    m_star = effective_mass(*star)
    bodies = [body("star", star, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))]
    for i in range(planets):
        radius = r_min * (r_max / r_min) ** (i / max(1, planets - 1))
        mass = split_mass(rng.uniform(1e23, 2e27))
        mass = (round(mass[0], 3), mass[1])
        phase = rng.uniform(0, 2 * math.pi)
        incl = rng.uniform(-0.1, 0.1)
        v = math.sqrt(G * m_star / radius) * rng.uniform(0.97, 1.03)
        pos = (radius * math.cos(phase), radius * math.sin(phase) * math.cos(incl),
               radius * math.sin(phase) * math.sin(incl))
        vel = (-v * math.sin(phase), v * math.cos(phase) * math.cos(incl), v * math.cos(phase) * math.sin(incl))
        bodies.append(body(f"planet_{i}", mass, pos, vel))
    # Give the star the velocity that keeps the barycenter at rest.
    momentum = [0.0, 0.0, 0.0]
    for planet in bodies[1:]:
        comp = planet["components"]
        m = effective_mass(comp["Mass"]["mantissa"], comp["Mass"]["exponent"])
        for k, axis in enumerate("xyz"):
            momentum[k] += m * comp["Velocity"][axis]
    bodies[0]["components"]["Velocity"] = {axis: -momentum[k] / m_star for k, axis in enumerate("xyz")}
    return bodies


def single(vel=(1.0, -2.0, 3.0)) -> list[dict]:
    return [body("lonely", (5.97, 24), (1.0e6, 2.0e6, 3.0e6), vel)]


def coincident() -> list[dict]:
    return [
        body("a", (1.0, 26), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        body("b", (1.0, 26), (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
        body("c", (1.0, 26), (1.0e6, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ]


def massless_probe(a: float = 149597870.0, G: float = G_DEFAULT) -> list[dict]:
    """A star and a massless test particle on a circular orbit around it."""
    star = (1.989, 30)
    v = math.sqrt(G * effective_mass(*star) / a)
    return [
        body("star", star, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        body("probe", (0.0, 0), (a, 0.0, 0.0), (0.0, v, 0.0)),
    ]


def shuffled(bodies: list[dict], seed: int) -> tuple[list[dict], list[int]]:
    """Return a permuted copy and the permutation (new index -> original index)."""
    rng = random.Random(seed)
    order = list(range(len(bodies)))
    rng.shuffle(order)
    return [bodies[i] for i in order], order


def translated(bodies: list[dict], offset) -> list[dict]:
    out = json.loads(json.dumps(bodies))
    for b in out:
        p = b["components"]["Position"]
        p["x"] += offset[0]
        p["y"] += offset[1]
        p["z"] += offset[2]
    return out
