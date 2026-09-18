"""Analytic two-body solution used as ground truth for the Kepler checks."""

from __future__ import annotations

import math


def solve_kepler(mean_anomaly: float, e: float) -> float:
    """Eccentric anomaly E from M = E - e sin E (Newton iteration)."""
    m = math.fmod(mean_anomaly, 2 * math.pi)
    ecc = m if e < 0.8 else math.pi
    for _ in range(60):
        f = ecc - e * math.sin(ecc) - m
        fp = 1.0 - e * math.cos(ecc)
        step = f / fp
        ecc -= step
        if abs(step) < 1e-15:
            break
    return ecc


def relative_state(t: float, a: float, e: float, mu: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Relative position/velocity of the secondary w.r.t. the primary at time t.

    Periapsis at t = 0 on the +x axis, motion counterclockwise in the XY plane.
    """
    n = math.sqrt(mu / a**3)
    ecc = solve_kepler(n * t, e)
    cos_e, sin_e = math.cos(ecc), math.sin(ecc)
    b = a * math.sqrt(1.0 - e * e)
    x = a * (cos_e - e)
    y = b * sin_e
    denom = 1.0 - e * cos_e
    vx = -a * n * sin_e / denom
    vy = b * n * cos_e / denom
    return (x, y, 0.0), (vx, vy, 0.0)


def two_body_state(t: float, a: float, e: float, m1: float, m2: float, G: float):
    """Barycentric positions/velocities of (primary, secondary) at time t."""
    total = m1 + m2
    (rx, ry, rz), (vx, vy, vz) = relative_state(t, a, e, G * total)
    f1, f2 = -m2 / total, m1 / total
    return (
        ((f1 * rx, f1 * ry, f1 * rz), (f1 * vx, f1 * vy, f1 * vz)),
        ((f2 * rx, f2 * ry, f2 * rz), (f2 * vx, f2 * vy, f2 * vz)),
    )


def period(a: float, mu: float) -> float:
    return 2 * math.pi * math.sqrt(a**3 / mu)
