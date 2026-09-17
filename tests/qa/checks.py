"""Correctness, integrator and robustness checks.

Every check is a function `check(ctx, cfg) -> Result`. It builds a scene, drives the
module through the runner, and compares against an analytic solution, a
high-precision reference, a conserved quantity, or an invariance. Thresholds come
exclusively from `cfg` (config/thresholds.json).
"""

from __future__ import annotations

import math
import traceback
from dataclasses import dataclass, field
from pathlib import Path

from . import kepler, scenes
from .runner import Runner


@dataclass
class Metric:
    name: str
    value: float | None
    threshold: float | None
    op: str = "<="
    unit: str = ""

    @property
    def ok(self) -> bool:
        if self.value is None or self.threshold is None:
            return self.value is not None
        if math.isnan(self.value):
            return False
        if self.op == "<=":
            return self.value <= self.threshold
        if self.op == ">=":
            return self.value >= self.threshold
        if self.op == "==":
            return self.value == self.threshold
        raise ValueError(self.op)

    def to_dict(self) -> dict:
        return {"name": self.name, "value": self.value, "threshold": self.threshold, "op": self.op,
                "unit": self.unit, "ok": self.ok}


@dataclass
class Result:
    name: str
    category: str
    required: bool = True
    metrics: list[Metric] = field(default_factory=list)
    details: str = ""
    series: dict = field(default_factory=dict)
    context: dict = field(default_factory=dict)
    error: str | None = None

    @property
    def status(self) -> str:
        if self.error:
            return "error" if self.required else "warn"
        if all(m.ok for m in self.metrics):
            return "pass"
        return "fail" if self.required else "warn"

    def to_dict(self) -> dict:
        return {"name": self.name, "category": self.category, "required": self.required, "status": self.status,
                "metrics": [m.to_dict() for m in self.metrics], "details": self.details, "series": self.series,
                "context": self.context, "error": self.error}


@dataclass
class Context:
    runner: Runner
    scene_dir: Path
    dt: float
    G: float
    ref_substeps: int
    ref_softening: float

    def scene(self, name: str, bodies: list[dict]) -> Path:
        return scenes.write_scene(self.scene_dir / f"{name}.json", bodies)


# --------------------------------------------------------------------------- helpers

def _dist(a, b) -> float:
    return math.dist(a, b)


def _norm(v) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _scene_scale(sample: dict) -> float:
    """RMS distance of the bodies to their barycenter (km)."""
    com = sample["center_of_mass"]
    pos = sample["positions"]
    if not pos:
        return 1.0
    return math.sqrt(sum(_dist(p, com) ** 2 for p in pos) / len(pos)) or 1.0


def _rms_speed(sample: dict) -> float:
    vel = sample["velocities"]
    if not vel:
        return 1.0
    return math.sqrt(sum(_norm(v) ** 2 for v in vel) / len(vel)) or 1.0


def _max_rel_position_error(sim: dict, ref: dict, scale: float | None = None) -> float:
    scale = scale or _scene_scale(ref)
    return max((_dist(p, q) for p, q in zip(sim["positions"], ref["positions"])), default=0.0) / scale


def _max_rel_velocity_error(sim: dict, ref: dict, scale: float | None = None) -> float:
    scale = scale or _rms_speed(ref)
    return max((_dist(p, q) for p, q in zip(sim["velocities"], ref["velocities"])), default=0.0) / scale


def _rel_drift(samples: list[dict], key: str, scale: float | None = None) -> list[float]:
    first = samples[0][key]
    if isinstance(first, list):
        base = scale or _norm(first) or 1.0
        return [_norm([a - b for a, b in zip(s[key], first)]) / base for s in samples]
    base = scale or abs(first) or 1.0
    return [abs(s[key] - first) / base for s in samples]


def _mass_scales(bodies: list[dict]) -> tuple[float, float, float]:
    """(sum m|v|, sum m|r||v|, total mass) from the initial scene, for drift normalisation."""
    p_scale = l_scale = total = 0.0
    for b in bodies:
        c = b["components"]
        m = scenes.effective_mass(c["Mass"]["mantissa"], c["Mass"]["exponent"])
        r = _norm((c["Position"]["x"], c["Position"]["y"], c["Position"]["z"]))
        v = _norm((c["Velocity"]["x"], c["Velocity"]["y"], c["Velocity"]["z"]))
        p_scale += m * v
        l_scale += m * r * v
        total += m
    return p_scale or 1.0, l_scale or 1.0, total or 1.0


def _series(x, y, xlabel, ylabel, log_y=True) -> dict:
    return {"x": list(x), "y": list(y), "xlabel": xlabel, "ylabel": ylabel, "log_y": log_y}


def _two_body(cfg: dict, ctx: Context):
    m1 = scenes.effective_mass(*cfg["primary_mass"])
    m2 = scenes.effective_mass(*cfg["secondary_mass"])
    a = float(cfg["semi_major_axis_km"])
    e = float(cfg["eccentricity"])
    bodies = scenes.two_body(tuple(cfg["primary_mass"]), tuple(cfg["secondary_mass"]), a, e, ctx.G)
    period = kepler.period(a, ctx.G * (m1 + m2))
    return bodies, m1, m2, a, e, period


def _kepler_errors(samples: list[dict], m1: float, m2: float, a: float, e: float, G: float) -> list[float]:
    errors = []
    for s in samples:
        (p1, _), (p2, _) = kepler.two_body_state(s["t"], a, e, m1, m2, G)
        errors.append(max(_dist(s["positions"][0], p1), _dist(s["positions"][1], p2)) / a)
    return errors


# --------------------------------------------------------------------------- checks

def check_kepler(ctx: Context, cfg: dict, name: str) -> Result:
    res = Result(name, cfg.get("category", "correctness"))
    bodies, m1, m2, a, e, period = _two_body(cfg, ctx)
    frames = max(1, int(round(cfg["orbits"] * period / ctx.dt)))
    every = max(1, frames // int(cfg["samples"]))
    sim = ctx.runner.simulate(ctx.scene(name, bodies), ctx.dt, frames, every, tag=name)
    samples = sim["samples"]
    pos_err = _kepler_errors(samples, m1, m2, a, e, ctx.G)
    e_drift = _rel_drift(samples, "energy")
    l_drift = _rel_drift(samples, "angular_momentum")
    t_orbits = [s["t"] / period for s in samples]
    res.metrics = [
        Metric("max_rel_position_error", max(pos_err), cfg["max_rel_position_error"]),
        Metric("max_rel_energy_drift", max(e_drift), cfg["max_rel_energy_drift"]),
        Metric("max_rel_angular_momentum_drift", max(l_drift), cfg["max_rel_angular_momentum_drift"]),
    ]
    res.details = (f"e={e}, a={a:.6g} km, period={period / 86400:.2f} d, {frames} frames of dt={ctx.dt} s, "
                   f"positions compared with the analytic Kepler solution in the barycentric frame")
    res.series = {
        "position_error": _series(t_orbits, pos_err, "orbits", "max |Δr| / a"),
        "energy_drift": _series(t_orbits, e_drift, "orbits", "|ΔE| / |E0|"),
    }
    res.context = {"scale_name": "orbit radius", "scale_km": a, "duration_days": frames * ctx.dt / 86400,
                   "eccentricity": e, "period_days": period / 86400}
    return res


def check_planetary_reference(ctx: Context, cfg: dict) -> Result:
    res = Result("planetary_reference", cfg.get("category", "correctness"))
    bodies = scenes.planetary(int(cfg["planets"]), int(cfg["seed"]), G=ctx.G)
    scene = ctx.scene("planetary_reference", bodies)
    frames = int(cfg["frames"])
    every = max(1, frames // int(cfg["samples"]))
    sim = ctx.runner.simulate(scene, ctx.dt, frames, every, tag="planetary-sim")
    ref = ctx.runner.reference(scene, ctx.dt, frames, every, ctx.ref_substeps, softening=ctx.ref_softening,
                               tag="planetary-ref")
    pos_err = [_max_rel_position_error(s, r) for s, r in zip(sim["samples"], ref["samples"])]
    vel_err = [_max_rel_velocity_error(s, r) for s, r in zip(sim["samples"], ref["samples"])]
    days = [s["t"] / 86400 for s in sim["samples"]]
    res.metrics = [
        Metric("max_rel_position_error", max(pos_err), cfg["max_rel_position_error"]),
        Metric("max_rel_velocity_error", max(vel_err), cfg["max_rel_velocity_error"]),
    ]
    res.details = (f"{len(bodies)} bodies (star + {cfg['planets']} planets), {frames} frames, compared with the "
                   f"long double RK4 reference ({ctx.ref_substeps} substeps/frame); errors relative to the RMS "
                   f"distance to the barycenter / RMS speed")
    res.series = {
        "position_error": _series(days, pos_err, "days", "max |Δr| / RMS radius"),
        "velocity_error": _series(days, vel_err, "days", "max |Δv| / RMS speed"),
    }
    res.context = {"scale_name": "system size", "scale_km": _scene_scale(ref["samples"][0]),
                   "speed_scale_km_s": _rms_speed(ref["samples"][0]), "duration_days": frames * ctx.dt / 86400,
                   "bodies": len(bodies)}
    return res


def check_conservation(ctx: Context, cfg: dict) -> Result:
    res = Result("conservation", cfg.get("category", "correctness"))
    bodies = scenes.cluster(int(cfg["bodies"]), int(cfg["seed"]))
    p_scale, l_scale, total_mass = _mass_scales(bodies)
    frames = int(cfg["frames"])
    every = max(1, frames // int(cfg["samples"]))
    sim = ctx.runner.simulate(ctx.scene("conservation", bodies), ctx.dt, frames, every, snapshots=False,
                              tag="conservation")
    samples = sim["samples"]
    e_drift = _rel_drift(samples, "energy")
    p_drift = _rel_drift(samples, "momentum", p_scale)
    l_drift = _rel_drift(samples, "angular_momentum", l_scale)
    v_com = [c / total_mass for c in samples[0]["momentum"]]
    com0 = samples[0]["center_of_mass"]
    scale = math.sqrt(sum(sum((b["components"]["Position"][k] - com0[i]) ** 2 for i, k in enumerate("xyz"))
                          for b in bodies) / len(bodies))
    com_drift = [_norm([c - (c0 + v * s["t"]) for c, c0, v in zip(s["center_of_mass"], com0, v_com)]) / scale
                 for s in samples]
    days = [s["t"] / 86400 for s in samples]
    res.metrics = [
        Metric("max_rel_energy_drift", max(e_drift), cfg["max_rel_energy_drift"]),
        Metric("max_rel_momentum_drift", max(p_drift), cfg["max_rel_momentum_drift"]),
        Metric("max_rel_angular_momentum_drift", max(l_drift), cfg["max_rel_angular_momentum_drift"]),
        Metric("max_rel_barycenter_drift", max(com_drift), cfg["max_rel_barycenter_drift"]),
    ]
    res.details = (f"{len(bodies)}-body random cluster over {frames} frames; momentum and angular momentum "
                   f"normalised by Σm|v| and Σm|r||v|, barycenter drift by the RMS radius")
    res.series = {
        "energy_drift": _series(days, e_drift, "days", "|ΔE| / |E0|"),
        "momentum_drift": _series(days, p_drift, "days", "|ΔP| / Σm|v|"),
        "angular_momentum_drift": _series(days, l_drift, "days", "|ΔL| / Σm|r||v|"),
    }
    res.context = {"scale_name": "cluster size", "scale_km": scale, "duration_days": frames * ctx.dt / 86400,
                   "bodies": len(bodies)}
    return res


def check_energy_bounded(ctx: Context, cfg: dict) -> Result:
    res = Result("energy_bounded", cfg.get("category", "integrator"))
    bodies, m1, m2, a, e, period = _two_body(cfg, ctx)
    orbits = int(cfg["orbits"])
    per_orbit = int(cfg["samples_per_orbit"])
    frames_per_orbit = max(1, int(round(period / ctx.dt)))
    frames = frames_per_orbit * orbits
    every = max(1, frames_per_orbit // per_orbit)
    sim = ctx.runner.simulate(ctx.scene("energy_bounded", bodies), ctx.dt, frames, every, snapshots=False,
                              tag="energy-bounded")
    samples = sim["samples"]
    e_drift = _rel_drift(samples, "energy")
    t_orbits = [s["t"] / period for s in samples]
    first = max((d for d, t in zip(e_drift, t_orbits) if t <= 1.0), default=0.0)
    last = max((d for d, t in zip(e_drift, t_orbits) if t >= orbits - 1.0), default=0.0)
    growth = last / first if first > 0 else (0.0 if last == 0 else math.inf)
    res.metrics = [Metric("energy_error_growth_factor", growth, cfg["max_growth_factor"]),
                   Metric("max_rel_energy_drift", max(e_drift), None)]
    res.details = (f"two-body e={e} over {orbits} orbits: max |ΔE/E0| in the last orbit divided by the max in "
                   f"the first orbit. A symplectic integrator keeps the energy error bounded (factor ≈ 1).")
    res.series = {"energy_drift": _series(t_orbits, e_drift, "orbits", "|ΔE| / |E0|")}
    res.context = {"orbits": orbits, "eccentricity": e, "first_orbit_max_drift": first, "last_orbit_max_drift": last}
    return res


def check_reversibility(ctx: Context, cfg: dict) -> Result:
    res = Result("reversibility", cfg.get("category", "integrator"))
    bodies = scenes.cluster(int(cfg["bodies"]), int(cfg["seed"]))
    frames = int(cfg["frames"])
    sim = ctx.runner.simulate(ctx.scene("reversibility", bodies), ctx.dt, 2 * frames, frames, reverse_at=frames,
                              tag="reversibility")
    start, end = sim["samples"][0], sim["samples"][-1]
    scale = _scene_scale(start)
    speed = _rms_speed(start)
    pos_err = _max_rel_position_error(end, start, scale)
    vel_err = max(_dist(v, [-c for c in v0]) for v, v0 in zip(end["velocities"], start["velocities"])) / speed
    res.metrics = [Metric("max_rel_position_error", pos_err, cfg["max_rel_position_error"]),
                   Metric("max_rel_velocity_error", vel_err, cfg["max_rel_velocity_error"])]
    res.details = (f"{len(bodies)} bodies integrated {frames} frames forward, velocities negated, {frames} frames "
                   f"more: a time-reversible integrator returns to the initial state up to round-off")
    res.context = {"scale_name": "cluster size", "scale_km": scale, "bodies": len(bodies), "frames": frames}
    return res


def check_convergence_order(ctx: Context, cfg: dict) -> Result:
    res = Result("convergence_order", cfg.get("category", "integrator"))
    bodies, m1, m2, a, e, period = _two_body(cfg, ctx)
    scene = ctx.scene("convergence_order", bodies)
    levels = [float(x) for x in cfg["dt_levels"]]
    duration = float(cfg["duration_s"])
    errors = []
    for dt in levels:
        frames = max(1, int(round(duration / dt)))
        sim = ctx.runner.simulate(scene, dt, frames, frames, tag=f"convergence-{int(dt)}")
        errors.append(_kepler_errors(sim["samples"][-1:], m1, m2, a, e, ctx.G)[0])
    orders = []
    for (dt0, e0), (dt1, e1) in zip(zip(levels, errors), zip(levels[1:], errors[1:])):
        if e0 > 0 and e1 > 0 and dt0 != dt1:
            orders.append(math.log(e0 / e1) / math.log(dt0 / dt1))
    deviation = max((abs(o - cfg["expected_order"]) for o in orders), default=math.inf)
    res.metrics = [Metric("max_order_deviation", deviation, cfg["order_tolerance"]),
                   Metric("min_observed_order", min(orders) if orders else None, None)]
    res.details = (f"two-body e={e} integrated for {duration / 86400:.1f} days with dt in {levels}; error at the end "
                   f"vs the analytic solution. Observed orders between successive levels: "
                   f"{', '.join(f'{o:.2f}' for o in orders)} (expected {cfg['expected_order']})")
    res.series = {"error_vs_dt": {**_series(levels, errors, "dt (s)", "|Δr| / a"), "log_x": True}}
    res.context = {"orders": orders, "expected_order": cfg["expected_order"], "dt_levels": levels,
                   "duration_days": duration / 86400}
    return res


def check_determinism(ctx: Context, cfg: dict) -> Result:
    res = Result("determinism", cfg.get("category", "robustness"))
    scene = ctx.scene("determinism", scenes.cluster(int(cfg["bodies"]), int(cfg["seed"])))
    frames = int(cfg["frames"])
    a = ctx.runner.simulate(scene, ctx.dt, frames, frames, tag="determinism-a")["samples"][-1]
    b = ctx.runner.simulate(scene, ctx.dt, frames, frames, tag="determinism-b")["samples"][-1]
    mismatches = sum(1 for p, q in zip(a["positions"], b["positions"]) if p != q)
    mismatches += sum(1 for p, q in zip(a["velocities"], b["velocities"]) if p != q)
    res.metrics = [Metric("mismatching_values", float(mismatches), 0.0, "==")]
    res.details = f"{cfg['bodies']} bodies, {frames} frames, two separate processes must agree bitwise"
    return res


def check_permutation_invariance(ctx: Context, cfg: dict) -> Result:
    res = Result("permutation_invariance", cfg.get("category", "robustness"))
    bodies = scenes.cluster(int(cfg["bodies"]), int(cfg["seed"]))
    shuffled, order = scenes.shuffled(bodies, int(cfg["seed"]) + 1)
    frames = int(cfg["frames"])
    a = ctx.runner.simulate(ctx.scene("permutation_a", bodies), ctx.dt, frames, frames, tag="perm-a")["samples"][-1]
    b = ctx.runner.simulate(ctx.scene("permutation_b", shuffled), ctx.dt, frames, frames, tag="perm-b")["samples"][-1]
    scale = _scene_scale(a)
    err = max(_dist(a["positions"][orig], b["positions"][new]) for new, orig in enumerate(order)) / scale
    res.metrics = [Metric("max_rel_error", err, cfg["max_rel_error"])]
    res.details = f"{len(bodies)} bodies, {frames} frames, same scene in a shuffled entity order"
    return res


def check_translation_invariance(ctx: Context, cfg: dict) -> Result:
    res = Result("translation_invariance", cfg.get("category", "robustness"))
    bodies = scenes.cluster(int(cfg["bodies"]), int(cfg["seed"]))
    offset = [float(x) for x in cfg["offset_km"]]
    frames = int(cfg["frames"])
    a = ctx.runner.simulate(ctx.scene("translation_a", bodies), ctx.dt, frames, frames, tag="trans-a")["samples"][-1]
    b = ctx.runner.simulate(ctx.scene("translation_b", scenes.translated(bodies, offset)), ctx.dt, frames, frames,
                            tag="trans-b")["samples"][-1]
    scale = _scene_scale(a)
    err = max(_dist([p + o for p, o in zip(pa, offset)], pb) for pa, pb in zip(a["positions"], b["positions"])) / scale
    res.metrics = [Metric("max_rel_error", err, cfg["max_rel_error"])]
    res.details = f"{len(bodies)} bodies, {frames} frames, scene shifted by {offset} km"
    return res


def check_padding_sizes(ctx: Context, cfg: dict) -> Result:
    res = Result("padding_sizes", cfg.get("category", "robustness"))
    frames = int(cfg["frames"])
    worst = 0.0
    per_size = []
    for n in cfg["sizes"]:
        scene = ctx.scene(f"padding_{n}", scenes.cluster(int(n), int(cfg["seed"])))
        sim = ctx.runner.simulate(scene, ctx.dt, frames, frames, tag=f"padding-{n}")
        ref = ctx.runner.reference(scene, ctx.dt, frames, frames, ctx.ref_substeps, softening=ctx.ref_softening,
                                   tag=f"padding-ref-{n}")
        err = _max_rel_position_error(sim["samples"][-1], ref["samples"][-1])
        if sim["samples"][-1]["non_finite"]:
            err = math.inf
        per_size.append((n, err))
        worst = max(worst, err)
    res.metrics = [Metric("max_rel_position_error", worst, cfg["max_rel_position_error"])]
    res.details = "body counts around SIMD block boundaries vs the reference after %d frames: " % frames + ", ".join(
        f"{n}: {e:.1e}" for n, e in per_size)
    res.series = {"error_vs_size": _series([n for n, _ in per_size], [e for _, e in per_size], "bodies",
                                           "max |Δr| / RMS radius")}
    res.context = {"scale_name": "cluster size", "sizes": list(cfg["sizes"]), "frames": frames}
    return res


def check_single_body_inertia(ctx: Context, cfg: dict) -> Result:
    res = Result("single_body_inertia", cfg.get("category", "robustness"))
    bodies = scenes.single()
    frames = int(cfg["frames"])
    sim = ctx.runner.simulate(ctx.scene("single_body", bodies), ctx.dt, frames, frames, tag="single")
    c = bodies[0]["components"]
    t = frames * ctx.dt
    expected = [c["Position"][k] + c["Velocity"][k] * t for k in "xyz"]
    travelled = _norm([c["Velocity"][k] * t for k in "xyz"])
    end = sim["samples"][-1]
    err = _dist(end["positions"][0], expected) / travelled
    v_err = _dist(end["velocities"][0], [c["Velocity"][k] for k in "xyz"]) / _norm([c["Velocity"][k] for k in "xyz"])
    res.metrics = [Metric("max_rel_error", max(err, v_err), cfg["max_rel_error"])]
    res.details = f"an isolated body must move in a straight line at constant velocity ({frames} frames)"
    res.context = {"scale_name": "distance travelled", "scale_km": travelled, "frames": frames}
    return res


def check_empty_world(ctx: Context, cfg: dict) -> Result:
    res = Result("empty_world", cfg.get("category", "robustness"))
    sim = ctx.runner.simulate(ctx.scene("empty", []), ctx.dt, int(cfg["frames"]), tag="empty")
    res.metrics = [Metric("bodies_out", float(len(sim["samples"][-1]["positions"])), 0.0, "==")]
    res.details = "init/update/syncOut with zero entities must not crash"
    return res


def check_coincident_bodies(ctx: Context, cfg: dict) -> Result:
    res = Result("coincident_bodies", cfg.get("category", "robustness"))
    sim = ctx.runner.simulate(ctx.scene("coincident", scenes.coincident()), ctx.dt, int(cfg["frames"]),
                              tag="coincident")
    res.metrics = [Metric("non_finite_values", float(sim["samples"][-1]["non_finite"]), 0.0, "==")]
    res.details = "two bodies at the exact same position must not produce NaN or infinities"
    return res


def check_massless_body_moves(ctx: Context, cfg: dict) -> Result:
    res = Result("massless_body_moves", cfg.get("category", "robustness"), required=bool(cfg.get("required", True)))
    bodies = scenes.massless_probe(G=ctx.G)
    frames = int(cfg["frames"])
    sim = ctx.runner.simulate(ctx.scene("massless_probe", bodies), ctx.dt, frames, frames, tag="massless")
    c = bodies[1]["components"]
    a = c["Position"]["x"]
    m_star = scenes.effective_mass(*[bodies[0]["components"]["Mass"][k] for k in ("mantissa", "exponent")])
    angle = math.sqrt(ctx.G * m_star / a**3) * frames * ctx.dt
    expected = [a * math.cos(angle), a * math.sin(angle), 0.0]
    err = _dist(sim["samples"][-1]["positions"][1], expected) / a
    res.metrics = [Metric("max_rel_position_error", err, cfg["max_rel_position_error"])]
    res.details = ("a massless test particle on a circular orbit must still be accelerated by the star "
                   f"(compared with the analytic circular orbit after {frames} frames)")
    res.context = {"scale_name": "orbit radius", "scale_km": a, "frames": frames}
    return res


def check_large_scene_smoke(ctx: Context, cfg: dict) -> Result:
    res = Result("large_scene_smoke", cfg.get("category", "robustness"))
    scene = ctx.scene(f"large_{cfg['bodies']}", scenes.cluster(int(cfg["bodies"]), int(cfg["seed"])))
    sim = ctx.runner.simulate(scene, ctx.dt, int(cfg["frames"]), snapshots=False, potential=False, tag="large")
    res.metrics = [Metric("non_finite_values", float(sim["samples"][-1]["non_finite"]), 0.0, "==")]
    res.details = f"{cfg['bodies']} bodies for {cfg['frames']} frames must complete without NaN"
    return res


CHECKS = {
    "kepler_circular": lambda ctx, cfg: check_kepler(ctx, cfg, "kepler_circular"),
    "kepler_eccentric": lambda ctx, cfg: check_kepler(ctx, cfg, "kepler_eccentric"),
    "planetary_reference": check_planetary_reference,
    "conservation": check_conservation,
    "energy_bounded": check_energy_bounded,
    "reversibility": check_reversibility,
    "convergence_order": check_convergence_order,
    "determinism": check_determinism,
    "permutation_invariance": check_permutation_invariance,
    "translation_invariance": check_translation_invariance,
    "padding_sizes": check_padding_sizes,
    "single_body_inertia": check_single_body_inertia,
    "empty_world": check_empty_world,
    "coincident_bodies": check_coincident_bodies,
    "massless_body_moves": check_massless_body_moves,
    "large_scene_smoke": check_large_scene_smoke,
}


def run_all(ctx: Context, thresholds: dict, only: list[str] | None = None) -> dict:
    results = []
    for name, fn in CHECKS.items():
        cfg = thresholds["checks"].get(name)
        if cfg is None or not cfg.get("enabled", True) or (only and name not in only):
            continue
        print(f"[check] {name} ...", end=" ", flush=True)
        try:
            result = fn(ctx, cfg)
        except Exception as exc:  # noqa: BLE001 - a crashing check must not stop the suite
            result = Result(name, cfg.get("category", "robustness"), required=bool(cfg.get("required", True)),
                            error=f"{exc}\n{traceback.format_exc()}")
        worst = ""
        if result.metrics:
            m = result.metrics[0]
            worst = f"{m.name}={m.value:.3e}" + (f" (limit {m.threshold:.1e})" if m.threshold is not None else "")
        print(f"{result.status.upper()} {worst}", flush=True)
        results.append(result.to_dict())
    summary = {s: sum(1 for r in results if r["status"] == s) for s in ("pass", "fail", "warn", "error")}
    return {"dt": ctx.dt, "G": ctx.G, "reference": {"substeps": ctx.ref_substeps, "softening_km": ctx.ref_softening},
            "summary": summary, "results": results}
