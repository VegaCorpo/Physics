from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from . import kepler, scenes
from .checks import Context, Metric, Result, _kepler_errors, _rel_drift, _scene_scale, _series, _two_body

CATEGORY = "timestep"
SECONDS_PER_DAY = 86400.0
SECONDS_PER_JULIAN_YEAR = 31557600.0
STAR, PLANET, MOON = 0, 1, 2
POSITION_ERROR_LABEL = "|Δr| / a"


@dataclass(frozen=True)
class Orbit:
    primary_mass: float
    secondary_mass: float
    semi_major_axis: float
    eccentricity: float
    period: float

    def position_errors(self, samples: list[dict], gravity: float) -> list[float]:
        return _kepler_errors(samples, self.primary_mass, self.secondary_mass, self.semi_major_axis,
                              self.eccentricity, gravity)


@dataclass(frozen=True)
class OrbitScenario:
    scene: Path
    orbit: Orbit
    duration: float


@dataclass(frozen=True)
class ScaleOutcome:
    position_error: float
    energy_drift: float


@dataclass(frozen=True)
class SplitBaseline:
    scene: Path
    dt: float
    frames: int
    final: dict


@dataclass(frozen=True)
class FixedRun:
    scene: Path
    frames: int


@dataclass(frozen=True)
class SatelliteRun:
    module: list[dict]
    reference: list[dict]


def _orbit_scene(ctx: Context, cfg: dict, name: str) -> tuple[Path, Orbit]:
    bodies, primary_mass, secondary_mass, semi_major_axis, eccentricity, period = _two_body(cfg, ctx)
    orbit = Orbit(primary_mass, secondary_mass, semi_major_axis, eccentricity, period)
    return ctx.scene(name, bodies), orbit


def _cluster_scene(ctx: Context, cfg: dict, name: str) -> Path:
    return ctx.scene(name, scenes.cluster(int(cfg["bodies"]), int(cfg["seed"])))


def _vectors(sample: dict) -> list[list[float]]:
    return [*sample["positions"], *sample["velocities"]]


def _mismatching_values(first: dict, last: dict) -> int:
    return sum(1 for before, after in zip(_vectors(first), _vectors(last)) if before != after)


def _max_relative_gap(reference: dict, candidate: dict, scale: float) -> float:
    gaps = (math.dist(expected, actual) for expected, actual in zip(reference["positions"], candidate["positions"]))
    return max(gaps, default=0.0) / scale


def _outcome_at_scale(ctx: Context, scenario: OrbitScenario, dt: float) -> ScaleOutcome:
    frames = max(1, round(scenario.duration / dt))
    simulation = ctx.runner.simulate(scenario.scene, dt, frames, frames, tag=f"ui-scale-{int(dt)}")
    samples = simulation["samples"]
    return ScaleOutcome(max(scenario.orbit.position_errors(samples[-1:], ctx.G)), _rel_drift(samples, "energy")[-1])


def check_ui_scales(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_ui_scales", cfg.get("category", CATEGORY))
    scene, orbit = _orbit_scene(ctx, cfg, "timestep_ui_scales")
    scenario = OrbitScenario(scene, orbit, float(cfg["duration_s"]))
    levels = [float(level) for level in cfg["dt_levels_s"]]
    outcomes = [_outcome_at_scale(ctx, scenario, dt) for dt in levels]
    errors = [outcome.position_error for outcome in outcomes]
    result.metrics = [
        Metric("max_rel_position_error", max(errors), cfg["max_rel_position_error"]),
        Metric("max_rel_energy_drift", max(outcome.energy_drift for outcome in outcomes), cfg["max_rel_energy_drift"]),
    ]
    result.details = (f"the same orbit integrated for {scenario.duration / SECONDS_PER_DAY:.0f} days with one "
                      f"update() per time scale in {levels} s; final position vs the analytic Kepler solution")
    result.series = {"error_vs_dt": {**_series(levels, errors, "dt (s)", POSITION_ERROR_LABEL), "log_x": True}}
    result.context = {"scale_name": "orbit radius", "scale_km": orbit.semi_major_axis, "dt_levels": levels,
                      "duration_days": scenario.duration / SECONDS_PER_DAY}
    return result


def _split_frames(baseline: SplitBaseline, multiplier: int) -> int:
    if baseline.frames % multiplier:
        raise ValueError(f"base_frames={baseline.frames} is not a multiple of {multiplier}")
    return baseline.frames // multiplier


def _split_gap(ctx: Context, baseline: SplitBaseline, multiplier: int) -> float:
    frames = _split_frames(baseline, multiplier)
    dt = baseline.dt * multiplier
    final = ctx.runner.simulate(baseline.scene, dt, frames, frames, tag=f"split-x{multiplier}")["samples"][-1]
    return _max_relative_gap(baseline.final, final, _scene_scale(baseline.final))


def check_split_invariance(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_split_invariance", cfg.get("category", CATEGORY))
    scene = _cluster_scene(ctx, cfg, "timestep_split_invariance")
    dt, frames = float(cfg["base_dt_s"]), int(cfg["base_frames"])
    final = ctx.runner.simulate(scene, dt, frames, frames, tag="split-x1")["samples"][-1]
    baseline = SplitBaseline(scene, dt, frames, final)
    multipliers = [int(multiplier) for multiplier in cfg["multipliers"]]
    gaps = [_split_gap(ctx, baseline, multiplier) for multiplier in multipliers]
    result.metrics = [Metric("max_rel_position_error", max(gaps), cfg["max_rel_position_error"])]
    result.details = (f"{cfg['bodies']} bodies over {frames * dt / SECONDS_PER_DAY:.1f} days: {frames} updates of "
                      f"{dt:g} s compared with fewer, longer updates (×{multipliers}); " +
                      ", ".join(f"×{multiplier}: {gap:.1e}" for multiplier, gap in zip(multipliers, gaps)))
    result.series = {"gap_vs_dt": {**_series([dt * multiplier for multiplier in multipliers], gaps, "dt (s)",
                                             "max |Δr| / RMS radius"), "log_x": True}}
    result.context = {"scale_name": "cluster size", "scale_km": _scene_scale(final), "multipliers": multipliers}
    return result


def check_changes_mid_run(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_changes_mid_run", cfg.get("category", CATEGORY))
    scene, orbit = _orbit_scene(ctx, cfg, "timestep_changes_mid_run")
    schedule = [(float(dt), int(frames)) for dt, frames in cfg["schedule"]]
    samples = ctx.runner.simulate_schedule(scene, schedule, tag="mid-run")["samples"]
    errors = orbit.position_errors(samples, ctx.G)
    days = [sample["t"] / SECONDS_PER_DAY for sample in samples]
    result.metrics = [
        Metric("max_rel_position_error", max(errors), cfg["max_rel_position_error"]),
        Metric("max_rel_energy_drift", max(_rel_drift(samples, "energy")), cfg["max_rel_energy_drift"]),
    ]
    result.details = (f"two-body e={orbit.eccentricity} on a single engine whose dt changes between updates "
                      f"(dt s × updates: {schedule}); every frame compared with the analytic Kepler solution")
    result.series = {"position_error": _series(days, errors, "days", POSITION_ERROR_LABEL)}
    result.context = {"scale_name": "orbit radius", "scale_km": orbit.semi_major_axis, "schedule": schedule,
                      "duration_days": days[-1]}
    return result


def check_zero_dt(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_zero_dt", cfg.get("category", CATEGORY))
    frames = int(cfg["frames"])
    samples = ctx.runner.simulate(_cluster_scene(ctx, cfg, "timestep_zero_dt"), 0.0, frames, frames,
                                  tag="zero-dt")["samples"]
    result.metrics = [Metric("mismatching_values", float(_mismatching_values(samples[0], samples[-1])), 0.0, "==")]
    result.details = (f"{cfg['bodies']} bodies, {frames} updates with dt = 0 must leave every position and velocity "
                      "unchanged")
    return result


def _invalid_dt_damage(ctx: Context, run: FixedRun, dt: float) -> int:
    samples = ctx.runner.simulate(run.scene, dt, run.frames, run.frames, tag=f"invalid-dt-{dt!r}")["samples"]
    return _mismatching_values(samples[0], samples[-1]) + samples[-1]["non_finite"]


def check_invalid_dt(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_invalid_dt", cfg.get("category", CATEGORY))
    scene = _cluster_scene(ctx, cfg, "timestep_invalid_dt")
    frames = int(cfg["frames"])
    invalid = [float(dt) for dt in cfg["invalid_dt"]]
    damage = [_invalid_dt_damage(ctx, FixedRun(scene, frames), dt) for dt in invalid]
    result.metrics = [Metric("changed_or_non_finite_values", float(sum(damage)), 0.0, "==")]
    result.details = (f"{cfg['bodies']} bodies, {frames} updates with each invalid dt; the state must be left "
                      "untouched and finite. Changed or non-finite values per dt: " +
                      ", ".join(f"{dt!r}: {count}" for dt, count in zip(invalid, damage)))
    result.context = {"invalid_dt": [repr(dt) for dt in invalid], "frames": frames}
    return result


def _satellite_system(cfg: dict) -> scenes.SatelliteSystem:
    return scenes.SatelliteSystem(tuple(cfg["star_mass"]), tuple(cfg["planet_mass"]), tuple(cfg["moon_mass"]),
                                  float(cfg["planet_distance_km"]), float(cfg["moon_distance_km"]))


def _moon_period(system: scenes.SatelliteSystem, gravity: float) -> float:
    pair_mass = scenes.effective_mass(*system.planet_mass) + scenes.effective_mass(*system.moon_mass)
    return kepler.period(system.moon_distance, gravity * pair_mass)


def _offset(sample: dict, body: int, origin: int) -> list[float]:
    return [position - center for position, center in zip(sample["positions"][body], sample["positions"][origin])]


def _relative_errors(run: SatelliteRun, body: int, origin: int) -> list[float]:
    scale = math.hypot(*_offset(run.reference[0], body, origin))
    pairs = zip(run.module, run.reference)
    return [math.dist(_offset(module, body, origin), _offset(reference, body, origin)) / scale
            for module, reference in pairs]


def _satellite_run(ctx: Context, cfg: dict, scene: Path) -> SatelliteRun:
    dt, frames = float(cfg["dt_s"]), int(cfg["frames"])
    module = ctx.runner.simulate(scene, dt, frames, 1, tag="satellite-sim")["samples"]
    reference = ctx.runner.reference(scene, dt, frames, 1, int(cfg["reference_substeps"]), softening=ctx.ref_softening,
                                     tag="satellite-ref")["samples"]
    return SatelliteRun(module, reference)


def _satellite_details(cfg: dict, years: float, lunar_orbits: float) -> str:
    return (f"star, planet and moon advanced one update of {cfg['dt_s']:g} s at a time for {years:.0f} years "
            f"({lunar_orbits:.0f} lunar orbits); moon position around the planet compared every update with "
            f"the long double RK4 reference ({cfg['reference_substeps']} substeps per update)")


def check_satellite(ctx: Context, cfg: dict) -> Result:
    result = Result("timestep_satellite", cfg.get("category", CATEGORY), required=bool(cfg.get("required", True)))
    system = _satellite_system(cfg)
    run = _satellite_run(ctx, cfg, ctx.scene("timestep_satellite", scenes.satellite_system(system, ctx.G)))
    moon_errors = _relative_errors(run, MOON, PLANET)
    years = [sample["t"] / SECONDS_PER_JULIAN_YEAR for sample in run.module]
    lunar_orbits = years[-1] * SECONDS_PER_JULIAN_YEAR / _moon_period(system, ctx.G)
    result.metrics = [
        Metric("max_rel_moon_position_error", max(moon_errors), cfg["max_rel_moon_position_error"]),
        Metric("max_rel_planet_position_error", max(_relative_errors(run, PLANET, STAR)),
               cfg["max_rel_planet_position_error"]),
    ]
    result.details = _satellite_details(cfg, years[-1], lunar_orbits)
    result.series = {"moon_position_error": _series(years, moon_errors, "years", "|Δr moon| / moon orbit radius")}
    result.context = {"scale_name": "moon orbit radius", "scale_km": system.moon_distance,
                      "duration_days": years[-1] * SECONDS_PER_JULIAN_YEAR / SECONDS_PER_DAY,
                      "lunar_orbits": lunar_orbits}
    return result

CHECKS = {
    "timestep_ui_scales": check_ui_scales,
    "timestep_split_invariance": check_split_invariance,
    "timestep_changes_mid_run": check_changes_mid_run,
    "timestep_satellite": check_satellite,
    "timestep_zero_dt": check_zero_dt,
    "timestep_invalid_dt": check_invalid_dt,
}
