"""Benchmark suite: frame time versus body count."""

from __future__ import annotations

import statistics
from pathlib import Path

from . import scenes
from .runner import Runner


def _stats(values: list[float]) -> dict:
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return {"median": None, "mean": None, "min": None, "max": None, "p95": None, "stdev": None, "samples": 0}
    p95 = ordered[min(n - 1, int(round(0.95 * (n - 1))))]
    return {
        "median": statistics.median(ordered),
        "mean": statistics.fmean(ordered),
        "min": ordered[0],
        "max": ordered[-1],
        "p95": p95,
        "stdev": statistics.pstdev(ordered) if n > 1 else 0.0,
        "samples": n,
    }


def run(runner: Runner, cfg: dict, scene_dir: Path, only: list[int] | None = None) -> dict:
    dt = float(cfg["dt"])
    results = []
    for n in cfg["body_counts"]:
        if only and n not in only:
            continue
        frames = int(cfg.get("frames_override", {}).get(str(n), cfg["frames"]))
        scene = scene_dir / f"cluster_{n}_seed{cfg['seed']}.json"
        if not scene.exists():
            scenes.write_scene(scene, scenes.cluster(n, cfg["seed"]))
        print(f"[bench] {n:>6} bodies: {cfg['repeats']} x ({cfg['warmup']} warmup + {frames} frames)", flush=True)
        raw = runner.bench(scene, dt, frames, cfg["warmup"], cfg["repeats"], tag=f"bench-{n}")
        update = [v for rep in raw["repeats"] for v in rep["update_ms"]]
        frame = [v for rep in raw["repeats"] for v in rep["frame_ms"]]
        init = [rep["init_ms"] for rep in raw["repeats"]]
        non_finite = sum(rep["non_finite"] for rep in raw["repeats"])
        entry = {
            "bodies": n,
            "frames": frames,
            "warmup": cfg["warmup"],
            "repeats": cfg["repeats"],
            "update_ms": _stats(update),
            "frame_ms": _stats(frame),
            "init_ms": _stats(init),
            "per_repeat_median_update_ms": [statistics.median(rep["update_ms"]) for rep in raw["repeats"]],
            "non_finite": non_finite,
        }
        med = entry["update_ms"]["median"]
        entry["pair_interactions_per_s"] = (n * n) / (med / 1000.0) if med else None
        print(f"         median update {med:.4f} ms, frame {entry['frame_ms']['median']:.4f} ms", flush=True)
        results.append(entry)
    return {
        "dt": dt,
        "engine": raw["engine"] if results else None,
        "config": {k: v for k, v in cfg.items() if not k.startswith("_")},
        "results": results,
    }


def compare(baseline: dict, candidate: dict, regression_pct: float, improvement_pct: float,
            min_ms: float = 0.0) -> list[dict]:
    """Per body count delta of the median update time, candidate versus baseline.

    Timings below `min_ms` are reported but never judged: at a few microseconds per
    frame the scheduler noise exceeds any real change.
    """
    base = {r["bodies"]: r for r in baseline.get("results", [])}
    rows = []
    for r in candidate.get("results", []):
        b = base.get(r["bodies"])
        if not b or not b["update_ms"]["median"] or not r["update_ms"]["median"]:
            continue
        delta = (r["update_ms"]["median"] / b["update_ms"]["median"] - 1.0) * 100.0
        if max(b["update_ms"]["median"], r["update_ms"]["median"]) < min_ms:
            verdict = "noise"
        else:
            verdict = "regression" if delta > regression_pct else "improvement" if delta < -improvement_pct else "stable"
        rows.append({"bodies": r["bodies"], "baseline_ms": b["update_ms"]["median"],
                     "candidate_ms": r["update_ms"]["median"], "delta_pct": delta, "verdict": verdict})
    return rows
