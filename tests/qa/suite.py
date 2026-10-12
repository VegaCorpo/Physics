from __future__ import annotations

import traceback
from collections.abc import Callable
from typing import NamedTuple

from . import checks, timestep
from .checks import Context, Result

REGISTRY: dict[str, Callable[[Context, dict], Result]] = {**checks.CHECKS, **timestep.CHECKS}
STATUSES = ("pass", "fail", "warn", "error")


class SelectedCheck(NamedTuple):
    name: str
    run: Callable[[Context, dict], Result]
    config: dict


def _is_selected(name: str, config: dict | None, only: list[str] | None) -> bool:
    if config is None or not config.get("enabled", True):
        return False
    return not only or name in only


def _selected(thresholds: dict, only: list[str] | None) -> list[SelectedCheck]:
    configs = thresholds["checks"]
    return [SelectedCheck(name, run, configs[name]) for name, run in REGISTRY.items()
            if _is_selected(name, configs.get(name), only)]


def _run_one(ctx: Context, selected: SelectedCheck) -> Result:
    try:
        return selected.run(ctx, selected.config)
    except Exception as exc:
        return Result(selected.name, selected.config.get("category", "robustness"),
                      required=bool(selected.config.get("required", True)),
                      error=f"{exc}\n{traceback.format_exc()}")


def _headline(result: Result) -> str:
    if not result.metrics:
        return ""
    metric = result.metrics[0]
    limit = f" (limit {metric.threshold:.1e})" if metric.threshold is not None else ""
    return f"{metric.name}={metric.value:.3e}{limit}"


def _summarise(ctx: Context, results: list[dict]) -> dict:
    summary = {status: sum(1 for result in results if result["status"] == status) for status in STATUSES}
    return {"dt": ctx.dt, "G": ctx.G, "reference": {"substeps": ctx.ref_substeps, "softening_km": ctx.ref_softening},
            "summary": summary, "results": results}


def run_all(ctx: Context, thresholds: dict, only: list[str] | None = None) -> dict:
    results = []
    for selected in _selected(thresholds, only):
        print(f"[check] {selected.name} ...", end=" ", flush=True)
        result = _run_one(ctx, selected)
        print(f"{result.status.upper()} {_headline(result)}", flush=True)
        results.append(result.to_dict())
    return _summarise(ctx, results)
