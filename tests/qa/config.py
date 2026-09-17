"""Configuration loading. Every threshold lives in config/*.json, never in code."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .paths import CONFIG_DIR


def _deep_merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_json(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def load_thresholds(override: Path | None = None) -> dict[str, Any]:
    base = load_json(CONFIG_DIR / "thresholds.json")
    if override:
        base = _deep_merge(base, load_json(override))
    return base


def load_benchmark(override: Path | None = None) -> dict[str, Any]:
    base = load_json(CONFIG_DIR / "benchmark.json")
    if override:
        base = _deep_merge(base, load_json(override))
    return base
