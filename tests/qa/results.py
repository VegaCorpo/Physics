"""Persistent results: results/<label>/<BuildType>/{meta,correctness,benchmark}.json."""

from __future__ import annotations

import json
import platform
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .paths import RESULTS_DIR


def cpu_model() -> str:
    try:
        text = Path("/proc/cpuinfo").read_text(encoding="utf-8")
        match = re.search(r"^model name\s*:\s*(.*)$", text, re.M)
        if match:
            return match.group(1).strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def cpu_governor() -> str:
    path = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return "unknown"


def machine_meta() -> dict:
    return {
        "host": platform.node(),
        "os": f"{platform.system()} {platform.release()}",
        "cpu": cpu_model(),
        "governor": cpu_governor(),
        "python": platform.python_version(),
    }


def result_dir(label: str, build_type: str, root: Path = RESULTS_DIR) -> Path:
    return root / label / build_type


def save(label: str, build_type: str, meta: dict, correctness: dict | None, benchmark: dict | None,
         root: Path = RESULTS_DIR) -> Path:
    target = result_dir(label, build_type, root)
    target.mkdir(parents=True, exist_ok=True)
    existing = load_one(target)
    meta = {**(existing.get("meta") or {}), **meta, "saved_at": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    with open(target / "meta.json", "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2)
    if correctness is not None:
        with open(target / "correctness.json", "w", encoding="utf-8") as handle:
            json.dump(correctness, handle, indent=1)
    if benchmark is not None:
        with open(target / "benchmark.json", "w", encoding="utf-8") as handle:
            json.dump(benchmark, handle, indent=1)
    return target


@dataclass
class Entry:
    label: str
    build_type: str
    meta: dict
    correctness: dict | None
    benchmark: dict | None
    path: Path

    @property
    def commit_date(self) -> str:
        return self.meta.get("commit_date", "")

    @property
    def title(self) -> str:
        return self.label


def load_one(target: Path) -> dict:
    out: dict = {}
    for name in ("meta", "correctness", "benchmark"):
        path = target / f"{name}.json"
        if path.exists():
            with open(path, encoding="utf-8") as handle:
                out[name] = json.load(handle)
    return out


def load_all(root: Path = RESULTS_DIR, build_type: str | None = None) -> list[Entry]:
    entries: list[Entry] = []
    if not root.exists():
        return entries
    for label_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        for type_dir in sorted(p for p in label_dir.iterdir() if p.is_dir()):
            if build_type and type_dir.name != build_type:
                continue
            data = load_one(type_dir)
            if "meta" not in data:
                continue
            entries.append(Entry(label=label_dir.name, build_type=type_dir.name, meta=data["meta"],
                                 correctness=data.get("correctness"), benchmark=data.get("benchmark"),
                                 path=type_dir))
    # Chronological by commit date, dirty/prebuilt snapshots after their base commit.
    entries.sort(key=lambda e: (e.commit_date, e.meta.get("dirty", False), e.label))
    return entries


def select(entries: list[Entry], labels: list[str]) -> list[Entry]:
    """Resolve user-given labels (full label, or a prefix of the label / commit hash)."""
    out = []
    for wanted in labels:
        exact = [e for e in entries if e.label == wanted]
        prefix = [e for e in entries if e.label.startswith(wanted) or e.meta.get("commit", "").startswith(wanted)]
        matches = exact or prefix
        if not matches:
            raise KeyError(f"no stored result matches '{wanted}'")
        out.append(matches[0])
    return out
