"""Filesystem layout of the QA tool.

tests/                  <- TESTS_DIR (this package's parent)
  config/               <- default configuration files
  results/<label>/<BuildType>/{meta,correctness,benchmark}.json
  .work/                <- worktrees, builds, scenes, raw runner output (gitignored)
"""

from __future__ import annotations

import os
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent.parent
DEFAULT_REPO = TESTS_DIR.parent
CONFIG_DIR = TESTS_DIR / "config"
RESULTS_DIR = TESTS_DIR / "results"
WORK_DIR = TESTS_DIR / ".work"


def cpm_cache(repo: Path) -> Path:
    """Reuse the engine's CPM cache when the module is checked out as a submodule."""
    env = os.environ.get("CPM_SOURCE_CACHE")
    if env:
        return Path(env).resolve()
    candidate = repo.parent / ".cpm_cache"
    if candidate.is_dir():
        return candidate
    return WORK_DIR / "cpm_cache"
