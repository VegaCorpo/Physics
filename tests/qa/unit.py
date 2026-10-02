"""Build and run the white-box unit tests (tests/unit) against a module checkout.

The black-box runner cannot observe collisions: common::IPhysicsEngine does not
expose the detected pairs. tests/unit compiles the module's Octree and Collider
sources directly and compares check_collisions() with a brute-force reference.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

from .build import BuildError, _run, checkout
from .paths import TESTS_DIR, WORK_DIR, cpm_cache

UNIT_DIR = TESTS_DIR / "unit"


def build(repo: Path, ref: str | None, build_type: str, jobs: int, local_common: Path | None = None,
          rebuild: bool = False) -> tuple[str, Path]:
    """Configure and build tests/unit for the sources of `ref`; returns (label, executable)."""
    co = checkout(repo, ref, rebuild)
    work = WORK_DIR / co.label
    work.mkdir(parents=True, exist_ok=True)
    log = work / f"unit-{build_type}.log"
    build_dir = work / f"unit-{build_type}"
    if rebuild:
        shutil.rmtree(build_dir, ignore_errors=True)
        if log.exists():
            log.unlink()
    common_args = [f"-DCPM_Common_SOURCE={local_common.resolve()}"] if local_common else []

    print(f"[unit] {co.label} ({build_type}) {co.subject}", flush=True)
    _run(["cmake", "-S", str(UNIT_DIR), "-B", str(build_dir), f"-DCMAKE_BUILD_TYPE={build_type}",
          f"-DPHYSICS_SOURCE_DIR={co.source}", f"-DCPM_SOURCE_CACHE={cpm_cache(repo.resolve())}", *common_args],
         log=log)
    _run(["cmake", "--build", str(build_dir), "-j", str(jobs)], log=log)
    exe = build_dir / "physics_unit_tests"
    if not exe.exists():
        raise BuildError(f"unit tests not built: {exe}")
    return co.label, exe


def run(exe: Path, gtest_filter: str | None = None, verbose: bool = False) -> int:
    """Run the GoogleTest binary, print a one-line summary and return its exit code."""
    report = exe.parent / "unit-report.json"
    if report.exists():
        report.unlink()
    cmd = [str(exe), f"--gtest_output=json:{report}", "--gtest_color=auto"]
    if not verbose:
        cmd.append("--gtest_brief=1")
    if gtest_filter:
        cmd.append(f"--gtest_filter={gtest_filter}")
    print(f"  $ {' '.join(cmd)}", flush=True)
    result = subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr)

    if report.exists():
        with open(report, encoding="utf-8") as handle:
            data = json.load(handle)
        failed = [f"{suite['name']}.{case['name']}" for suite in data.get("testsuites", [])
                  for case in suite.get("testsuite", []) if case.get("failures")]
        total = data.get("tests", 0)
        print(f"[unit] {total - len(failed)}/{total} passed" + (f", failed: {', '.join(failed)}" if failed else ""),
              flush=True)
    return result.returncode
