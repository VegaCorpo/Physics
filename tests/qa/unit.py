"""Build and run the white-box unit tests (tests/unit) against a module checkout.

The black-box runner cannot observe collisions: common::IPhysicsEngine does not
expose the detected pairs. tests/unit compiles the module's Octree and Collider
sources directly and compares check_collisions() with a brute-force reference.

run() condenses the GoogleTest JSON into the unit.json stored next to the
runner results, so the HTML report can show the unit tests per version.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

from .build import BuildError, Checkout, _compiler_id, _run, checkout
from .paths import TESTS_DIR, WORK_DIR, cpm_cache

UNIT_DIR = TESTS_DIR / "unit"


def build(repo: Path, ref: str | None, build_type: str, jobs: int, local_common: Path | None = None,
          rebuild: bool = False) -> tuple[Checkout, Path]:
    """Configure and build tests/unit for the sources of `ref`; returns (checkout, executable)."""
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
    return co, exe


def meta(co: Checkout, exe: Path, build_type: str) -> dict:
    """Commit description for results.save when the unit tests are the first results of a commit."""
    return {"label": co.label, "commit": co.commit, "short": co.short, "subject": co.subject,
            "commit_date": co.commit_date, "dirty": co.dirty, "build_type": build_type,
            "source_dir": str(co.source), "compiler": _compiler_id(exe.parent)}


def _seconds(text: str | None) -> float:
    try:
        return float(str(text).rstrip("s"))
    except ValueError:
        return 0.0


def _status(case: dict) -> str:
    if case.get("failures"):
        return "fail"
    if case.get("result") in ("SKIPPED", "SUPPRESSED") or case.get("status") == "NOTRUN":
        return "skip"
    return "pass"


def summarize(data: dict | None, returncode: int, gtest_filter: str | None) -> dict:
    """GoogleTest JSON -> {summary, suites: [{name, tests: [{name, status, time_ms, message}]}]}.

    A binary that dies before writing its report (crash, abort) is recorded as an error."""
    suites = []
    for suite in (data or {}).get("testsuites", []):
        tests = [{"name": case["name"], "status": _status(case), "time_ms": _seconds(case.get("time")) * 1000.0,
                  "message": "\n".join(f.get("failure", "") for f in case.get("failures", [])) or None}
                 for case in suite.get("testsuite", [])]
        suites.append({"name": suite["name"], "tests": tests})
    statuses = [t["status"] for s in suites for t in s["tests"]]
    error = 1 if data is None or (returncode != 0 and "fail" not in statuses) else 0
    return {
        "filter": gtest_filter,
        "returncode": returncode,
        "time_s": _seconds((data or {}).get("time")),
        "summary": {"total": len(statuses), "pass": statuses.count("pass"), "fail": statuses.count("fail"),
                    "skip": statuses.count("skip"), "error": error},
        "suites": suites,
    }


def run(exe: Path, gtest_filter: str | None = None, verbose: bool = False) -> tuple[int, dict]:
    """Run the GoogleTest binary, print a one-line summary; returns (exit code, summarize() result)."""
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

    data = None
    if report.exists():
        with open(report, encoding="utf-8") as handle:
            data = json.load(handle)
    out = summarize(data, result.returncode, gtest_filter)
    c = out["summary"]
    failed = [f"{s['name']}.{t['name']}" for s in out["suites"] for t in s["tests"] if t["status"] == "fail"]
    print(f"[unit] {c['pass']}/{c['total']} passed" + (f", failed: {', '.join(failed)}" if failed else "")
          + (f" (exit code {result.returncode}, no complete report)" if c["error"] else ""), flush=True)
    return result.returncode, out
