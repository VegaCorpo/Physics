"""Command line interface: python -m qa <command> or ./physics-qa <command>."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

from . import __version__, bench, build, checks, config, overlaps, report, results, unit
from .paths import DEFAULT_REPO, RESULTS_DIR, WORK_DIR
from .runner import Runner


def _add_build_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO,
                        help="module repository to test (default: the parent of tests/)")
    parser.add_argument("--ref", default=None,
                        help="git ref to build in a detached worktree; default: the working tree as is")
    parser.add_argument("--build-type", default="Release", choices=["Release", "Debug"])
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    parser.add_argument("--local-common", type=Path, default=None,
                        help="use this Common checkout instead of the tag pinned by package-lock.cmake")
    parser.add_argument("--rebuild", action="store_true", help="wipe and rebuild the module and runner")
    parser.add_argument("--lib", type=Path, default=None,
                        help="use an already built shared library (skips the module build)")
    parser.add_argument("--label", default=None, help="results label override (default: short commit hash)")


def _prepare(args) -> build.Build:
    if args.lib:
        return build.from_prebuilt(args.repo, args.lib, args.repo, args.build_type, args.jobs, args.label)
    b = build.prepare(args.repo, args.ref, args.build_type, args.jobs, args.local_common, args.rebuild)
    if args.label:
        b.label = args.label
    return b


def _runner(b: build.Build, G: float, tag: str) -> Runner:
    work = WORK_DIR / b.label / f"runs-{b.build_type}-{tag}"
    shutil.rmtree(work, ignore_errors=True)
    return Runner(Path(b.runner), Path(b.lib), work, G)


def _meta(b: build.Build, extra: dict) -> dict:
    return {**b.to_dict(), **results.machine_meta(), "qa_version": __version__, **extra}


def _unit_meta(co, exe, build_type: str, label: str, root: Path) -> dict:
    """Commit and machine fields for a unit-only save; keys already stored by `run` are left untouched."""
    existing = results.load_one(results.result_dir(label, build_type, root)).get("meta") or {}
    defaults = {**unit.meta(co, exe, build_type), **results.machine_meta(), "qa_version": __version__, "label": label}
    return {k: v for k, v in defaults.items() if k not in existing}


def cmd_run(args) -> int:
    b = _prepare(args)
    correctness = benchmark = unit_results = None
    engine = None
    if not args.no_unit:
        _, exe = unit.build(args.repo, args.ref, args.build_type, args.jobs, args.local_common, args.rebuild)
        _, unit_results = unit.run(exe)
    if not args.no_tests:
        thresholds = config.load_thresholds(args.thresholds)
        ctx = checks.Context(_runner(b, thresholds["G"], "tests"), WORK_DIR / b.label / "scenes", thresholds["dt"],
                             thresholds["G"], thresholds["reference"]["substeps"],
                             thresholds["reference"]["softening_km"])
        correctness = checks.run_all(ctx, thresholds, args.only)
        print(f"[tests] {correctness['summary']}")
    if not args.no_bench:
        plan = config.load_benchmark(args.benchmark)
        runner = _runner(b, config.load_thresholds(args.thresholds)["G"], "bench")
        benchmark = bench.run(runner, plan, WORK_DIR / "scenes", args.bodies)
        engine = benchmark.get("engine")
    target = results.save(b.label, b.build_type, _meta(b, {"engine": engine}), correctness, benchmark, args.results,
                          unit=unit_results)
    print(f"[saved] {target}")
    if correctness and (correctness["summary"]["fail"] or correctness["summary"]["error"]):
        return 1
    if unit_results and (unit_results["summary"]["fail"] or unit_results["summary"]["error"]):
        return 1
    return 0


def cmd_report(args) -> int:
    entries = results.load_all(args.results, args.build_type)
    if args.labels:
        entries = results.select(entries, args.labels)
    if not entries:
        print("no stored results", file=sys.stderr)
        return 1
    plan = config.load_benchmark(args.benchmark)
    html = report.render(entries, plan)
    out = Path(args.out)
    out.write_text(html, encoding="utf-8")
    print(f"[report] {out} ({len(entries)} result set(s): {', '.join(e.label for e in entries)})")
    if args.strict and len(entries) >= 2:
        base, cand = entries[-2], entries[-1]
        if base.benchmark and cand.benchmark:
            rows = bench.compare(base.benchmark, cand.benchmark, plan["regression_threshold_pct"],
                                 plan["improvement_threshold_pct"], plan.get("min_ms_for_verdict", 0.0))
            if any(r["verdict"] == "regression" for r in rows):
                print("[report] benchmark regression detected", file=sys.stderr)
                return 2
        if cand.correctness and (cand.correctness["summary"]["fail"] or cand.correctness["summary"]["error"]):
            return 1
        if cand.unit and (cand.unit["summary"]["fail"] or cand.unit["summary"]["error"]):
            return 1
    return 0


def cmd_list(args) -> int:
    for e in results.load_all(args.results):
        c = e.correctness["summary"] if e.correctness else None
        b = e.benchmark["results"] if e.benchmark else []
        summary = f"tests {c['pass']}P/{c['fail']}F/{c['warn']}W/{c['error']}E" if c else "no tests"
        u = e.unit["summary"] if e.unit else None
        summary += f" unit {u['pass']}/{u['total']}" if u else " no unit"
        biggest = max(b, key=lambda r: r["bodies"]) if b else None
        bench_s = f"{biggest['bodies']} bodies {biggest['update_ms']['median']:.3f} ms" if biggest else "no bench"
        print(f"{e.label:24} {e.build_type:8} {e.commit_date:25} {summary:42} {bench_s}   {e.meta.get('subject', '')}")
    return 0


def cmd_build(args) -> int:
    b = _prepare(args)
    print(f"[built] lib={b.lib}\n        runner={b.runner}")
    return 0


def cmd_unit(args) -> int:
    co, exe = unit.build(args.repo, args.ref, args.build_type, args.jobs, args.local_common, args.rebuild)
    code, unit_results = unit.run(exe, args.filter, args.verbose)
    if args.filter or args.no_save:
        return code  # a filtered run would overwrite the full results with a subset
    label = args.label or co.label
    target = results.save(label, args.build_type, _unit_meta(co, exe, args.build_type, label, args.results), None, None,
                          args.results, unit=unit_results)
    print(f"[saved] {target / 'unit.json'}")
    return code


def cmd_overlaps(args) -> int:
    total = 0
    for scene in args.scenes:
        text, count = overlaps.describe(scene, args.limit)
        print(text)
        total += count
    if args.expect_some and total == 0:
        print("[overlaps] no pair in contact: the module cannot report any collision on these scenes",
              file=sys.stderr)
        return 1
    return 0


def cmd_clean(args) -> int:
    for path in WORK_DIR.glob("*/src"):
        os.system(f"git -C '{args.repo}' worktree remove --force '{path}' >/dev/null 2>&1")
    shutil.rmtree(WORK_DIR, ignore_errors=True)
    os.system(f"git -C '{args.repo}' worktree prune")
    print(f"[clean] removed {WORK_DIR}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="physics-qa", description="Black-box QA for the physics module")
    parser.add_argument("--results", type=Path, default=RESULTS_DIR, help="results directory")
    sub = parser.add_subparsers(dest="command", required=True)

    p_run = sub.add_parser("run", help="build a ref, run the correctness suite and the benchmarks, store results")
    _add_build_args(p_run)
    p_run.add_argument("--no-tests", action="store_true")
    p_run.add_argument("--no-bench", action="store_true")
    p_run.add_argument("--no-unit", action="store_true", help="skip the white-box unit tests (tests/unit)")
    p_run.add_argument("--only", nargs="*", help="run only these checks")
    p_run.add_argument("--bodies", nargs="*", type=int, help="benchmark only these body counts")
    p_run.add_argument("--thresholds", type=Path, default=None, help="JSON overriding config/thresholds.json")
    p_run.add_argument("--benchmark", type=Path, default=None, help="JSON overriding config/benchmark.json")
    p_run.set_defaults(func=cmd_run)

    p_test = sub.add_parser("test", help="same as run --no-bench")
    _add_build_args(p_test)
    p_test.add_argument("--only", nargs="*")
    p_test.add_argument("--thresholds", type=Path, default=None)
    p_test.add_argument("--no-unit", action="store_true", help="skip the white-box unit tests (tests/unit)")
    p_test.set_defaults(func=cmd_run, no_bench=True, no_tests=False, benchmark=None, bodies=None)

    p_bench = sub.add_parser("bench", help="same as run --no-tests")
    _add_build_args(p_bench)
    p_bench.add_argument("--bodies", nargs="*", type=int)
    p_bench.add_argument("--benchmark", type=Path, default=None)
    p_bench.add_argument("--thresholds", type=Path, default=None)
    p_bench.set_defaults(func=cmd_run, no_tests=True, no_bench=False, no_unit=True, only=None)

    p_build = sub.add_parser("build", help="only build the module and the runner")
    _add_build_args(p_build)
    p_build.set_defaults(func=cmd_build)

    p_unit = sub.add_parser("unit", help="build and run the white-box unit tests (tests/unit) on a ref")
    p_unit.add_argument("--repo", type=Path, default=DEFAULT_REPO,
                        help="module repository to test (default: the parent of tests/)")
    p_unit.add_argument("--ref", default=None,
                        help="git ref to build in a detached worktree; default: the working tree as is")
    p_unit.add_argument("--build-type", default="Release", choices=["Release", "Debug"])
    p_unit.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    p_unit.add_argument("--local-common", type=Path, default=None,
                        help="use this Common checkout instead of the tag pinned by package-lock.cmake")
    p_unit.add_argument("--rebuild", action="store_true", help="wipe and rebuild the unit tests")
    p_unit.add_argument("--filter", default=None, help="GoogleTest filter, e.g. 'Collider.*'")
    p_unit.add_argument("--verbose", action="store_true", help="print every test, not only failures")
    p_unit.add_argument("--label", default=None, help="results label override (default: short commit hash)")
    p_unit.add_argument("--no-save", action="store_true",
                        help="do not store unit.json (always skipped with --filter)")
    p_unit.set_defaults(func=cmd_unit)

    p_ov = sub.add_parser("overlaps", help="count the pairs of bodies in contact at t=0 in scene files")
    p_ov.add_argument("scenes", nargs="+", type=Path, help="scene JSON files (engine layout)")
    p_ov.add_argument("--limit", type=int, default=20, help="pairs to list per scene")
    p_ov.add_argument("--expect-some", action="store_true", help="exit 1 when no scene has any contact")
    p_ov.set_defaults(func=cmd_overlaps)

    p_report = sub.add_parser("report", help="render an HTML report from stored results")
    p_report.add_argument("labels", nargs="*", help="result labels or commit prefixes (default: all)")
    p_report.add_argument("--build-type", default="Release")
    p_report.add_argument("--out", default="physics-qa-report.html")
    p_report.add_argument("--benchmark", type=Path, default=None)
    p_report.add_argument("--strict", action="store_true",
                          help="exit 2 on a benchmark regression, 1 on failing checks in the last result")
    p_report.set_defaults(func=cmd_report)

    p_cmp = sub.add_parser("compare", help="render a report for exactly two result sets (baseline, candidate)")
    p_cmp.add_argument("baseline")
    p_cmp.add_argument("candidate")
    p_cmp.add_argument("--build-type", default="Release")
    p_cmp.add_argument("--out", default="physics-qa-compare.html")
    p_cmp.add_argument("--benchmark", type=Path, default=None)
    p_cmp.set_defaults(func=cmd_report, strict=True)

    p_list = sub.add_parser("list", help="list stored results")
    p_list.set_defaults(func=cmd_list)

    p_clean = sub.add_parser("clean", help="remove worktrees and build directories under tests/.work")
    p_clean.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    p_clean.set_defaults(func=cmd_clean)

    args = parser.parse_args(argv)
    if args.command == "compare":
        args.labels = [args.baseline, args.candidate]
    return args.func(args)
