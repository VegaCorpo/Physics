"""Check out, build and locate the module library and the QA runner for a git ref."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass, asdict
from pathlib import Path

from .paths import TESTS_DIR, WORK_DIR, cpm_cache


class BuildError(RuntimeError):
    pass


@dataclass
class Build:
    label: str
    commit: str
    short: str
    subject: str
    commit_date: str
    dirty: bool
    build_type: str
    repo: str
    source_dir: str
    lib: str
    runner: str
    compiler: str

    def to_dict(self) -> dict:
        return asdict(self)


def _git(repo: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(repo), *args], capture_output=True, text=True)
    if result.returncode != 0:
        raise BuildError(f"git {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def _run(cmd: list[str], cwd: Path | None = None, log: Path | None = None) -> None:
    print(f"  $ {' '.join(cmd)}", flush=True)
    with open(log, "a", encoding="utf-8") if log else open(os.devnull, "w") as handle:
        handle.write(f"$ {' '.join(cmd)}\n")
        handle.flush()
        result = subprocess.run(cmd, cwd=cwd, stdout=handle, stderr=subprocess.STDOUT, text=True)
    if result.returncode != 0:
        tail = ""
        if log and log.exists():
            tail = "\n".join(log.read_text(encoding="utf-8").splitlines()[-30:])
        raise BuildError(f"command failed ({result.returncode}): {' '.join(cmd)}\n{tail}")


def describe_ref(repo: Path, ref: str) -> tuple[str, str, str, str]:
    commit = _git(repo, "rev-parse", "--verify", f"{ref}^{{commit}}")
    short = commit[:12]
    subject = _git(repo, "log", "-1", "--format=%s", commit)
    date = _git(repo, "log", "-1", "--format=%cI", commit)
    return commit, short, subject, date


def is_dirty(repo: Path) -> bool:
    status = _git(repo, "status", "--porcelain", "--untracked-files=no", "--", ".", ":!tests")
    return bool(status.strip())


def _compiler_id(build_dir: Path) -> str:
    cache = build_dir / "CMakeCache.txt"
    if not cache.exists():
        return "unknown"
    match = re.search(r"^CMAKE_CXX_COMPILER:\w+=(.*)$", cache.read_text(encoding="utf-8"), re.M)
    if not match:
        return "unknown"
    compiler = match.group(1)
    try:
        version = subprocess.run([compiler, "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    except (OSError, IndexError):
        version = compiler
    return version


def _find_lib(build_dir: Path) -> Path:
    candidates = sorted(build_dir.glob("liborbital_physics*.so")) + sorted(build_dir.glob("*.so"))
    for candidate in candidates:
        if candidate.is_file() and "_deps" not in candidate.parts:
            return candidate
    raise BuildError(f"no shared library found in {build_dir}")


def prepare(repo: Path, ref: str | None, build_type: str, jobs: int, local_common: Path | None = None,
            rebuild: bool = False) -> Build:
    """Build the module at `ref` (or the working tree when ref is None) plus a matching runner.

    Layout: .work/<label>/{src,build,runner}. The source checkout is a detached git
    worktree so the user's checkout is never touched; the working tree itself is used
    in place when ref is None (label suffixed with -dirty when it has changes).
    """
    repo = repo.resolve()
    cache = cpm_cache(repo)

    if ref is None:
        commit, short, subject, date = describe_ref(repo, "HEAD")
        dirty = is_dirty(repo)
        label = f"{short}-dirty" if dirty else short
        source = repo
    else:
        commit, short, subject, date = describe_ref(repo, ref)
        dirty = False
        label = short
        source = WORK_DIR / label / "src"
        if rebuild and source.exists():
            subprocess.run(["git", "-C", str(repo), "worktree", "remove", "--force", str(source)],
                           capture_output=True)
            shutil.rmtree(source, ignore_errors=True)
        if not source.exists():
            source.parent.mkdir(parents=True, exist_ok=True)
            _git(repo, "worktree", "prune")
            _git(repo, "worktree", "add", "--detach", str(source), commit)

    work = WORK_DIR / label
    work.mkdir(parents=True, exist_ok=True)
    log = work / f"build-{build_type}.log"
    if rebuild and log.exists():
        log.unlink()

    common_args = [f"-DCPM_Common_SOURCE={local_common.resolve()}"] if local_common else []

    print(f"[build] {label} ({build_type}) {subject}", flush=True)
    lib_build = work / f"build-{build_type}"
    if rebuild:
        shutil.rmtree(lib_build, ignore_errors=True)
    _run(["cmake", "-S", str(source), "-B", str(lib_build), f"-DCMAKE_BUILD_TYPE={build_type}",
          f"-DCPM_SOURCE_CACHE={cache}", *common_args], log=log)
    _run(["cmake", "--build", str(lib_build), "-j", str(jobs)], log=log)
    lib = _find_lib(lib_build)

    runner_build = work / f"runner-{build_type}"
    _run(["cmake", "-S", str(TESTS_DIR), "-B", str(runner_build), "-DCMAKE_BUILD_TYPE=Release",
          f"-DPHYSICS_SOURCE_DIR={source}", f"-DCPM_SOURCE_CACHE={cache}", *common_args], log=log)
    _run(["cmake", "--build", str(runner_build), "-j", str(jobs)], log=log)
    runner = runner_build / "physics_qa_runner"
    if not runner.exists():
        raise BuildError(f"runner not built: {runner}")

    return Build(label=label, commit=commit, short=short, subject=subject, commit_date=date, dirty=dirty,
                 build_type=build_type, repo=str(repo), source_dir=str(source), lib=str(lib),
                 runner=str(runner), compiler=_compiler_id(lib_build))


def from_prebuilt(repo: Path, lib: Path, source_dir: Path, build_type: str, jobs: int, label: str | None) -> Build:
    """Use an already built library; only the runner is built (against source_dir's Common)."""
    repo = repo.resolve()
    commit, short, subject, date = describe_ref(repo, "HEAD")
    label = label or f"{short}-prebuilt"
    work = WORK_DIR / label
    work.mkdir(parents=True, exist_ok=True)
    runner_build = work / f"runner-{build_type}"
    log = work / f"build-{build_type}.log"
    _run(["cmake", "-S", str(TESTS_DIR), "-B", str(runner_build), "-DCMAKE_BUILD_TYPE=Release",
          f"-DPHYSICS_SOURCE_DIR={source_dir.resolve()}", f"-DCPM_SOURCE_CACHE={cpm_cache(repo)}"], log=log)
    _run(["cmake", "--build", str(runner_build), "-j", str(jobs)], log=log)
    return Build(label=label, commit=commit, short=short, subject=subject, commit_date=date, dirty=True,
                 build_type=build_type, repo=str(repo), source_dir=str(source_dir), lib=str(lib.resolve()),
                 runner=str(runner_build / "physics_qa_runner"), compiler="prebuilt")
