"""Thin wrapper around the physics_qa_runner executable."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path


class RunnerError(RuntimeError):
    pass


class Runner:
    def __init__(self, executable: Path, lib: Path, work: Path, G: float):
        self.executable = Path(executable)
        self.lib = Path(lib)
        self.work = Path(work)
        self.work.mkdir(parents=True, exist_ok=True)
        self.G = G
        self._counter = 0

    def _out(self, tag: str) -> Path:
        self._counter += 1
        return self.work / f"{self._counter:04d}-{tag}.json"

    def _exec(self, args: list[str], out: Path) -> dict:
        cmd = [str(self.executable), *args, "--out", str(out), "--G", repr(self.G)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise RunnerError(f"runner exited with {result.returncode}: {result.stderr.strip()}\n  cmd: {' '.join(cmd)}")
        with open(out, encoding="utf-8") as handle:
            return json.load(handle)

    def simulate(self, scene: Path, dt: float, frames: int, sample_every: int | None = None,
                 snapshots: bool = True, reverse_at: int | None = None, potential: bool = True,
                 tag: str = "sim") -> dict:
        args = ["simulate", "--lib", str(self.lib), "--scene", str(scene), "--dt", repr(dt), "--frames", str(frames)]
        if sample_every:
            args += ["--sample-every", str(sample_every)]
        if snapshots:
            args.append("--snapshots")
        if reverse_at is not None:
            args += ["--reverse-at", str(reverse_at)]
        if not potential:
            args.append("--no-potential")
        return self._exec(args, self._out(tag))

    def reference(self, scene: Path, dt: float, frames: int, sample_every: int | None = None,
                  substeps: int = 16, snapshots: bool = True, softening: float = 0.0, tag: str = "ref") -> dict:
        args = ["reference", "--scene", str(scene), "--dt", repr(dt), "--frames", str(frames),
                "--substeps", str(substeps), "--softening", repr(softening)]
        if sample_every:
            args += ["--sample-every", str(sample_every)]
        if snapshots:
            args.append("--snapshots")
        return self._exec(args, self._out(tag))

    def bench(self, scene: Path, dt: float, frames: int, warmup: int, repeats: int, tag: str = "bench") -> dict:
        args = ["bench", "--lib", str(self.lib), "--scene", str(scene), "--dt", repr(dt), "--frames", str(frames),
                "--warmup", str(warmup), "--repeats", str(repeats)]
        return self._exec(args, self._out(tag))
