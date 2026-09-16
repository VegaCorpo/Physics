# Physics QA

Black-box test, accuracy and benchmark tooling for the physics module. It never
links the module: a small C++ runner `dlopen()`s the built `liborbital_physics.so`
and drives it through `common::IPhysicsEngine` exactly like Core does
(`syncIn`, `update(dt)`, `syncOut` each frame). Any commit since the interface
stabilised (b2a4058 onward) and any other module implementing the interface can
therefore be measured and compared.

Nothing here is wired into the module's own CMake project: this directory has its
own `CMakeLists.txt`, its own dependency lock (`nlohmann_json`) and needs only
CMake, a C++23 compiler and Python 3 (standard library only).

## Quick start

```sh
cd Physics/tests
./physics-qa run                       # working tree: build, checks, benchmarks, store results
./physics-qa run --ref 6fbfca1         # any git ref, built in a detached worktree under .work/
./physics-qa run --ref main --build-type Debug
./physics-qa test --only kepler_circular convergence_order
./physics-qa bench --bodies 1000 10000
./physics-qa list                      # stored result sets
./physics-qa report --out report.html  # every stored result, chronological
./physics-qa compare 6fbfca1 a54a3ef   # exactly two result sets, exit 2 on a benchmark regression
./physics-qa clean                     # remove worktrees and builds under .work/
```

`run` exits non-zero when a required check fails. `--lib path/to/liborbital_physics.so`
tests an already built library; `--local-common ../../Common` builds against the
Common checkout instead of the tag pinned in `package-lock.cmake`.

The HTML report is a single self-contained file with a navigation bar and five
pages: Overview (how to read it, at-a-glance verdict, versions), Speed, Checks,
Error charts and Method & glossary. It is written for non-specialists: one
plain-language question per check, every error given as a percentage plus its
scientific value and, where possible, a distance, a bar showing how much of the
allowed error was used, and charts with a reading guide, a shaded "too much error"
zone, the worst point of the newest version called out, and a verdict line.

Results land in `results/<commit>/<BuildType>/{meta,correctness,benchmark}.json`
and are meant to be committed, so `report` can chart the whole history. The label
gets a `-dirty` suffix when the working tree has uncommitted changes.

## What is checked

All tolerances live in `config/thresholds.json` (override any subset with
`--thresholds my.json`). Errors are relative: positions are divided by the scene
scale (semi-major axis, or RMS distance to the barycenter), velocities by the RMS
speed, conserved quantities by their initial magnitude or by the sum of the
per-body magnitudes when the total can be zero.

| Check | Category | Ground truth | What a failure means |
|---|---|---|---|
| `kepler_circular`, `kepler_eccentric` | correctness | analytic Kepler orbit (barycentric two-body) | wrong gravity law, wrong G/units, wrong mass decoding |
| `planetary_reference` | correctness | built-in RK4 long double integrator, 16 substeps/frame | wrong N-body force accumulation |
| `conservation` | correctness | energy, momentum, angular momentum, barycenter of a 200-body cluster | non-symmetric forces, self-interaction, lost bodies |
| `energy_bounded` | integrator | energy error over 10 orbits must not grow | integrator lost its symplectic character |
| `reversibility` | integrator | forward N frames, negate velocities, back N frames | integrator not time-reversible (e.g. forces evaluated at the wrong time) |
| `convergence_order` | integrator | error vs dt against the analytic orbit, expected slope 2 | integrator order regressed (e.g. Euler instead of Verlet) |
| `determinism` | robustness | two processes, bitwise equality | non-deterministic reduction order |
| `permutation_invariance`, `translation_invariance` | robustness | same scene reordered / shifted | order- or origin-dependent code paths |
| `padding_sizes` | robustness | body counts around SIMD block boundaries vs reference | tail/padding handling bugs |
| `single_body_inertia`, `empty_world`, `coincident_bodies`, `large_scene_smoke` | robustness | straight line, no crash, no NaN | edge-case handling |
| `massless_body_moves` | robustness (not required) | analytic circular orbit of a massless probe | massless bodies are currently skipped by the integrator |

The reference integrator is `runner/Reference.hpp`: classical RK4 in 80-bit long
double on the unsoftened Newtonian problem. With 16 substeps per 7200 s frame its
error is orders of magnitude below the module's double precision Verlet error.

## Benchmarks

`config/benchmark.json` sets the body counts (10, 100, 1000, 10000, 50000 by
default), the number of warm-up and timed frames, repeats and the regression
threshold. Every repeat uses a fresh engine. The runner times `update()` alone and
the whole frame contract separately; the report uses the median of `update()`.
Deltas below `min_ms_for_verdict` are shown but never judged.

For stable numbers, run on an idle machine with the `performance` CPU governor;
the report shows a warning otherwise. The benchmark runs with all hardware
threads, as the module's TBB parallel loops expect.

## Layout

```
tests/
  physics-qa            entry point (python3 -m qa)
  CMakeLists.txt        standalone runner project (PHYSICS_SOURCE_DIR selects the Common version)
  runner/               C++: Plugin.hpp (dlopen), Scene.hpp, Invariants.hpp, Reference.hpp, main.cpp
  qa/                   Python: build.py, checks.py, bench.py, scenes.py, kepler.py, report.py, svg.py, cli.py
  config/               thresholds.json, benchmark.json
  results/              committed JSON results per commit and build type
  .work/                worktrees, builds, generated scenes, raw runner output (ignored)
```

## Runner on its own

```sh
cmake -S . -B .work/runner -DPHYSICS_SOURCE_DIR=.. && cmake --build .work/runner
.work/runner/physics_qa_runner simulate --lib ../build/liborbital_physics.so \
    --scene ../../scenes/simulation_1.json --dt 7200 --frames 4383 --sample-every 100 --snapshots --out sim.json
.work/runner/physics_qa_runner reference --scene ../../scenes/simulation_1.json --dt 7200 --frames 4383 --out ref.json
.work/runner/physics_qa_runner bench --lib ../build/liborbital_physics.so --scene ../../scenes/benchmark_1000.json \
    --dt 7200 --frames 30 --warmup 3 --repeats 3 --out bench.json
```

Note: the runner never `dlclose()`s the library. The module leaves TBB worker
threads alive after the engine is destroyed, and unmapping the code they run
crashes the process.
