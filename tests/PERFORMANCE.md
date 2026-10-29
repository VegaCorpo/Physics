# Performance comparisons

How to measure, compare and track the cost of the physics module: integrators, time step, scene size and,
over time, every commit. It relies only on the QA tooling described in [README.md](README.md)
(`physics-qa` and the `physics_qa_runner` executable).

A speed number alone does not choose an integrator. PEFRL evaluates the forces four times per step and
Verlet once; the module compensates by giving PEFRL steps four times longer (28800 s against 7200 s), so
both cost about the same per simulated second while PEFRL stays orders of magnitude more accurate. Every
comparison here therefore pairs a **cost** measurement with an **accuracy** measurement taken on the
same build.

## What is compared

| Axis | Values | How it is selected |
|---|---|---|
| Integrator | `verlet` (order 2, 1 force evaluation, 7200 s steps), `pefrl` (order 4, 4 evaluations, 28800 s steps) | `--thresholds config/integrators/<name>.json`, or `--integrator <name>` on the runner |
| Update length `dt` | 7200 s, 86400 s, 31557600 s (1 year) | `"dt"` in a `--benchmark` override, or `--dt` on the runner |
| Body count | 10, 100, 1000, 10000, 50000 synthetic clusters; `scenes/*.json` | `"body_counts"` / `--bodies`, or `--scene` on the runner |
| Version | any commit or the working tree | `--ref`, `--label` |
| Build type | `Release` (reference), `Debug` | `--build-type` |

The integrator must always be given explicitly: `common::DEFAULT_INTEGRATOR` is `PEFRL` in Common, so a
run without `--thresholds config/integrators/...` measures PEFRL.

## How Core drives the module

Core advances the simulation with a fixed step, following
[Fix Your Timestep](https://gafferongames.com/post/fix_your_timestep/):

1. every 16 ms tick, the elapsed real time multiplied by the time scale is added to a time debt;
2. Core asks the module for `stepDuration()` and calls `update(k × stepDuration())`, where `k` is the
   number of whole steps owed, capped by what fits in a 12 ms budget (cost of the previous tick);
3. debt beyond the budget is dropped instead of piling up, so the simulation slows down instead of
   stuttering when the machine cannot keep up;
4. the renderer interpolates between the last two published states, so the motion stays smooth even
   when the module publishes less often than the screen refreshes.

## Metrics

| Metric | Meaning | Source |
|---|---|---|
| `update_ms` (median) | wall time of one `update(dt)` | `bench` |
| Cost per step | `update_ms` / number of steps in the update | `bench` |
| Real-time speed | fastest time scale Core can sustain: steps fitting in the 12 ms budget × step duration / 16 ms | derived from the cost per step |
| Position error | relative error against an analytic orbit or the RK4 reference | `test` checks |
| Energy growth | energy error after 10 orbits / after the first one; ≈ 1 means bounded | `energy_bounded` |

When the requested time scale exceeds the real-time speed, Core runs at the real-time speed instead.

## Protocol

1. Build in `Release`, on an idle machine, with the `performance` CPU governor
   (`sudo cpupower frequency-set -g performance`); the report warns when the governor is different.
2. Only compare results measured on the same machine: `meta.json` stores the CPU, governor and compiler.
3. Run each integrator under its own label, `<commit>-<integrator>`, so the two result sets do not
   overwrite each other in `results/<label>/<BuildType>/`.
4. Commit the result sets with the change they measure, so `report` can chart the history.
5. Until the Common version declaring `IPhysicsEngine::stepDuration()` is tagged and pinned, add
   `--local-common ../../Common` to every `physics-qa` command that builds the module.

## Commands

All commands run from `Physics/tests`.

### Accuracy and speed versus body count, per integrator

```sh
COMMIT=$(git rev-parse --short=12 HEAD)
./physics-qa run --thresholds config/integrators/verlet.json --label "$COMMIT-verlet"
./physics-qa run --thresholds config/integrators/pefrl.json  --label "$COMMIT-pefrl"
./physics-qa report "$COMMIT-verlet" "$COMMIT-pefrl" --out integrators.html
```

`run` stores `correctness.json` (accuracy) and `benchmark.json` (`update_ms` per body count, `dt` from
`config/benchmark.json`) under each label. The report shows both side by side; `compare` does the same for
exactly two labels and exits 2 on a speed regression.

### Speed versus `dt`

`config/benchmark.json` benchmarks a single `dt`. A `--benchmark` override changes it without touching the
shared plan:

```sh
echo '{"dt": 31557600, "body_counts": [10, 100, 1000], "frames": 2, "warmup": 1, "repeats": 1}' > /tmp/year.json
./physics-qa bench --benchmark /tmp/year.json --thresholds config/integrators/pefrl.json --label "$COMMIT-pefrl-year"
```

For a scene of the engine rather than a synthetic cluster, call the runner directly; it prints nothing
and writes the raw timings to `--out`:

```sh
RUNNER=.work/$COMMIT/runner-Release/physics_qa_runner
LIB=.work/$COMMIT/build-Release/liborbital_physics.so
for integrator in verlet pefrl; do
    for dt in 7200 86400 31557600; do
        $RUNNER bench --lib $LIB --scene ../../scenes/solar_systems.json --dt $dt \
            --frames 10 --warmup 1 --repeats 3 --integrator $integrator --out "$integrator-$dt.json"
    done
done
```

Both paths exist after any `physics-qa run`, `test` or `build` on that commit (with a `-dirty` suffix for
an uncommitted working tree).

### History of one integrator

```sh
./physics-qa run --ref <older-commit> --thresholds config/integrators/verlet.json --label "<older-commit>-verlet"
./physics-qa report --out history.html
```

## Baseline

Measured on 2026-10-22 on top of commit `cbcdd30`, AMD Ryzen 7 PRO 8840HS (16 threads), GCC 16.2.1,
Release, Barnes–Hut gravity. "Before" is `cbcdd30` (every integrator limited to 7200 s sub-steps),
"after" adds the per-integrator step. The governor was `powersave`: absolute timings are pessimistic and
noisy, ratios are reliable. Below 0.05 ms the timings are dominated by scheduling noise.

### Cost: median `update_ms`

| Scene | Bodies | `dt` | Verlet | PEFRL before | PEFRL after |
|---|---:|---|---:|---:|---:|
| `solar_systems.json` | 10 | 2 h | 0.003 | 0.008 | 0.008 |
| `solar_systems.json` | 10 | 1 day | 0.060 | 0.105 | 0.029 |
| `solar_systems.json` | 10 | 1 year | 11.1 | 45.2 | 11.4 |
| `benchmark_1000.json` | 1000 | 2 h | 0.575 | 2.00 | 2.05 |
| `benchmark_1000.json` | 1000 | 1 day | 5.80 | 23.9 | 6.20 |
| `benchmark_1000.json` | 1000 | 1 year | 3618 | 16570 | 3742 |
| `benchmark_10000.json` | 10000 | 2 h | 12.5 | 44.0 | 42.4 |
| `benchmark_10000.json` | 10000 | 1 day | 164 | 729 | 171 |

An update of 2 h stays a single PEFRL step of 2 h, hence the unchanged cost on that line.

### Real-time speed with Core's 12 ms budget

| Bodies | Verlet | PEFRL |
|---:|---|---|
| 10 | ≈ 67 years/s | ≈ 66 years/s |
| 1000 | ≈ 73 days/s | ≈ 62 days/s |
| 10000 | ≈ 5.2 days/s | ≈ 5.8 days/s, one 57 ms step per tick |

With 10000 bodies a single PEFRL step exceeds the tick: the module then publishes about 17 times per
second and the renderer's interpolation keeps the motion smooth.

### Accuracy: same build

| Check | What is measured | Verlet | PEFRL before | PEFRL after |
|---|---|---:|---:|---:|
| `kepler_eccentric` | position error, e = 0.6 orbit, 2 h updates | 2.8e-4 | 3.4e-10 | 3.4e-10 |
| `planetary_reference` | position error against RK4, N-body, 2 h updates | 7.8e-6 | 2.4e-11 | 2.4e-11 |
| `timestep_ui_scales` | position error over 10 years, `dt` from 1 h to 10 years | 4.4e-5 | 5.6e-12 | 1.2e-9 |
| `timestep_satellite` | Moon position error after 20 updates of 1 year | 2.2e-1 | 4.3e-6 | 1.1e-3 |
| `energy_bounded` | energy error growth over 10 orbits | 1.00 | 0.79 | 0.79 |

For the same cost per simulated second, PEFRL with 28800 s steps remains about 200 times more accurate
than Verlet on the Moon and keeps the energy error bounded.
