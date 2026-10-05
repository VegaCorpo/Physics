"""Self-contained HTML report (inline CSS + SVG, no external assets).

Written for readers who are not physicists: every check has a plain-language
name and purpose, every number is translated into a percentage or a distance,
and each chart carries a one-sentence explanation of what it shows.
"""

from __future__ import annotations

import math
import re
from datetime import datetime, timezone
from html import escape

from . import bench, svg
from .results import Entry

STATUS = {
    "pass": ("✓", "PASS", "good"),
    "fail": ("✗", "FAIL", "critical"),
    "warn": ("!", "WARN", "warning"),
    "error": ("⚠", "ERROR", "serious"),
    "skip": ("–", "SKIP", "muted"),
}

CATEGORIES = {
    "correctness": ("Is the physics right?",
                    "The simulated motion is compared with exact textbook solutions, with a high-precision "
                    "reference simulation, and with the laws of conservation. These checks catch a wrong "
                    "gravity formula, wrong units or a wrong mass decoding."),
    "integrator": ("Does the time stepping behave as designed?",
                   "The module advances time with a velocity Verlet integrator. Such an integrator has three "
                   "signature properties that are verified here: its energy error stays bounded instead of "
                   "drifting, it can run backwards to the exact starting point, and halving the time step "
                   "divides the error by four."),
    "robustness": ("Does it behave in unusual situations?",
                   "Edge cases and invariances: results must not depend on the order of the bodies, on where "
                   "the scene is located, or on how many bodies there are; degenerate scenes must not crash "
                   "or produce NaN."),
}

# Human name, question answered, how it is measured (in words).
CHECK_INFO = {
    "kepler_circular": ("Earth-like circular orbit",
                        "Does a planet on a circular orbit around a star follow the exact orbit for one full year?",
                        "The simulated positions are compared, at 40 points along the orbit, with the exact "
                        "Kepler solution. The reported value is the largest gap, as a fraction of the orbit radius."),
    "kepler_eccentric": ("Elongated (eccentric) orbit",
                         "Same as above with a strongly elongated orbit, where the speed varies a lot near the star.",
                         "Largest gap between simulated and exact position over one orbit, as a fraction of the "
                         "orbit's semi-major axis. Harder than the circular case: expect a larger error."),
    "planetary_reference": ("Star with eleven planets",
                            "Do many bodies interacting at once move like a high-precision reference simulation?",
                            "The same scene is run with an independent RK4 integrator in extended precision, "
                            "sub-stepped 16 times per frame. Largest position gap as a fraction of the system size."),
    "conservation": ("Conservation laws in a 200-body cluster",
                     "Are energy, momentum, angular momentum and the barycenter conserved?",
                     "Each quantity is recomputed at 20 points in time. The value is the largest relative change "
                     "since the start. Momentum and barycenter should be conserved almost exactly; energy only "
                     "approximately, because of the time step."),
    "energy_bounded": ("Energy error stays bounded",
                       "Over ten orbits, does the energy error oscillate instead of slowly drifting away?",
                       "Largest energy error during the last orbit divided by the largest during the first orbit. "
                       "A value near 1 means no drift; a growing value means the integrator lost its symplectic "
                       "property."),
    "reversibility": ("Running backwards returns to the start",
                      "After N steps forward, reversing all velocities and running N steps back, are the bodies "
                      "where they started?",
                      "Largest distance to the starting position, as a fraction of the cluster size. Verlet is "
                      "time-reversible, so this should be at round-off level."),
    "convergence_order": ("Halving the time step divides the error by four",
                          "Is the integrator really second order?",
                          "The same orbit is integrated with several time steps; the end error should scale with "
                          "the square of the step. The value is how far the measured order is from 2."),
    "determinism": ("Same input, same output, bit for bit",
                    "Do two separate runs of the same scene give identical numbers?",
                    "Counts the position and velocity values that differ between two processes."),
    "permutation_invariance": ("Order of the bodies does not matter",
                               "Does shuffling the list of bodies change the outcome?",
                               "Largest difference between the two runs, matched body by body, as a fraction of "
                               "the cluster size."),
    "translation_invariance": ("Location of the scene does not matter",
                               "Does moving the whole scene by millions of km change the relative motion?",
                               "Largest difference after undoing the shift, as a fraction of the cluster size."),
    "padding_sizes": ("Any number of bodies works",
                      "The module processes bodies in SIMD batches. Are odd counts (1, 3, 17, 65…) handled correctly?",
                      "For each size, 20 frames are compared with the reference integrator. The value is the "
                      "worst case over all sizes."),
    "single_body_inertia": ("A lone body moves in a straight line",
                            "With nothing to attract it, does a body keep its velocity exactly?",
                            "Distance to the expected position, as a fraction of the distance travelled."),
    "empty_world": ("Empty scene does not crash", "Can the module be started and stepped with zero bodies?",
                    "Passes if the runner completes and returns zero bodies."),
    "coincident_bodies": ("Two bodies at the same spot", "Do overlapping bodies produce NaN or infinite values?",
                          "Counts non-finite values after 10 frames."),
    "massless_body_moves": ("Massless bodies are still pulled by gravity",
                            "Does a test particle with zero mass orbit the star like any other body?",
                            "Distance to the exact circular orbit as a fraction of its radius. Known limitation: "
                            "the current integrator skips bodies with zero mass, so this check is informational."),
    "large_scene_smoke": ("10 000 bodies run cleanly", "Does a large scene run without numeric blow-up?",
                          "Counts non-finite values after 2 frames."),
}

# GoogleTest suites of tests/unit: question answered, what the tests do (in words).
UNIT_SUITES = {
    "Epsilon": ("Is the softening length chosen correctly?",
                "The module keeps the epsilon given by the engine, or derives it from the smallest body radius when "
                "the engine sends 0, falling back to a fixed default for point masses. These tests check the value "
                "picked in every case: explicit, derived, zero radii, empty scene, padding slots, re-sync."),
    "Softening": ("Does gravity use the softening length as intended?",
                  "Softening removes the infinite force when two bodies get very close. These tests check that the "
                  "force follows the Plummer formula, stays exact once bodies are further apart than their radii, "
                  "and never produces NaN for coincident bodies."),
    "Collider": ("Are all touching bodies detected, and only those?",
                 "The collision pairs found through the octree are compared with a brute-force check of every pair, "
                 "on hand-made cases (touching, separated, across octree cells) and on random clusters."),
    "Octree": ("Is the spatial tree built correctly?",
               "The octree splits space so that collisions only test nearby bodies. These tests check that every "
               "body ends in exactly one leaf that contains it, and that rebuilding the tree starts from scratch."),
}

METRIC_WORDS = {
    "max_rel_position_error": "position off by",
    "max_rel_velocity_error": "speed off by",
    "max_rel_energy_drift": "energy changed by",
    "max_rel_momentum_drift": "momentum changed by",
    "max_rel_angular_momentum_drift": "angular momentum changed by",
    "max_rel_barycenter_drift": "barycenter moved by",
    "max_rel_error": "difference of",
    "energy_error_growth_factor": "last-orbit error is",
    "max_order_deviation": "order deviates by",
    "min_observed_order": "measured order",
    "mismatching_values": "differing values",
    "non_finite_values": "NaN / infinite values",
    "bodies_out": "bodies returned",
}

GLOSSARY = [
    ("Physics step (frame)", "One call to the module's update() with a time step dt of 7200 s (two hours of "
                             "simulated time). The engine calls it repeatedly; its duration bounds the frame rate."),
    ("Relative error", "An error divided by a natural size of the problem, so that scenes of very different "
                       "scale can share one limit. 0.001% of an orbit radius of 150 million km is 1 500 km."),
    ("Limit", "The largest error accepted for a check. All limits are in config/thresholds.json and were "
              "calibrated at 3–10× the value measured on a known-good version."),
    ("Median", "The middle value of all measured step times: half the steps were faster, half slower. It "
               "ignores the occasional hiccup that would distort an average."),
    ("Regression", "A version that is slower than the previous one by more than the configured percentage, on "
                   "a body count large enough for the measurement to be meaningful."),
    ("Integrator", "The numerical recipe that advances positions and velocities in time. The module uses velocity "
                   "Verlet, a second-order, symplectic, time-reversible scheme."),
    ("Symplectic", "A property of some integrators: the energy error oscillates around zero forever instead of "
                   "accumulating, which is what keeps orbits stable over long simulations."),
    ("Reference simulation", "An independent, much slower integrator (RK4 in 80-bit floating point, 16 sub-steps "
                             "per frame) used as ground truth when no exact formula exists."),
    ("NaN", "\"Not a number\": the result of an invalid operation such as dividing zero by zero. Once produced, it "
            "contaminates everything it touches."),
]

CSS = """
:root { color-scheme: light dark;
  --page:#f6f6f3; --surface-1:#fcfcfb; --surface-2:#f1f1ee; --surface-3:#e7e7e2; --border:#e1e0d9; --grid:#e6e6e1;
  --text-primary:#1a1a19; --text-secondary:#52514e; --text-muted:#7a7a74;
  --series-1:#2a78d6; --series-2:#eb6834; --series-3:#1baf7a; --series-4:#eda100;
  --series-5:#e87ba4; --series-6:#008300; --series-7:#4a3aa7; --series-8:#e34948;
  --good:#0ca30c; --warning:#fab219; --serious:#ec835a; --critical:#d03b3b; --info:#2a78d6;
  --good-ink:#006300; --critical-ink:#b42d2d; --warning-ink:#8a5a00;
  --good-soft:#e6f4e6; --critical-soft:#fbeaea; --warning-soft:#fdf3dc; --info-soft:#e8f0fb;
  --shadow: 0 1px 2px rgba(0,0,0,.04); }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --page:#111110; --surface-1:#1a1a19; --surface-2:#242422; --surface-3:#30302d; --border:#33332f; --grid:#2c2c2a;
  --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8d8d86;
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70; --series-4:#c98500;
  --series-5:#d55181; --series-6:#008300; --series-7:#9085e9; --series-8:#e66767; --info:#3987e5;
  --good-ink:#4cc94c; --critical-ink:#f08080; --warning-ink:#f5c04a;
  --good-soft:#16301a; --critical-soft:#3a1c1c; --warning-soft:#3a2f14; --info-soft:#18273b;
  --shadow: none; } }
:root[data-theme="dark"] {
  --page:#111110; --surface-1:#1a1a19; --surface-2:#242422; --surface-3:#30302d; --border:#33332f; --grid:#2c2c2a;
  --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8d8d86;
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70; --series-4:#c98500;
  --series-5:#d55181; --series-6:#008300; --series-7:#9085e9; --series-8:#e66767; --info:#3987e5;
  --good-ink:#4cc94c; --critical-ink:#f08080; --warning-ink:#f5c04a;
  --good-soft:#16301a; --critical-soft:#3a1c1c; --warning-soft:#3a2f14; --info-soft:#18273b;
  --shadow: none; }
* { box-sizing: border-box; }
html { scroll-padding-top: 120px; }
body { margin:0; background:var(--page); color:var(--text-primary);
  font: 15px/1.55 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
main { max-width: 1120px; margin: 0 auto; padding: 20px 16px 72px; }
a { color: var(--info); }
h2 { font-size: 24px; line-height: 1.25; margin: 0 0 6px; letter-spacing: -.01em; }
h3 { font-size: 17px; margin: 0 0 4px; }
p { margin: 0 0 10px; color: var(--text-secondary); }
.muted { color: var(--text-muted); font-size: 13px; }
.lead { font-size: 15px; max-width: 72ch; }
code { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12.5px; background: var(--surface-2);
  padding: 1px 5px; border-radius: 4px; }
.sr { position: absolute; width: 1px; height: 1px; overflow: hidden; clip: rect(0 0 0 0); white-space: nowrap; }
:focus-visible { outline: 2px solid var(--info); outline-offset: 2px; border-radius: 4px; }

/* header + tabs */
header.top { position: sticky; top: 0; z-index: 20; background: color-mix(in srgb, var(--page) 92%, transparent);
  backdrop-filter: blur(8px); border-bottom: 1px solid var(--border); }
.top-inner { max-width: 1120px; margin: 0 auto; padding: 10px 16px 0; display: flex; align-items: baseline;
  gap: 4px 12px; flex-wrap: wrap; }
.brand { font-weight: 700; font-size: 16px; color: var(--text-primary); text-decoration: none; }
.brand-sub { font-size: 13px; color: var(--text-muted); }
nav.tabs { max-width: 1120px; margin: 0 auto; padding: 6px 12px 0; display: flex; gap: 2px; overflow-x: auto;
  scrollbar-width: none; }
nav.tabs::-webkit-scrollbar { display: none; }
nav.tabs a { display: inline-flex; align-items: center; gap: 8px; white-space: nowrap; padding: 8px 12px 10px;
  color: var(--text-secondary); text-decoration: none; font-size: 14px; font-weight: 500;
  border-bottom: 2px solid transparent; }
nav.tabs a:hover { color: var(--text-primary); }
nav.tabs a.active { color: var(--text-primary); border-bottom-color: var(--info); }

/* chips, badges, pills: always icon + text, never colour alone */
.chip { display: inline-flex; align-items: center; gap: 4px; padding: 1px 8px; border-radius: 999px; font-size: 12px;
  font-weight: 600; line-height: 18px; white-space: nowrap; background: var(--surface-3); color: var(--text-secondary); }
.chip.good { background: var(--good-soft); color: var(--good-ink); }
.chip.critical { background: var(--critical-soft); color: var(--critical-ink); }
.chip.warning, .chip.serious { background: var(--warning-soft); color: var(--warning-ink); }
.badge { display:inline-block; padding:1px 8px; border-radius:999px; font-size:12px; font-weight:600; color:#fff;
  white-space:nowrap; line-height: 18px; }
.badge.good { background: var(--good); } .badge.critical { background: var(--critical); }
.badge.warning { background: var(--warning); color:#1a1a19; } .badge.serious { background: var(--serious); color:#1a1a19; }
.badge.muted { background: var(--text-muted); }
.pills { display: flex; flex-wrap: wrap; gap: 4px; }
.pill { display: inline-flex; align-items: center; gap: 3px; padding: 0 7px; border-radius: 6px; font-size: 12px;
  line-height: 20px; font-weight: 600; background: var(--surface-3); color: var(--text-secondary); white-space: nowrap; }
.pill.good { background: var(--good-soft); color: var(--good-ink); }
.pill.critical { background: var(--critical-soft); color: var(--critical-ink); }
.pill.warning, .pill.serious { background: var(--warning-soft); color: var(--warning-ink); }
.swatch { display:inline-block; width: 10px; height: 10px; border-radius: 3px; margin-right: 6px; vertical-align: 0; }

/* page structure */
.page { display: none; } .page.active { display: block; }
.page-head { margin: 8px 0 20px; }
.subnav { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 12px; }
.subnav a { display: inline-flex; align-items: center; gap: 6px; padding: 4px 12px; border: 1px solid var(--border);
  border-radius: 999px; font-size: 13px; color: var(--text-secondary); text-decoration: none; background: var(--surface-1); }
.subnav a:hover { border-color: var(--text-muted); color: var(--text-primary); }
.block { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px; padding: 18px 20px;
  margin: 0 0 18px; box-shadow: var(--shadow); }
.block > h3 + p { margin-top: 2px; }
.block-head { display: flex; justify-content: space-between; align-items: baseline; gap: 8px 16px; flex-wrap: wrap; }

/* hero verdict */
.hero { display: flex; gap: 16px; align-items: flex-start; border-radius: 14px; padding: 20px 22px; margin: 4px 0 16px;
  border: 1px solid var(--border); background: var(--surface-1); box-shadow: var(--shadow); }
.hero.good { border-left: 6px solid var(--good); } .hero.critical { border-left: 6px solid var(--critical); }
.hero.warning { border-left: 6px solid var(--warning); }
.hero-icon { flex: none; width: 40px; height: 40px; border-radius: 50%; display: grid; place-items: center;
  font-size: 20px; font-weight: 700; color: #fff; background: var(--good); }
.hero.critical .hero-icon { background: var(--critical); } .hero.warning .hero-icon { background: var(--warning); color:#1a1a19; }
.hero-kicker { font-size: 13px; color: var(--text-muted); }
.hero-title { font-size: 22px; font-weight: 650; line-height: 1.3; margin: 2px 0 4px; }
.hero ul { margin: 6px 0 0; padding: 0; list-style: none; color: var(--text-secondary); }
.hero li { margin: 2px 0; }

/* tiles */
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 12px; margin: 0 0 18px; }
.tile { display: flex; flex-direction: column; gap: 2px; background: var(--surface-1); border: 1px solid var(--border);
  border-radius: 12px; padding: 14px 16px; text-decoration: none; color: inherit; box-shadow: var(--shadow); }
a.tile:hover { border-color: var(--text-muted); }
.tile .label { font-size: 12px; color: var(--text-muted); text-transform: uppercase; letter-spacing: .05em; font-weight: 600; }
.tile .value { font-size: 24px; font-weight: 650; line-height: 1.25; margin: 2px 0; }
.tile .sub { font-size: 13px; color: var(--text-secondary); }
.tile .go { margin-top: auto; padding-top: 8px; font-size: 13px; color: var(--info); font-weight: 500; }
.progress { height: 6px; border-radius: 3px; background: var(--critical-soft); overflow: hidden; margin: 6px 0 2px; }
.progress > span { display: block; height: 100%; background: var(--good); border-radius: 3px; }
.progress.all { background: var(--surface-3); }

/* explainers */
details.explain { background: var(--info-soft); border-radius: 10px; padding: 10px 16px; margin: 0 0 18px; }
details.explain > summary { cursor: pointer; font-weight: 600; color: var(--text-primary); }
details.explain[open] > summary { margin-bottom: 8px; }
details.explain p { margin: 6px 0; }
.notice { display: flex; gap: 10px; align-items: flex-start; border-radius: 10px; padding: 10px 14px; margin: 0 0 18px;
  background: var(--warning-soft); color: var(--text-primary); font-size: 14px; }
.notice p { margin: 0; color: var(--text-primary); }
.guide { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 8px 20px; }
.guide div { font-size: 14px; color: var(--text-secondary); }
.guide b { display: block; color: var(--text-primary); }

/* tables */
.wrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-size: 14px; }
th, td { text-align: left; padding: 10px 10px; border-bottom: 1px solid var(--border); vertical-align: top; }
tr:last-child td { border-bottom: none; }
th { color: var(--text-muted); font-weight: 600; font-size: 12px; text-transform: uppercase; letter-spacing: .04em; }
th .muted { text-transform: none; letter-spacing: 0; font-weight: 500; }
td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; white-space: nowrap; }
.delta-up { color: var(--critical-ink); font-weight:600; } .delta-down { color: var(--good-ink); font-weight:600; }

/* check rows */
.rows-head, details.row > summary { display: grid; grid-template-columns: minmax(0, 1.25fr) minmax(0, 1.2fr) minmax(0, .8fr);
  gap: 16px; align-items: start; }
.rows-head { padding: 0 12px 6px 34px; font-size: 12px; color: var(--text-muted); text-transform: uppercase;
  letter-spacing: .04em; font-weight: 600; border-bottom: 1px solid var(--border); }
details.row { border-bottom: 1px solid var(--border); }
details.row:last-child { border-bottom: none; }
details.row > summary { list-style: none; cursor: pointer; padding: 12px 12px 12px 34px; position: relative; border-radius: 8px; }
details.row > summary::-webkit-details-marker { display: none; }
details.row > summary::before { content: "›"; position: absolute; left: 12px; top: 10px; font-size: 20px; line-height: 1;
  color: var(--text-muted); transition: transform .15s; }
details.row[open] > summary::before { transform: rotate(90deg); }
details.row > summary:hover { background: var(--surface-2); }
.row-body { padding: 0 12px 16px 34px; font-size: 14px; }
.row-body p { margin: 0 0 8px; }
.check-name { font-weight: 600; } .check-q { color: var(--text-secondary); font-size: 13px; }
.reading { margin-top: 4px; font-size: 14px; }
.meter { position: relative; height: 6px; border-radius: 3px; background: var(--surface-3); margin: 6px 0 2px; max-width: 260px; overflow: hidden; }
.meter > span { position:absolute; left:0; top:0; bottom:0; border-radius: 3px; background: var(--good); }
.meter.over > span { background: var(--critical); } .meter.warn > span { background: var(--warning); }
.meter-label { font-size: 12px; color: var(--text-muted); }
.col-label { display: none; font-size: 11px; color: var(--text-muted); text-transform: uppercase; letter-spacing: .04em;
  font-weight: 600; margin-bottom: 2px; }
.mini { width: 100%; font-size: 13px; margin-top: 8px; } .mini td, .mini th { padding: 6px 8px; }

/* unit tests */
.test-list { list-style: none; margin: 8px 0 0; padding: 0; }
.test-list li { padding: 8px 0; border-top: 1px solid var(--border); }
.test-list li:first-child { border-top: none; }
.test-line { display: flex; gap: 10px; align-items: baseline; flex-wrap: wrap; }
.test-line .name { font-weight: 600; flex: 1 1 280px; } .test-line .pills { flex: none; }
details.more { margin-top: 10px; } details.more > summary { cursor: pointer; color: var(--text-secondary); font-size: 14px; }
pre.failure { white-space: pre-wrap; word-break: break-word; font-size: 12px; background: var(--surface-2);
  padding: 8px 10px; border-radius: 6px; margin: 6px 0 0; max-height: 220px; overflow: auto; }
details.fail-msg > summary { cursor: pointer; font-size: 13px; color: var(--text-muted); }

/* charts */
.charts { display:grid; grid-template-columns: repeat(auto-fit, minmax(min(100%, 480px), 1fr)); gap: 16px; }
figure { margin: 0; } figcaption { font-size: 13px; color: var(--text-secondary); margin: 6px 0 0; }
.chart-card { background: var(--surface-1); border: 1px solid var(--border); border-radius: 12px; padding: 14px 16px; box-shadow: var(--shadow); }
.chart-verdict { font-size: 14px; margin: 0 0 6px; color: var(--text-primary); }
.chart-card details { margin-top: 6px; font-size: 13px; } .chart-card summary { cursor: pointer; color: var(--text-secondary); }
.chart-card details p { font-size: 13px; margin: 6px 0 0; }
svg.chart { width: 100%; height: auto; display:block; }
.chart-title { font-size: 14px; font-weight: 600; fill: var(--text-primary); }
.tick { font-size: 11px; fill: var(--text-muted); font-variant-numeric: tabular-nums; } .axis-label { font-size: 12px; fill: var(--text-secondary); }
.grid { stroke: var(--grid); stroke-width: 1; } .axis { stroke: var(--border); stroke-width: 1; }
.line { fill: none; stroke-width: 2; stroke-linejoin: round; } .marker { stroke: var(--surface-1); stroke-width: 2; }
.marker:hover { r: 6; } .bar:hover { opacity: .8; } .bar.muted-bar { fill: var(--text-muted); opacity: .45; }
.threshold { stroke: var(--critical); stroke-width: 1.5; stroke-dasharray: 6 4; }
.direct-label { font-size: 12px; fill: var(--text-secondary); }
.legend { display:flex; flex-wrap:wrap; gap: 2px 14px; font-size: 12px; color: var(--text-secondary); margin: 4px 0 2px; }
.chart-empty { color: var(--text-muted); font-size: 13px; padding: 12px; }
.over-zone { fill: var(--critical); opacity: .07; }
.threshold-label { font-size: 11px; fill: var(--critical); }
text.callout { font-size: 12px; font-weight: 600; fill: var(--text-primary); }
.callout-ring { fill: none; stroke: var(--text-primary); stroke-width: 1.5; }

dl.glossary { display: grid; grid-template-columns: max-content 1fr; gap: 10px 20px; margin: 0; }
dl.glossary dt { font-weight: 600; } dl.glossary dd { margin: 0; color: var(--text-secondary); }

@media (max-width: 760px) {
  .rows-head { display: none; }
  details.row > summary { grid-template-columns: 1fr; gap: 8px; }
  .col-label { display: block; }
  dl.glossary { grid-template-columns: 1fr; gap: 2px; } dl.glossary dd { margin-bottom: 8px; }
  .hero { padding: 16px; } .hero-title { font-size: 19px; }
}
@media print {
  header.top { position: static; } nav.tabs { display: none; }
  .page { display: block !important; break-before: page; }
}
"""


# ----------------------------------------------------------------------------- formatting helpers

def _badge(status: str, text: str | None = None) -> str:
    icon, label, cls = STATUS.get(status, STATUS["skip"])
    return f'<span class="badge {cls}" title="{label}">{icon} {text or label}</span>'


def _ms(v) -> str:
    return "–" if v is None else f"{v:.4f}" if v < 1 else f"{v:.3f}" if v < 100 else f"{v:.1f}"


_SUP = str.maketrans("0123456789-", "⁰¹²³⁴⁵⁶⁷⁸⁹⁻")


def _sci(value: float) -> str:
    """Scientific notation with a real exponent: 4.5×10⁻⁶."""
    if value == 0:
        return "0"
    if not math.isfinite(value):
        return "NaN"
    exponent = math.floor(math.log10(abs(value)))
    mantissa = value / 10 ** exponent
    if abs(mantissa - round(mantissa)) < 5e-3:
        mantissa_text = f"{round(mantissa):d}"
    else:
        mantissa_text = f"{mantissa:.2g}".rstrip("0").rstrip(".")
    return f"{mantissa_text}×10{str(exponent).translate(_SUP)}"


def _plain(fraction: float) -> str:
    """Relative error as plain text: a percentage, or '1 part in 10ⁿ' when tiny."""
    if fraction == 0:
        return "0%"
    if not math.isfinite(fraction):
        return "not a number"
    p = fraction * 100
    if p >= 1e-4:
        return f"{p:.3g}%"
    exponent = math.floor(math.log10(1.0 / fraction))
    mantissa = (1.0 / fraction) / 10 ** exponent
    lead = "" if mantissa < 1.5 else f"{mantissa:.0f}×"
    return f"1 part in {lead}10{str(exponent).translate(_SUP)}"


def _pct(fraction: float, sci: bool = True) -> str:
    """Plain wording followed by the scientific value in brackets: 0.00045% (4.5×10⁻⁶)."""
    plain = _plain(fraction)
    if not sci or fraction == 0 or fraction >= 0.01 or not math.isfinite(fraction):
        return plain  # above 1 % the percentage alone is clearer than "200% (2×10⁰)"
    return f"{plain} ({_sci(fraction)})"


def _km(value: float) -> str:
    if value >= 1e6:
        return f"{value / 1e6:.3g} million km"
    if value >= 1000:
        return f"{value:,.0f} km"
    if value >= 1:
        return f"{value:.3g} km"
    if value >= 1e-3:
        return f"{value * 1000:.3g} m"
    return f"{value * 1e6:.3g} mm"


def _steps_per_s(ms: float) -> str:
    if not ms:
        return "–"
    rate = 1000.0 / ms
    return f"{rate:,.0f} steps/s" if rate >= 10 else f"{rate:.1f} steps/s"


def _humanize(name: str) -> str:
    """GoogleTest CamelCase name -> sentence: NonZeroEpsilonIsKeptAsIs -> Non zero epsilon is kept as is."""
    words = re.findall(r"NaN|[A-Z]+(?=[A-Z][a-z]|\b|\d)|[A-Z]?[a-z]+|\d+", name) or [name]
    text = " ".join(w if w == "NaN" or (w.isupper() and len(w) > 1) else w.lower() for w in words)
    return text[:1].upper() + text[1:]


def _version_name(i: int) -> str:
    return f"v{i + 1}"


def _series_name(i: int, e: Entry) -> str:
    return f"{_version_name(i)} · {e.meta.get('commit', e.label)[:7]}"


def _col_header(i: int, e: Entry) -> str:
    return (f"<span class='swatch' style='background:var(--series-{i % 8 + 1})'></span>{_version_name(i)}"
            f"<br><span class='muted'>{escape(e.meta.get('commit', e.label)[:7])}</span>")


def _reading(metric: dict, context: dict) -> str:
    """One plain sentence for the primary metric of a check."""
    name, value = metric["name"], metric["value"]
    if value is None:
        return "not measured"
    words = METRIC_WORDS.get(name, name.replace("_", " "))
    if name in ("mismatching_values", "non_finite_values", "bodies_out"):
        return f"{int(value)} {words}"
    if name == "energy_error_growth_factor":
        return f"{words} {value:.2f}× the first-orbit error"
    if name == "max_order_deviation":
        orders = context.get("orders") or []
        measured = f"{min(orders):.2f}–{max(orders):.2f}" if orders else f"{value:.2f} from"
        return f"measured order {measured} (expected {context.get('expected_order', 2)})"
    if name == "min_observed_order":
        return f"{words} {value:.3f}"
    text = f"{words} {_pct(value)}"
    scale_km = context.get("scale_km")
    scale_name = context.get("scale_name")
    if name.endswith("position_error") and scale_km:
        text += f" of the {scale_name} (≈ {_km(value * scale_km)})"
    elif name.endswith("velocity_error") and context.get("speed_scale_km_s"):
        text += f" of the typical speed (≈ {value * context['speed_scale_km_s'] * 1000:.3g} m/s)"
    elif name in ("max_rel_error", "max_rel_barycenter_drift") and scale_km:
        text += f" of the {scale_name}"
    return text


def _meter(metric: dict) -> str:
    """Bar showing how much of the allowed error was used."""
    value, limit, op = metric["value"], metric["threshold"], metric["op"]
    if value is None or limit is None:
        return ""
    if op == "==":
        ok = value == limit
        return f"<div class='meter-label'>{'exactly as required' if ok else 'must be ' + svg.fmt(limit)}</div>"  # noqa
    if op == ">=":
        return f"<div class='meter-label'>must be at least {svg.fmt(limit)}</div>"
    if limit == 0:
        return ""
    ratio = value / limit
    width = max(1.5, min(100.0, ratio * 100.0))
    cls = "over" if ratio > 1 else ("warn" if ratio > 0.7 else "")
    limit_text = f"limit {_plain(limit)}, {_sci(limit)}" if limit < 1 else f"limit {svg.fmt(limit)}"
    if ratio > 1:
        label = f"{ratio:,.0f}× over the limit ({limit_text})" if ratio >= 10 else f"{ratio:.1f}× over the limit ({limit_text})"
    elif ratio < 0.001:
        label = f"far below the limit ({limit_text})"
    else:
        label = f"{ratio * 100:.0f}% of the allowed error ({limit_text})"
    return f"<div class='meter {cls}'><span style='width:{width:.1f}%'></span></div><div class='meter-label'>{label}</div>"


def _delta_cell(row: dict | None) -> str:
    if row is None:
        return "<td class='num'>–</td>"
    delta, verdict = row["delta_pct"], row["verdict"]
    cls = {"regression": "delta-up", "improvement": "delta-down"}.get(verdict, "")
    sign = "+" if delta >= 0 else ""
    words = {"regression": "slower", "improvement": "faster", "stable": "no real change", "noise": "too fast to judge"}[verdict]
    return f"<td class='num {cls}'>{sign}{delta:.1f}%<br><span class='muted'>{words}</span></td>"


def _find(entry: Entry, check: str) -> dict | None:
    if not entry.correctness:
        return None
    return next((r for r in entry.correctness["results"] if r["name"] == check), None)


def _primary(r: dict) -> dict | None:
    return r["metrics"][0] if r and r.get("metrics") else None


# ----------------------------------------------------------------------------- sections

def _chip(status: str, text: str) -> str:
    icon, _, cls = STATUS.get(status, STATUS["skip"])
    return f"<span class='chip {cls}'><span aria-hidden='true'>{icon}</span> {escape(text)}</span>"


def _pill(i: int, status: str | None, detail: str = "") -> str:
    """Compact per-version status: 'v2 ✓', with the reading in the tooltip and for screen readers."""
    icon, label, cls = STATUS.get(status or "skip", STATUS["skip"])
    if status is None:
        icon, label = "–", "not run"
    tip = f"{_version_name(i)}: {label}" + (f" — {detail}" if detail else "")
    return (f"<span class='pill {cls}' title='{escape(tip, quote=True)}'>{_version_name(i)} "
            f"<span aria-hidden='true'>{icon}</span><span class='sr'>{escape(label)}</span></span>")


def _ok(summary: dict | None) -> bool:
    return bool(summary) and not (summary["fail"] or summary["error"])


def _progress(done: int, total: int) -> str:
    pct = 100.0 * done / total if total else 0.0
    return f"<div class='progress{' all' if done == total else ''}'><span style='width:{pct:.1f}%'></span></div>"


def _slug(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")


def _speed_rows(with_bench: list[Entry], plan: dict) -> list[dict]:
    if len(with_bench) < 2:
        return []
    return bench.compare(with_bench[-2].benchmark, with_bench[-1].benchmark, plan["regression_threshold_pct"],
                         plan["improvement_threshold_pct"], plan.get("min_ms_for_verdict", 0.0))


def _speed_verdict(rows: list[dict]) -> tuple[str, str] | None:
    """(status, short text) for the newest benchmark compared with the previous one, None without a comparison."""
    if not rows:
        return None
    judged = [r for r in rows if r["verdict"] != "noise"]
    regs = [r for r in judged if r["verdict"] == "regression"]
    imps = [r for r in judged if r["verdict"] == "improvement"]
    if regs:
        worst = max(regs, key=lambda r: r["delta_pct"])
        return "fail", f"{worst['delta_pct']:+.0f}% slower at {worst['bodies']:,} bodies"
    if imps:
        best = min(imps, key=lambda r: r["delta_pct"])
        return "pass", f"{-best['delta_pct']:.0f}% faster at {best['bodies']:,} bodies"
    return "skip", "no significant change"


def _page_head(title: str, lead: str, subnav: list[tuple[str, str, str]] | None = None) -> str:
    """subnav: [(anchor id, label, extra html such as a chip)]."""
    out = [f"<div class='page-head'><h2>{escape(title)}</h2><p class='lead'>{lead}</p>"]
    if subnav:
        out.append("<nav class='subnav' aria-label='On this page'>" + "".join(
            f"<a href='#{aid}'>{escape(label)}{extra}</a>" for aid, label, extra in subnav) + "</nav>")
    out.append("</div>")
    return "".join(out)


# ----------------------------------------------------------------------------- overview

def _section_overview(entries: list[Entry], with_bench: list[Entry], with_unit: list[Entry], plan: dict) -> str:
    latest, i_latest = entries[-1], len(entries) - 1
    c = latest.correctness["summary"] if latest.correctness else None
    u = latest.unit["summary"] if latest.unit else None
    rows = _speed_rows(with_bench, plan) if latest in with_bench else []
    speed = _speed_verdict(rows)

    issues, fine, notes = [], [], []
    if c:
        bad = c["fail"] + c["error"]
        if bad:
            issues.append(f"{bad} physics check{'s' if bad > 1 else ''} fail{'' if bad > 1 else 's'}")
        else:
            fine.append(f"All {c['pass']} required physics checks pass"
                        + (f" ({c['warn']} known limitation{'s' if c['warn'] > 1 else ''} reported)" if c["warn"] else ""))
    else:
        notes.append("Physics checks were not run for this version")
    if u:
        if _ok(u):
            fine.append(f"All {u['total']} unit tests pass")
        else:
            failing = [s["name"] for s in latest.unit["suites"] if any(t["status"] == "fail" for t in s["tests"])]
            issues.append(f"{u['fail']} of {u['total']} unit tests fail" + (f" ({', '.join(failing)})" if failing else "")
                          + (" — the test program crashed" if u["error"] else ""))
    else:
        notes.append("Unit tests were not run for this version")
    if speed:
        prev = _version_name(entries.index(with_bench[-2]))
        noisy = any(e.meta.get("governor") != "performance" for e in with_bench[-2:])
        caveat = " (timings noisy: CPU governor not set to performance)" if noisy and speed[0] != "skip" else ""
        if speed[0] == "fail":
            issues.append(f"Speed: {speed[1]} than {prev}{caveat}")
        elif speed[0] == "pass":
            fine.append(f"Speed: {speed[1]} than {prev}{caveat}")
        else:
            fine.append(f"Speed: no significant change compared with {prev}")

    status = "critical" if issues else "good"
    title = ("Needs attention: " + issues[0][0].lower() + issues[0][1:]) if issues else "Everything checked passes"
    m = latest.meta
    items = ([f"<li><b>✗</b> {escape(t)}</li>" for t in issues] + [f"<li>✓ {escape(t)}</li>" for t in fine]
             + [f"<li class='muted'>– {escape(t)}</li>" for t in notes])
    out = [f"<section class='hero {status}' aria-label='Verdict for the newest version'>"
           f"<div class='hero-icon' aria-hidden='true'>{'✗' if issues else '✓'}</div><div>"
           f"<div class='hero-kicker'>Newest version · <b>{_version_name(i_latest)}</b> · "
           f"<code>{escape(m.get('commit', latest.label)[:7])}</code> · {escape(m.get('subject', ''))} · "
           f"{escape(m.get('commit_date', '')[:10])}</div>"
           f"<div class='hero-title'>{escape(title)}</div><ul>{''.join(items)}</ul></div></section>"]

    tiles = []
    if c:
        total = c["pass"] + c["fail"] + c["error"]
        limitation = f" · {c['warn']} known limitation" if c["warn"] else ""
        tiles.append(f"<a class='tile' href='#checks'><span class='label'>Physics checks</span>"
                     f"<span class='value'>{c['pass']} / {total} pass</span>{_progress(c['pass'], total)}"
                     f"<span class='sub'>Motion compared with exact solutions and conservation laws{limitation}</span>"
                     f"<span class='go'>See the checks →</span></a>")
    if u:
        failing = [f"{s['name']} ({sum(t['status'] == 'fail' for t in s['tests'])})" for s in latest.unit["suites"]
                   if any(t["status"] == "fail" for t in s["tests"])]
        tiles.append(f"<a class='tile' href='#unit'><span class='label'>Unit tests</span>"
                     f"<span class='value'>{u['pass']} / {u['total']} pass</span>{_progress(u['pass'], u['total'])}"
                     f"<span class='sub'>{('Failing: ' + escape(', '.join(failing))) if failing else 'Softening, collisions and octree internals'}</span>"
                     f"<span class='go'>See the unit tests →</span></a>")
    if latest in with_bench:
        big = max(latest.benchmark["results"], key=lambda r: r["bodies"])
        change = f"<span class='sub'>{_chip(speed[0], speed[1])} vs {_version_name(entries.index(with_bench[-2]))}</span>" if speed else ""
        tiles.append(f"<a class='tile' href='#speed'><span class='label'>Speed</span>"
                     f"<span class='value'>{_ms(big['update_ms']['median'])} ms</span>"
                     f"<span class='sub'>per physics step with {big['bodies']:,} bodies "
                     f"({_steps_per_s(big['update_ms']['median'])})</span>{change}"
                     f"<span class='go'>See the timings →</span></a>")
    kep = _find(latest, "kepler_circular")
    km = _primary(kep)
    if km and km["value"] is not None and kep.get("context", {}).get("scale_km"):
        tiles.append(f"<a class='tile' href='#charts'><span class='label'>Accuracy headline</span>"
                     f"<span class='value'>{_km(km['value'] * kep['context']['scale_km'])}</span>"
                     f"<span class='sub'>off after one simulated year of an Earth-like orbit, i.e. {_pct(km['value'])} "
                     f"of the orbit radius</span><span class='go'>See the error charts →</span></a>")
    out.append("<div class='tiles'>" + "".join(tiles) + "</div>")

    out.append(f"""<details class='explain'><summary>How to read this report</summary>
<p>It compares {len(entries)} version{'s' if len(entries) > 1 else ''} of the physics module, numbered v1 (oldest) to
v{len(entries)} (newest). Each version was built from its git commit and driven exactly like the game engine does.</p>
<p><b>Speed</b> is how long one physics step takes: lower is better. <b>Physics checks</b> compare the simulated motion
with exact solutions; each one reports a measured error next to the limit it must stay under.
<b>Unit tests</b> check internals the engine cannot see (softening, collisions) and simply pass or fail.</p>
<p>{_badge('pass')} within the limit · {_badge('fail')} outside it · {_badge('warn')} known limitation, reported but not
enforced. Use the tabs at the top to move between pages; every chip in the tabs tells you whether that page needs a look.</p>
</details>""")
    out.append(_section_versions(entries))
    return "\n".join(out)


def _section_versions(entries: list[Entry]) -> str:
    machines = {(e.meta.get("cpu", "?"), e.meta.get("compiler", ""), e.meta.get("build_type", ""),
                 e.meta.get("governor", "?")) for e in entries}
    per_row = len(machines) > 1
    out = ["<section class='block' id='versions'><div class='block-head'><h3>Versions compared</h3>"
           "<span class='muted'>oldest first</span></div>",
           "<div class='wrap'><table><tr><th>Version</th><th>Change</th><th>Date</th><th>Physics checks</th>"
           f"<th>Unit tests</th>{'<th>Machine</th>' if per_row else ''}</tr>"]
    for i, e in enumerate(entries):
        m = e.meta
        c = e.correctness["summary"] if e.correctness else None
        u = e.unit["summary"] if e.unit else None
        checks = (_chip("pass" if _ok(c) else "fail", f"{c['pass']}/{c['pass'] + c['fail'] + c['error']}")
                  + (f" <span class='muted'>{c['warn']} known limitation</span>" if c["warn"] else "")) if c else "<span class='muted'>not run</span>"
        unit = _chip("pass" if _ok(u) else "fail", f"{u['pass']}/{u['total']}") if u else "<span class='muted'>not run</span>"
        machine = (f"<td>{escape(m.get('cpu', '?'))}<br><span class='muted'>{escape(m.get('compiler', ''))} · "
                   f"governor {escape(m.get('governor', '?'))}</span></td>") if per_row else ""
        out.append(f"<tr><td><span class='swatch' style='background:var(--series-{i % 8 + 1})'></span><b>{_version_name(i)}</b>"
                   f"{'<br><span class=muted>uncommitted changes</span>' if m.get('dirty') else ''}</td>"
                   f"<td>{escape(m.get('subject', ''))}<br><code>{escape(m.get('commit', e.label)[:7])}</code></td>"
                   f"<td class='num'>{escape(m.get('commit_date', '')[:10])}</td><td>{checks}</td><td>{unit}</td>{machine}</tr>")
    out.append("</table></div>")
    if not per_row:
        cpu, compiler, build_type, governor = next(iter(machines))
        out.append(f"<p class='muted' style='margin:10px 0 0'>All versions measured on the same machine: {escape(cpu)} · "
                   f"{escape(compiler)} · {escape(build_type)} build · CPU governor <code>{escape(governor)}</code>.</p>")
    out.append("</section>")
    return "\n".join(out)


# ----------------------------------------------------------------------------- speed

def _section_speed(entries: list[Entry], with_bench: list[Entry], plan: dict) -> str:
    if not with_bench:
        return _page_head("Speed", "No benchmark data stored.")
    out = [_page_head("Speed", "How long the module needs to compute one physics step, for scenes of growing size. "
                      "Every version computes exactly the same random clusters (fixed seed); each figure is the median "
                      "of several fresh runs after a warm-up. Lower is better.")]
    latest, previous = with_bench[-1], (with_bench[-2] if len(with_bench) >= 2 else None)
    rows = _speed_rows(with_bench, plan)
    governors = {e.meta.get("governor", "unknown") for e in with_bench}
    if governors - {"performance"}:
        out.append(f"<div class='notice' role='note'><span aria-hidden='true'>⚠</span><p><b>Timings are noisy.</b> The CPU "
                   f"frequency governor was <code>{escape(', '.join(sorted(governors)))}</code> during at least one run, so "
                   f"the same code can vary by 10–30% between runs, especially on small scenes. Set it to "
                   f"<code>performance</code> before comparing versions.</p></div>")

    res = latest.benchmark["results"]
    big = max(res, key=lambda r: r["bodies"])
    mid = next((r for r in res if r["bodies"] == 1000), None)
    tiles = [f"<div class='tile'><span class='label'>{big['bodies']:,} bodies</span>"
             f"<span class='value'>{_ms(big['update_ms']['median'])} ms</span>"
             f"<span class='sub'>per step for {_version_name(entries.index(latest))} · {_steps_per_s(big['update_ms']['median'])}</span></div>"]
    if mid and mid is not big:
        tiles.append(f"<div class='tile'><span class='label'>1,000 bodies</span>"
                     f"<span class='value'>{_ms(mid['update_ms']['median'])} ms</span>"
                     f"<span class='sub'>per step · {_steps_per_s(mid['update_ms']['median'])}</span></div>")
    verdict = _speed_verdict(rows)
    if verdict:
        tiles.append(f"<div class='tile'><span class='label'>{_version_name(entries.index(latest))} vs "
                     f"{_version_name(entries.index(previous))}</span><span class='value'>{_chip(verdict[0], verdict[1])}</span>"
                     f"<span class='sub'>A change counts only above {plan['regression_threshold_pct']:g}% and on scenes "
                     f"slower than {plan.get('min_ms_for_verdict', 0)} ms per step.</span></div>")
    out.append("<div class='tiles'>" + "".join(tiles) + "</div>")

    line_series = [{"name": _series_name(entries.index(e), e), "slot": entries.index(e),
                    "x": [r["bodies"] for r in e.benchmark["results"]],
                    "y": [r["update_ms"]["median"] for r in e.benchmark["results"]]} for e in with_bench]
    out.append("<div class='charts' style='margin-bottom:18px'><figure class='chart-card'>")
    out.append(svg.line_chart(line_series, "Time per physics step", "number of bodies", "milliseconds per step",
                              log_x=True, log_y=True, width=560, height=320))
    out.append("<figcaption>Both axes are logarithmic: the straight line means the cost grows like the square of the "
               "number of bodies, as expected when every body attracts every other one.</figcaption></figure>")
    if rows:
        out.append("<figure class='chart-card'>")
        out.append(svg.bar_chart([f"{r['bodies']:,}" for r in rows],
                                 [{"name": f"{_version_name(entries.index(latest))} vs {_version_name(entries.index(previous))}",
                                   "slot": entries.index(latest), "values": [r["delta_pct"] for r in rows]}],
                                 f"Change of {_version_name(entries.index(latest))} compared with {_version_name(entries.index(previous))}",
                                 "% slower (+) or faster (−)", threshold=plan["regression_threshold_pct"],
                                 threshold_label="limit", symmetric=True, width=560, height=320,
                                 muted=[r["verdict"] == "noise" for r in rows]))
        out.append("<figcaption>Above zero means slower. Grey bars are scenes too small to judge: at that scale the "
                   "operating system's noise is larger than any real change.</figcaption></figure>")
    out.append("</div>")

    counts = sorted({r["bodies"] for e in with_bench for r in e.benchmark["results"]})
    delta_by_n = {r["bodies"]: r for r in rows}
    out.append("<section class='block'><div class='block-head'><h3>Detailed timings</h3>"
               "<span class='muted'>median ms per step, spread below · hover for the best step</span></div>"
               "<div class='wrap'><table><tr><th class='num'>bodies</th>")
    for e in with_bench:
        out.append(f"<th class='num'>{_col_header(entries.index(e), e)}</th>")
    out.append(f"<th class='num'>change<br><span class='muted'>{_version_name(entries.index(latest))} vs "
               f"{_version_name(entries.index(previous)) if previous else '–'}</span></th></tr>")
    for n in counts:
        out.append(f"<tr><td class='num'><b>{n:,}</b></td>")
        for e in with_bench:
            r = next((r for r in e.benchmark["results"] if r["bodies"] == n), None)
            if r:
                u = r["update_ms"]
                out.append(f"<td class='num' title='95th percentile {_ms(u['p95'])} ms · best {_ms(u['min'])} ms · "
                           f"{_steps_per_s(u['median'])} · {u['samples']} timed steps'>{_ms(u['median'])}"
                           f"<br><span class='muted'>± {_ms(u['stdev'])}</span></td>")
            else:
                out.append("<td class='num'>–</td>")
        out.append(_delta_cell(delta_by_n.get(n)) + "</tr>")
    out.append("</table></div>")
    cfg = latest.benchmark.get("config", {})
    out.append(f"<p class='muted' style='margin:10px 0 0'>Measurement plan: {cfg.get('repeats')} independent runs × "
               f"({cfg.get('warmup')} warm-up + {cfg.get('frames')} timed steps; fewer for the largest scenes: "
               f"{escape(', '.join(f'{k} bodies → {v}' for k, v in cfg.get('frames_override', {}).items()))}). "
               f"Time step {cfg.get('dt')} s. Only update() is timed.</p></section>")
    return "\n".join(out)


# ----------------------------------------------------------------------------- physics checks

def _check_cell(r: dict | None) -> str:
    if not r:
        return "<span class='muted'>not run</span>"
    m = _primary(r)
    cell = _badge(r["status"])
    if r.get("error"):
        return cell + f"<div class='reading'>{escape(r['error'].splitlines()[0])}</div>"
    if m:
        cell += f"<div class='reading'>{escape(_reading(m, r.get('context', {})))}</div>{_meter(m)}"
    return cell


def _check_detail(r: dict) -> str:
    m = _primary(r)
    if r.get("error"):
        return r["error"].splitlines()[0]
    return _reading(m, r.get("context", {})) if m else ""


def _section_checks(entries: list[Entry], with_tests: list[Entry]) -> str:
    if not with_tests:
        return _page_head("Physics checks", "No correctness data stored.")
    latest = with_tests[-1]
    older = with_tests[:-1]
    names: list[str] = []
    for e in with_tests:
        for r in e.correctness["results"]:
            if r["name"] not in names:
                names.append(r["name"])
    by_entry = {e.label: {r["name"]: r for r in e.correctness["results"]} for e in with_tests}

    cats = []
    for cat, (title, intro) in CATEGORIES.items():
        cat_names = [n for n in names if any(by_entry[e.label].get(n, {}).get("category") == cat for e in with_tests)]
        if cat_names:
            cats.append((cat, title, intro, cat_names))

    def score(cat_names):
        rs = [by_entry[latest.label].get(n) for n in cat_names]
        rs = [r for r in rs if r]
        bad = sum(r["status"] in ("fail", "error") for r in rs)
        return ("fail" if bad else "pass"), f"{len(rs) - bad}/{len(rs)}"

    subnav = []
    for cat, title, _, cat_names in cats:
        st, txt = score(cat_names)
        subnav.append((f"checks-{cat}", title, " " + _chip(st, txt)))
    out = [_page_head("Physics checks",
                      "Each check answers one question about the physics. The newest version is shown in full; earlier "
                      "versions are summarised on the right (hover a pill for its value). Click a row for how it is "
                      "measured and the values of every version.", subnav)]
    out.append(f"""<details class='explain'><summary>How to read a check</summary>
<p>The measured error is written in plain terms (a percentage, and where possible a distance). The bar shows how much
of the allowed error was used: a short green bar is comfortable, a bar near the end is close to the limit, red means
the limit was exceeded.</p>
<p>{_badge('pass')} within the limit · {_badge('fail')} outside it · {_badge('warn')} known limitation, reported but not
enforced · {_badge('error')} the check could not run.</p></details>""")

    i_latest = entries.index(latest)
    for cat, title, intro, cat_names in cats:
        st, txt = score(cat_names)
        out.append(f"<section class='block' id='checks-{cat}'><div class='block-head'><h3>{escape(title)}</h3>"
                   f"{_chip(st, txt + ' pass')}</div><p class='muted'>{escape(intro)}</p>"
                   f"<div class='rows-head'><span>Check</span><span>Newest · {_version_name(i_latest)}</span>"
                   f"<span>{'Earlier versions' if older else ''}</span></div>")
        for name in cat_names:
            human, question, how = CHECK_INFO.get(name, (name, "", ""))
            r_latest = by_entry[latest.label].get(name)
            pills = "".join(_pill(entries.index(e), (by_entry[e.label].get(name) or {}).get("status"),
                                  _check_detail(by_entry[e.label][name]) if name in by_entry[e.label] else "")
                            for e in older)
            out.append(f"<details class='row'><summary>"
                       f"<div><div class='check-name'>{escape(human)}</div><div class='check-q'>{escape(question)}</div></div>"
                       f"<div><div class='col-label'>Newest · {_version_name(i_latest)}</div>{_check_cell(r_latest)}</div>"
                       f"<div>{'<div class=col-label>Earlier versions</div><div class=pills>' + pills + '</div>' if older else ''}</div>"
                       f"</summary><div class='row-body'><p><b>How it is measured.</b> {escape(how)}</p>")
            others = [x for x in (r_latest or {}).get("metrics", [])[1:] if x["value"] is not None]
            if others:
                out.append(f"<p><b>Other measurements ({_version_name(i_latest)}).</b></p><ul>" + "".join(
                    f"<li>{escape(_reading(x, r_latest.get('context', {})))}"
                    + (f" — limit {_plain(x['threshold'])} ({_sci(x['threshold'])})" if x["threshold"] is not None and x["threshold"] < 1
                       else f" — limit {svg.fmt(x['threshold'])}" if x["threshold"] is not None else "")
                    + f" {'✓' if x['ok'] else '✗'}</li>" for x in others) + "</ul>")
            if older:
                out.append("<table class='mini'><tr><th>Version</th><th>Result</th></tr>")
                for e in with_tests:
                    r = by_entry[e.label].get(name)
                    out.append(f"<tr><td>{_col_header(entries.index(e), e)}</td><td>{_check_cell(r)}</td></tr>")
                out.append("</table>")
            out.append("</div></details>")
        out.append("</section>")
    out.append("<p class='muted'>All limits come from <code>config/thresholds.json</code>; raw values are in "
               "<code>results/&lt;commit&gt;/&lt;build&gt;/correctness.json</code>.</p>")
    return "\n".join(out)


# ----------------------------------------------------------------------------- unit tests

def _section_unit(entries: list[Entry], with_unit: list[Entry]) -> str:
    lead = ("Some behaviour is internal and invisible to the engine: the softening length and the collision pairs. "
            "These tests compile the module's own source files and check them directly. Each test simply passes or fails.")
    if not with_unit:
        return _page_head("Unit tests", lead + " No results stored yet: run <code>./physics-qa unit</code> "
                          "(or <code>./physics-qa run</code>) to add them.")
    latest = with_unit[-1]
    older = with_unit[:-1]
    suites: list[str] = []
    for e in with_unit:
        for suite in e.unit["suites"]:
            if suite["name"] not in suites:
                suites.append(suite["name"])
    by_entry = {e.label: {(s["name"], t["name"]): t for s in e.unit["suites"] for t in s["tests"]} for e in with_unit}

    def tests_of(suite):
        names: list[str] = []
        for e in reversed(with_unit):
            for s in e.unit["suites"]:
                if s["name"] == suite:
                    names += [t["name"] for t in s["tests"] if t["name"] not in names]
        return names

    def latest_status(suite, name):
        t = by_entry[latest.label].get((suite, name))
        return t["status"] if t else None

    def score(suite):
        names = [n for n in tests_of(suite) if latest_status(suite, n) is not None]
        passed = sum(latest_status(suite, n) == "pass" for n in names)
        return passed, len(names)

    subnav = []
    for suite in suites:
        passed, total = score(suite)
        subnav.append((f"unit-{_slug(suite)}", suite, " " + _chip("pass" if passed == total else "fail", f"{passed}/{total}")))
    out = [_page_head("Unit tests", lead, subnav)]
    missing = [e for e in entries if not e.unit]
    if missing:
        out.append(f"<p class='muted'>Not run for {', '.join(_version_name(entries.index(e)) for e in missing)}"
                   f"{' (unit tests were added later)' if len(missing) == len(entries) - len(with_unit) else ''}.</p>")
    for e in with_unit:
        if e.unit["summary"]["error"]:
            out.append(f"<div class='notice' role='alert'><span aria-hidden='true'>⚠</span><p>The unit test program of "
                       f"{_version_name(entries.index(e))} stopped (exit code {e.unit.get('returncode')}) before "
                       f"finishing its report; some tests may be missing.</p></div>")

    tiles = []
    for suite in suites:
        passed, total = score(suite)
        question = UNIT_SUITES.get(suite, (suite, ""))[0]
        tiles.append(f"<a class='tile' href='#unit-{_slug(suite)}'><span class='label'>{escape(suite)}</span>"
                     f"<span class='value'>{passed} / {total} pass</span>{_progress(passed, total)}"
                     f"<span class='sub'>{escape(question)}</span></a>")
    out.append("<div class='tiles'>" + "".join(tiles) + "</div>")

    def item(suite, name, show_message):
        t = by_entry[latest.label].get((suite, name))
        pills = ("<span class='pills'>" + "".join(
            _pill(entries.index(e), (by_entry[e.label].get((suite, name)) or {}).get("status")) for e in older)
            + "</span>") if older else ""
        status = latest_status(suite, name) or "skip"
        line = (f"<div class='test-line'>{_badge(status)}<span class='name'>{escape(_humanize(name))}"
                f"<br><code>{escape(suite)}.{escape(name)}</code></span>{pills}</div>")
        if show_message and t and t.get("message"):
            line += (f"<details class='fail-msg'><summary>Why it failed</summary>"
                     f"<pre class='failure'>{escape(t['message'])}</pre></details>")
        return f"<li>{line}</li>"

    for suite in suites:
        question, intro = UNIT_SUITES.get(suite, (suite, ""))
        names = tests_of(suite)
        failing = [n for n in names if latest_status(suite, n) == "fail"]
        rest = [n for n in names if n not in failing]
        passed, total = score(suite)
        out.append(f"<section class='block' id='unit-{_slug(suite)}'><div class='block-head'><h3>{escape(suite)} — "
                   f"{escape(question)}</h3>{_chip('pass' if passed == total else 'fail', f'{passed}/{total} pass')}</div>"
                   f"<p class='muted'>{escape(intro)}</p>")
        if failing:
            out.append(f"<p><b>{len(failing)} failing in {_version_name(entries.index(latest))}:</b></p>"
                       "<ul class='test-list'>" + "".join(item(suite, n, True) for n in failing) + "</ul>")
        if rest:
            label = f"{len(rest)} passing test{'s' if len(rest) > 1 else ''}" if all(
                latest_status(suite, n) == "pass" for n in rest) else f"{len(rest)} other tests"
            out.append(f"<details class='more'><summary>✓ {label}</summary>"
                       "<ul class='test-list'>" + "".join(item(suite, n, False) for n in rest) + "</ul></details>")
        out.append("</section>")
    out.append("<p class='muted'>Sources in <code>tests/unit/</code> (GoogleTest); raw results are in "
               "<code>results/&lt;commit&gt;/&lt;build&gt;/unit.json</code>.</p>")
    return "\n".join(out)


CHART_SPECS = [
    # (check, series key, metric with the limit, group, title, what the curve is, what a healthy curve looks like)
    ("kepler_circular", "position_error", "max_rel_position_error", "Orbits compared with the exact solution",
     "Circular orbit: position error over one year",
     "Distance between the simulated planet and its exact position, as the year goes by.",
     "A slow, smooth rise that stays under the limit. The simulated planet runs very slightly ahead of or behind "
     "the exact one, so the gap grows with time."),
    ("kepler_eccentric", "position_error", "max_rel_position_error", "Orbits compared with the exact solution",
     "Elongated orbit: position error over one orbit",
     "Same reading for a strongly elongated orbit.",
     "Larger than the circular case, still under the limit. The planet moves much faster near the star, where a "
     "fixed two-hour step is coarser."),
    ("kepler_eccentric", "energy_drift", "max_rel_energy_drift", "Orbits compared with the exact solution",
     "Elongated orbit: energy error over one orbit",
     "How much the total energy differs from its starting value.",
     "A flat plateau that drops back near the end: the error appears near the star and disappears again when the "
     "planet returns to its starting point. It oscillates instead of accumulating."),
    ("planetary_reference", "position_error", "max_rel_position_error", "Many bodies at once",
     "Eleven planets: gap to the reference simulation",
     "Largest gap between the module and the high-precision reference, over about half a year.",
     "A gentle rise that levels off well under the limit."),
    ("conservation", "energy_drift", "max_rel_energy_drift", "Many bodies at once",
     "200-body cluster: energy error",
     "Change of the total energy of a random 200-body cluster over time.",
     "Bumps are normal: they happen when two bodies pass close to each other. What matters is staying under the "
     "limit and returning to low values afterwards."),
    ("conservation", "momentum_drift", "max_rel_momentum_drift", "Many bodies at once",
     "200-body cluster: momentum error",
     "Change of the total momentum of the cluster over time.",
     "A flat line at the very bottom (around 10⁻¹⁶, the precision of a double). Gravity pulls two bodies equally "
     "and oppositely, so momentum cannot change; anything larger means the forces are not symmetric."),
    ("energy_bounded", "energy_drift", None, "Integrator properties",
     "Ten orbits: energy error stays bounded",
     "Energy error over ten consecutive elongated orbits.",
     "The same pattern repeated ten times with the same height: the error never climbs from one orbit to the next. "
     "This is what a symplectic integrator looks like. The sharp dips are moments where the error crosses zero, "
     "which a logarithmic axis exaggerates."),
    ("convergence_order", "error_vs_dt", None, "Integrator properties",
     "Integrator order: error versus time step",
     "End-of-run position error for several time steps (here the horizontal axis is the time step, not time).",
     "A straight line going up two grid lines for every one grid line to the right: halving the step divides the "
     "error by four, as a second-order integrator must."),
    ("padding_sizes", "error_vs_size", "max_rel_position_error", "Integrator properties",
     "Error for every scene size around SIMD boundaries",
     "Gap to the reference after 20 steps, for scene sizes around the SIMD batch size (horizontal axis: number of bodies).",
     "A smooth staircase far below the limit. A sudden jump at particular sizes would reveal a bug in how the last, "
     "partial batch is handled."),
]

X_WORDS = {"orbits": "orbits completed", "days": "simulated days", "dt (s)": "time step (seconds)", "bodies": "number of bodies"}
Y_WORDS = {"max |Δr| / a": "position error (fraction of orbit radius)",
           "|ΔE| / |E0|": "energy error (fraction of initial energy)",
           "max |Δr| / RMS radius": "position error (fraction of system size)",
           "max |Δv| / RMS speed": "speed error (fraction of typical speed)",
           "|ΔP| / Σm|v|": "momentum error (relative)", "|Δr| / a": "position error (fraction of orbit radius)"}


def _chart_guide() -> str:
    return """<details class='explain' open><summary>How to read these charts</summary>
<p>Every chart shows an <i>error</i>: the gap between what the module computed and what is exactly right. Errors are never
zero in a simulation; the question is whether they stay small.</p>
<div class='guide'>
<div><b>Down is good.</b> The lower a curve, the more accurate the module. Versions with identical numerics overlap, so
you may see a single colour.</div>
<div><b>The red zone is failure.</b> Above the dashed red line is "too much error". The worst point of the newest
version is circled.</div>
<div><b>Each grid line is 10×.</b> The vertical scale is logarithmic: 10⁻⁵ is 0.001%. Flat at 10⁻¹⁶ is the precision
limit of the computer: the best possible result.</div>
<div><b>Left to right is time</b> (orbits or simulated days), unless the chart title says time step or number of bodies.</div>
</div></details>"""


def _section_charts(entries: list[Entry], with_tests: list[Entry]) -> str:
    if not with_tests:
        return _page_head("Error charts", "No correctness data stored.")
    by_entry = {e.label: {r["name"]: r for r in e.correctness["results"]} for e in with_tests}
    latest = with_tests[-1]
    groups: list[str] = []
    for spec in CHART_SPECS:
        if spec[3] not in groups:
            groups.append(spec[3])
    out = [_page_head("Error charts", "The physics checks seen as curves over time: not only whether a version passed, "
                      "but how its error builds up, which is what reveals a subtle change between versions.",
                      [(f"charts-{_slug(g)}", g, "") for g in groups]),
           _chart_guide()]
    current_group = None
    for check, key, metric, group, title, what, healthy in CHART_SPECS:
        series, threshold, meta = [], None, None
        for e in with_tests:
            r = by_entry[e.label].get(check)
            if not r or key not in r.get("series", {}):
                continue
            s = r["series"][key]
            meta = s
            i = entries.index(e)
            series.append({"name": _series_name(i, e), "slot": i, "x": s["x"], "y": s["y"]})
            if metric:
                m = next((m for m in r["metrics"] if m["name"] == metric), None)
                if m and m["threshold"] is not None:
                    threshold = m["threshold"]
        if not series:
            continue
        if group != current_group:
            if current_group is not None:
                out.append("</div></section>")
            out.append(f"<section id='charts-{_slug(group)}' style='margin-bottom:22px'><h3 style='margin:4px 0 10px'>"
                       f"{escape(group)}</h3><div class='charts'>")
            current_group = group
        # Worst point of the newest version, and its verdict line.
        annotate, verdict = [], ""
        r_latest = by_entry[latest.label].get(check)
        if r_latest and key in r_latest.get("series", {}):
            xs, ys = r_latest["series"][key]["x"], r_latest["series"][key]["y"]
            finite = [(x, y) for x, y in zip(xs, ys) if y is not None and math.isfinite(y) and y > 0]
            if finite:
                wx, wy = max(finite, key=lambda p: p[1])
                unit = X_WORDS.get(meta["xlabel"], meta["xlabel"])
                annotate = [(wx, wy, f"worst: {_plain(wy)}")]
                where = f"at {svg.fmt(wx)} {unit}"
                name = _version_name(entries.index(latest))
                if threshold:
                    ratio = wy / threshold
                    verdict = (f"{_badge(r_latest['status'])} {name}: worst error {_pct(wy)} {where} — "
                               + (f"{ratio:,.0f}× over the limit." if ratio > 1 else
                                  f"{ratio * 100:.0f}% of the limit." if ratio >= 0.001 else "far below the limit."))
                else:
                    verdict = f"{_badge(r_latest['status'])} {name}: largest value {_pct(wy)} {where}."
        out.append("<figure class='chart-card'>")
        if verdict:
            out.append(f"<div class='chart-verdict'>{verdict}</div>")
        out.append(svg.line_chart(series, title, X_WORDS.get(meta["xlabel"], meta["xlabel"]),
                                  Y_WORDS.get(meta["ylabel"], meta["ylabel"]), log_x=meta.get("log_x", False),
                                  log_y=meta.get("log_y", True), threshold=threshold, annotate=annotate,
                                  width=560, height=320))
        out.append(f"<details><summary>What this chart shows</summary><p>{escape(what)}</p>"
                   f"<p><b>A healthy curve:</b> {escape(healthy)}</p></details></figure>")
    if current_group is not None:
        out.append("</div></section>")
    return "\n".join(out)


NAV_JS = r"""
<script>
(function () {
  var pages = Array.prototype.slice.call(document.querySelectorAll('.page'));
  var links = Array.prototype.slice.call(document.querySelectorAll('nav.tabs a'));
  function show(id) {
    var target = id ? document.getElementById(id) : null;
    var page = target ? (target.classList.contains('page') ? target : target.closest('.page')) : null;
    if (!page) { page = pages[0]; target = null; }
    pages.forEach(function (p) { p.classList.toggle('active', p === page); });
    links.forEach(function (a) {
      var on = a.getAttribute('href') === '#' + page.id;
      a.classList.toggle('active', on);
      if (on) { a.setAttribute('aria-current', 'page'); } else { a.removeAttribute('aria-current'); }
    });
    setTimeout(function () {
      if (target && target !== page) { target.scrollIntoView(); } else { window.scrollTo(0, 0); }
    }, 0);
  }
  window.addEventListener('hashchange', function () { show(location.hash.slice(1)); });
  window.addEventListener('beforeprint', function () {
    document.querySelectorAll('details').forEach(function (d) { d.open = true; });
  });
  show(location.hash.slice(1));
})();
</script>"""


def _section_method(latest: Entry) -> str:
    c = latest.correctness or {}
    ref = c.get("reference", {})
    out = [_page_head("Method & glossary", "How the numbers in this report were produced, and the words it uses."),
           "<section class='block'><h3>Method</h3>",
           f"<p>All checks use a time step of {c.get('dt', '?')} s, the value the engine uses, and G = {c.get('G', '?')} "
           f"km³·kg⁻¹·s⁻² (kilometres, kilograms, seconds). Masses go through the same float mantissa × 10ⁿ encoding "
           f"as in the engine, so the exact solutions use exactly the masses the module sees.</p>"
           f"<p>The reference simulation is a classical RK4 integrator in 80-bit floating point with "
           f"{ref.get('substeps', '?')} sub-steps per frame and no softening; its own error is several orders of "
           f"magnitude below what is measured here.</p></section>",
           "<section class='block'><h3 style='margin-bottom:12px'>Glossary</h3><dl class='glossary'>"]
    for term, definition in GLOSSARY:
        out.append(f"<dt>{escape(term)}</dt><dd>{escape(definition)}</dd>")
    out.append("</dl></section>")
    return "\n".join(out)


PAGES = [("overview", "Overview"), ("speed", "Speed"), ("checks", "Physics checks"), ("unit", "Unit tests"),
         ("charts", "Error charts"), ("method", "Method & glossary")]


def _tab_chips(entries: list[Entry], with_bench: list[Entry], with_tests: list[Entry], with_unit: list[Entry],
               plan: dict) -> dict[str, str]:
    """Status chip shown next to each tab, so the menu says where to look."""
    chips = {}
    if with_tests:
        c = with_tests[-1].correctness["summary"]
        bad = c["fail"] + c["error"]
        chips["checks"] = _chip("fail", f"{bad} failing") if bad else _chip("pass", f"{c['pass']}")
    if with_unit:
        u = with_unit[-1].unit["summary"]
        chips["unit"] = _chip("fail", f"{u['fail']} failing") if not _ok(u) else _chip("pass", f"{u['pass']}")
    verdict = _speed_verdict(_speed_rows(with_bench, plan))
    if verdict:
        chips["speed"] = _chip(*{"fail": ("fail", "slower"), "pass": ("pass", "faster")}.get(verdict[0], ("skip", "stable")))
    return chips


def render(entries: list[Entry], plan: dict) -> str:
    """One self-contained file; the tabs switch between pages (hash links, works offline and when printed)."""
    entries = list(entries)
    with_bench = [e for e in entries if e.benchmark and e.benchmark.get("results")]
    with_tests = [e for e in entries if e.correctness]
    with_unit = [e for e in entries if e.unit]
    latest = entries[-1]
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    chips = _tab_chips(entries, with_bench, with_tests, with_unit, plan)
    header = ("<header class='top'><div class='top-inner'><a class='brand' href='#overview'>Physics QA</a>"
              f"<span class='brand-sub'>{len(entries)} version{'s' if len(entries) > 1 else ''} · newest "
              f"{_version_name(len(entries) - 1)} <code>{escape(latest.meta.get('commit', latest.label)[:7])}</code> · "
              f"{escape(latest.build_type)} build · generated {now}</span></div>"
              "<nav class='tabs' aria-label='Report pages'>" + "".join(
                  f"<a href='#{pid}'>{escape(title)}{chips.get(pid, '')}</a>" for pid, title in PAGES) + "</nav></header>")
    pages = {
        "overview": _section_overview(entries, with_bench, with_unit, plan),
        "speed": _section_speed(entries, with_bench, plan),
        "checks": _section_checks(entries, with_tests),
        "unit": _section_unit(entries, with_unit),
        "charts": _section_charts(entries, with_tests),
        "method": _section_method(entries[-1]),
    }
    parts = ["<!doctype html><html lang='en'><head><meta charset='utf-8'>",
             "<meta name='viewport' content='width=device-width, initial-scale=1'>",
             "<title>Physics QA report</title>", f"<style>{CSS}</style>",
             "<noscript><style>.page{display:block;margin-bottom:48px}</style></noscript></head><body>",
             header, "<main>"]
    for pid, _ in PAGES:
        parts.append(f"<section class='page' id='{pid}'>{pages[pid]}</section>")
    parts.append("</main>" + NAV_JS + "</body></html>")
    return "\n".join(parts)
