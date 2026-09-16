"""Self-contained HTML report (inline CSS + SVG, no external assets).

Written for readers who are not physicists: every check has a plain-language
name and purpose, every number is translated into a percentage or a distance,
and each chart carries a one-sentence explanation of what it shows.
"""

from __future__ import annotations

import math
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
  --surface-1:#fcfcfb; --surface-2:#f1f1ee; --surface-3:#e7e7e2; --border:#dcdcd6; --grid:#e6e6e1;
  --text-primary:#1a1a19; --text-secondary:#4d4d49; --text-muted:#7a7a74;
  --series-1:#2a78d6; --series-2:#eb6834; --series-3:#1baf7a; --series-4:#eda100;
  --series-5:#e87ba4; --series-6:#008300; --series-7:#4a3aa7; --series-8:#e34948;
  --good:#0ca30c; --warning:#fab219; --serious:#ec835a; --critical:#d03b3b; --info:#2a78d6; }
@media (prefers-color-scheme: dark) { :root:not([data-theme="light"]) {
  --surface-1:#1a1a19; --surface-2:#242422; --surface-3:#30302d; --border:#3a3a37; --grid:#2f2f2c;
  --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8d8d86;
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70; --series-4:#c98500;
  --series-5:#d55181; --series-6:#008300; --series-7:#9085e9; --series-8:#e66767; --info:#3987e5; } }
:root[data-theme="dark"] {
  --surface-1:#1a1a19; --surface-2:#242422; --surface-3:#30302d; --border:#3a3a37; --grid:#2f2f2c;
  --text-primary:#ffffff; --text-secondary:#c3c2b7; --text-muted:#8d8d86;
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70; --series-4:#c98500;
  --series-5:#d55181; --series-6:#008300; --series-7:#9085e9; --series-8:#e66767; --info:#3987e5; }
* { box-sizing: border-box; }
body { margin:0; padding:24px 16px 64px; background:var(--surface-1); color:var(--text-primary);
  font: 15px/1.5 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 1100px; margin: 0 auto; }
h1 { font-size: 26px; margin: 0 0 4px; }
h2 { font-size: 21px; margin: 44px 0 8px; border-bottom:1px solid var(--border); padding-bottom:6px; }
h3 { font-size: 17px; margin: 28px 0 6px; }
p, li { color: var(--text-secondary); } .muted { color: var(--text-muted); font-size: 13px; }
.lead { font-size: 15px; margin: 0 0 12px; }
code { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 13px; background: var(--surface-2); padding: 1px 4px; border-radius: 3px; }
table { border-collapse: collapse; width: 100%; margin: 8px 0 12px; font-size: 14px; }
th, td { text-align: left; padding: 8px 8px; border-bottom: 1px solid var(--border); vertical-align: top; }
th { color: var(--text-secondary); font-weight: 600; background: var(--surface-2); }
td.num, th.num { text-align: right; font-variant-numeric: tabular-nums; white-space: nowrap; }
.wrap { overflow-x: auto; }
.badge { display:inline-block; padding:1px 8px; border-radius:10px; font-size:12px; font-weight:600; color:#fff; white-space:nowrap; }
.badge.good { background: var(--good); } .badge.critical { background: var(--critical); }
.badge.warning { background: var(--warning); color:#1a1a19; } .badge.serious { background: var(--serious); color:#1a1a19; }
.badge.muted { background: var(--text-muted); }
.delta-up { color: var(--critical); font-weight:600; } .delta-down { color: var(--good); font-weight:600; }
.cards { display:flex; flex-wrap:wrap; gap:12px; margin: 12px 0; }
.card { flex: 1 1 240px; background: var(--surface-2); border-radius: 8px; padding: 12px 16px; }
.card .label { font-size: 12px; color: var(--text-muted); text-transform: uppercase; letter-spacing: .04em; }
.card .value { font-size: 22px; font-weight: 600; margin: 2px 0; } .card .sub { font-size: 13px; color: var(--text-secondary); }
.howto { background: var(--surface-2); border-left: 4px solid var(--info); border-radius: 6px; padding: 12px 16px; margin: 16px 0; }
.howto p { margin: 4px 0; }
.charts { display:grid; grid-template-columns: repeat(auto-fit, minmax(480px, 1fr)); gap: 24px; }
figure { margin: 0; } figcaption { font-size: 13px; color: var(--text-secondary); margin: 4px 0 0; }
svg.chart { width: 100%; height: auto; display:block; }
.chart-title { font-size: 14px; font-weight: 600; fill: var(--text-primary); }
.tick { font-size: 11px; fill: var(--text-muted); } .axis-label { font-size: 12px; fill: var(--text-secondary); }
.grid { stroke: var(--grid); stroke-width: 1; } .axis { stroke: var(--border); stroke-width: 1; }
.line { fill: none; stroke-width: 2; stroke-linejoin: round; } .marker { stroke: var(--surface-1); stroke-width: 2; }
.marker:hover { r: 6; } .bar:hover { opacity: .8; }
.threshold { stroke: var(--critical); stroke-width: 1.5; stroke-dasharray: 6 4; }
.direct-label { font-size: 12px; fill: var(--text-secondary); }
.legend { display:flex; flex-wrap:wrap; gap: 4px 16px; font-size: 13px; color: var(--text-secondary); margin: 4px 0 8px; }
.swatch { display:inline-block; width: 12px; height: 12px; border-radius: 3px; margin-right: 6px; vertical-align: -1px; }
details { margin: 6px 0; } summary { cursor: pointer; color: var(--text-secondary); }
.chart-empty { color: var(--text-muted); font-size: 13px; padding: 12px; }
nav.topbar { position: sticky; top: 0; z-index: 10; background: var(--surface-1); border-bottom: 1px solid var(--border);
  margin: 0 -16px 8px; padding: 8px 16px; display: flex; flex-wrap: wrap; align-items: center; gap: 4px 6px; }
nav.topbar .brand { font-weight: 700; margin-right: 12px; }
nav.topbar a { color: var(--text-secondary); text-decoration: none; padding: 6px 12px; border-radius: 16px; font-size: 14px; }
nav.topbar a:hover { background: var(--surface-2); }
nav.topbar a.active { background: var(--info); color: #fff; }
.page { display: none; scroll-margin-top: 72px; } .page.active { display: block; }
.page h2:first-child { margin-top: 12px; }
.chart-block { background: var(--surface-2); border-radius: 8px; padding: 12px 14px; }
.chart-verdict { font-size: 14px; margin: 6px 0 2px; }
.over-zone { fill: var(--critical); opacity: .07; }
.threshold-label { font-size: 11px; fill: var(--critical); }
.callout { font-size: 12px; font-weight: 600; fill: var(--text-primary); }
.callout-ring { fill: none; stroke: var(--text-primary); stroke-width: 1.5; }
.guide { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; margin: 12px 0 20px; }
.guide div { background: var(--surface-2); border-radius: 8px; padding: 10px 14px; font-size: 14px; color: var(--text-secondary); }
.guide b { color: var(--text-primary); }
.check-name { font-weight: 600; } .check-q { color: var(--text-secondary); font-size: 13px; }
.reading { margin-top: 4px; font-size: 13px; }
.meter { position: relative; height: 6px; border-radius: 3px; background: var(--surface-3); margin: 6px 0 2px; max-width: 220px; overflow: hidden; }
.meter > span { position:absolute; left:0; top:0; bottom:0; border-radius: 3px; background: var(--good); }
.meter.over > span { background: var(--critical); } .meter.warn > span { background: var(--warning); }
.meter-label { font-size: 12px; color: var(--text-muted); }
dl { display: grid; grid-template-columns: max-content 1fr; gap: 6px 16px; } dt { font-weight: 600; } dd { margin: 0; color: var(--text-secondary); }
@media (max-width: 640px) { dl { grid-template-columns: 1fr; } .charts { grid-template-columns: 1fr; } }
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

def _section_howto(entries: list[Entry]) -> str:
    n = len(entries)
    return f"""
<div class='howto'>
<p><b>How to read this report.</b> It compares {n} version{'s' if n > 1 else ''} of the physics module, numbered
v1 (oldest) to v{n} (newest). Each version was built from its git commit and driven exactly like the game engine
does, through the public interface only.</p>
<p><b>Speed</b> is how long one physics step takes: lower is better, and the change compared with the previous
version tells you whether the code got faster or slower.</p>
<p><b>Accuracy and correctness</b> compare the simulated motion with exact solutions and physical laws. Every check
reports a measured error next to the <i>limit</i> it must stay under; a green bar shows how much of the allowed
error was used. {_badge('pass')} means within the limit, {_badge('fail')} means outside it, {_badge('warn')} marks a
known limitation that is reported but not enforced.</p>
</div>"""


def _section_glance(entries: list[Entry], with_bench: list[Entry], plan: dict) -> str:
    latest = entries[-1]
    i_latest = len(entries) - 1
    cards = []
    c = latest.correctness["summary"] if latest.correctness else None
    if c:
        ok = not (c["fail"] or c["error"])
        cards.append(f"<div class='card'><div class='label'>Correctness of {_version_name(i_latest)}</div>"
                     f"<div class='value'>{_badge('pass' if ok else 'fail', 'All required checks pass' if ok else 'Some checks fail')}</div>"
                     f"<div class='sub'>{c['pass']} passed · {c['fail']} failed · {c['warn']} known limitation(s) · {c['error']} error(s)</div></div>")
        kep = _find(latest, "kepler_circular")
        m = _primary(kep)
        if m and m["value"] is not None and kep.get("context", {}).get("scale_km"):
            ctx = kep["context"]
            cards.append(f"<div class='card'><div class='label'>Accuracy headline</div>"
                         f"<div class='value'>{_km(m['value'] * ctx['scale_km'])} off</div>"
                         f"<div class='sub'>position of an Earth-like planet after one simulated year: "
                         f"{_pct(m['value'])} of the orbit radius. Energy conserved to {_pct(kep['metrics'][1]['value'])}.</div></div>")
    if latest in with_bench:
        res = latest.benchmark["results"]
        big = max(res, key=lambda r: r["bodies"])
        mid = next((r for r in res if r["bodies"] == 1000), None)
        sub = f"{big['bodies']:,} bodies per step ({_steps_per_s(big['update_ms']['median'])})"
        if mid and mid is not big:
            sub += f" · 1 000 bodies: {_ms(mid['update_ms']['median'])} ms ({_steps_per_s(mid['update_ms']['median'])})"
        cards.append(f"<div class='card'><div class='label'>Speed of {_version_name(i_latest)}</div>"
                     f"<div class='value'>{_ms(big['update_ms']['median'])} ms per step</div><div class='sub'>{sub}</div></div>")
        if len(with_bench) >= 2:
            prev = with_bench[-2]
            rows = bench.compare(prev.benchmark, latest.benchmark, plan["regression_threshold_pct"],
                                 plan["improvement_threshold_pct"], plan.get("min_ms_for_verdict", 0.0))
            judged = [r for r in rows if r["verdict"] != "noise"]
            regs = [r for r in judged if r["verdict"] == "regression"]
            imps = [r for r in judged if r["verdict"] == "improvement"]
            if regs:
                worst = max(regs, key=lambda r: r["delta_pct"])
                verdict = _badge("fail", f"Slower: +{worst['delta_pct']:.0f}% at {worst['bodies']:,} bodies")
            elif imps:
                best = min(imps, key=lambda r: r["delta_pct"])
                verdict = _badge("pass", f"Faster: {best['delta_pct']:.0f}% at {best['bodies']:,} bodies")
            else:
                verdict = _badge("pass", "No significant change")
            big_row = next((r for r in rows if r["bodies"] == big["bodies"]), None)
            sub = (f"vs {_version_name(entries.index(prev))}; at {big['bodies']:,} bodies: {big_row['delta_pct']:+.1f}%"
                   if big_row else f"vs {_version_name(entries.index(prev))}")
            cards.append(f"<div class='card'><div class='label'>Speed change</div><div class='value'>{verdict}</div>"
                         f"<div class='sub'>{sub}. A change counts only above {plan['regression_threshold_pct']:g}%.</div></div>")
    return "<h2>At a glance</h2><div class='cards'>" + "".join(cards) + "</div>"


def _section_versions(entries: list[Entry]) -> str:
    out = ["<h2>Versions compared</h2>",
           "<p class='lead'>Oldest first. The label is the git commit of the physics module; the machine column "
           "matters because timings are only comparable on the same hardware.</p>",
           "<div class='wrap'><table><tr><th>Version</th><th>Commit</th><th>Date</th><th>Checks</th><th>Machine</th></tr>"]
    for i, e in enumerate(entries):
        m = e.meta
        c = e.correctness["summary"] if e.correctness else None
        tests = (f"{_badge('pass' if not (c['fail'] or c['error']) else 'fail')} "
                 f"{c['pass']} pass · {c['fail']} fail · {c['warn']} known limitation") if c else "–"
        machine = (f"{escape(m.get('cpu', '?'))}<br><span class='muted'>{escape(m.get('compiler', ''))} · "
                   f"{m.get('build_type', '')} build · CPU governor {escape(m.get('governor', '?'))}</span>")
        out.append(f"<tr><td><span class='swatch' style='background:var(--series-{i % 8 + 1})'></span><b>{_version_name(i)}</b>"
                   f"{' <span class=muted>(uncommitted changes)</span>' if m.get('dirty') else ''}</td>"
                   f"<td>{escape(m.get('subject', ''))}<br><code>{escape(m.get('commit', '')[:12])}</code></td>"
                   f"<td>{escape(m.get('commit_date', '')[:10])}</td><td>{tests}</td><td>{machine}</td></tr>")
    out.append("</table></div>")
    return "\n".join(out)


def _section_speed(entries: list[Entry], with_bench: list[Entry], plan: dict) -> str:
    out = ["<h2>Speed</h2>"]
    if not with_bench:
        out.append("<p>No benchmark data stored.</p>")
        return "\n".join(out)
    out.append("<p class='lead'>How long the module needs to compute one physics step, for scenes of growing size. "
               "The scenes are random clusters generated with a fixed seed, so every version computes exactly the "
               "same thing. Each measurement is the median over several fresh runs after a warm-up.</p>")
    counts = sorted({r["bodies"] for e in with_bench for r in e.benchmark["results"]})
    latest, previous = with_bench[-1], (with_bench[-2] if len(with_bench) >= 2 else None)
    rows = bench.compare(previous.benchmark, latest.benchmark, plan["regression_threshold_pct"],
                         plan["improvement_threshold_pct"], plan.get("min_ms_for_verdict", 0.0)) if previous else []
    governors = {e.meta.get("governor", "unknown") for e in with_bench}
    if governors - {"performance"}:
        out.append(f"<p>{_badge('warn', 'Measurement noise')} The CPU frequency governor was "
                   f"<code>{escape(', '.join(sorted(governors)))}</code> during at least one run. Timings can then vary by "
                   f"several percent between runs; set it to <code>performance</code> for reliable comparisons.</p>")

    line_series = [{"name": _series_name(entries.index(e), e), "slot": entries.index(e),
                    "x": [r["bodies"] for r in e.benchmark["results"]],
                    "y": [r["update_ms"]["median"] for r in e.benchmark["results"]]} for e in with_bench]
    out.append("<div class='charts'><figure>")
    out.append(svg.line_chart(line_series, "Time per physics step", "number of bodies", "milliseconds per step",
                              log_x=True, log_y=True))
    out.append("<figcaption>Lower is better. Both axes are logarithmic (each grid line is 10× the previous one), "
               "so the straight line means the cost grows like the square of the number of bodies, as expected for "
               "an all-pairs gravity computation. Versions with identical performance overlap.</figcaption></figure>")
    if rows:
        out.append("<figure>")
        out.append(svg.bar_chart([f"{r['bodies']:,}" for r in rows],
                                 [{"name": f"{_version_name(entries.index(latest))} vs {_version_name(entries.index(previous))}",
                                   "slot": entries.index(latest), "values": [r["delta_pct"] for r in rows]}],
                                 f"Change of {_version_name(entries.index(latest))} compared with {_version_name(entries.index(previous))}",
                                 "% slower (+) or faster (−)", threshold=plan["regression_threshold_pct"],
                                 threshold_label="regression limit", symmetric=True))
        out.append("<figcaption>Bars above zero mean the newest version is slower. A bar crossing the dashed line "
                   f"is a regression. Scenes that run in less than {plan.get('min_ms_for_verdict', 0)} ms per step are "
                   "shown but not judged: at that scale the operating system's scheduling noise is larger than any "
                   "real change.</figcaption></figure>")
    out.append("</div>")

    out.append("<h3>Detailed timings</h3><p class='muted'>Median time per step in milliseconds, with the spread (standard "
               "deviation) below it, and the number of steps per second that this allows. Hover a cell for the 95th "
               "percentile and the best step.</p>")
    out.append("<div class='wrap'><table><tr><th class='num'>bodies</th>")
    for e in with_bench:
        out.append(f"<th class='num'>{_col_header(entries.index(e), e)}</th>")
    out.append(f"<th class='num'>change<br><span class='muted'>{_version_name(entries.index(latest))} vs "
               f"{_version_name(entries.index(previous)) if previous else '–'}</span></th></tr>")
    delta_by_n = {r["bodies"]: r for r in rows}
    for n in counts:
        out.append(f"<tr><td class='num'>{n:,}</td>")
        for e in with_bench:
            r = next((r for r in e.benchmark["results"] if r["bodies"] == n), None)
            if r:
                u = r["update_ms"]
                out.append(f"<td class='num' title='95th percentile {_ms(u['p95'])} ms · best {_ms(u['min'])} ms · "
                           f"{u['samples']} timed steps'>{_ms(u['median'])} ms<br><span class='muted'>± {_ms(u['stdev'])} · "
                           f"{_steps_per_s(u['median'])}</span></td>")
            else:
                out.append("<td class='num'>–</td>")
        out.append(_delta_cell(delta_by_n.get(n)) + "</tr>")
    out.append("</table></div>")
    cfg = latest.benchmark.get("config", {})
    out.append(f"<p class='muted'>Measurement plan: {cfg.get('repeats')} independent runs × ({cfg.get('warmup')} warm-up steps + "
               f"{cfg.get('frames')} timed steps; {escape(str(cfg.get('frames_override', {})))} for the largest scenes). "
               f"Time step {cfg.get('dt')} s. The timed part is update() alone; the full frame including data exchange "
               f"with the engine adds the copies of positions and velocities.</p>")
    return "\n".join(out)


def _section_checks(entries: list[Entry], with_tests: list[Entry]) -> str:
    out = ["<h2>Accuracy and correctness</h2>"]
    if not with_tests:
        out.append("<p>No correctness data stored.</p>")
        return "\n".join(out)
    out.append("<p class='lead'>Each check answers one question about the physics. The measured error is written in "
               "plain terms (a percentage, and where possible a distance) and shown against the limit it must not "
               "exceed. Versions are side by side so a change in behaviour stands out.</p>")
    names: list[str] = []
    for e in with_tests:
        for r in e.correctness["results"]:
            if r["name"] not in names:
                names.append(r["name"])
    by_entry = {e.label: {r["name"]: r for r in e.correctness["results"]} for e in with_tests}

    for cat, (title, intro) in CATEGORIES.items():
        cat_names = [n for n in names if any(by_entry[e.label].get(n, {}).get("category") == cat for e in with_tests)]
        if not cat_names:
            continue
        out.append(f"<h3>{escape(title)}</h3><p>{escape(intro)}</p><div class='wrap'><table><tr><th style='width:34%'>Check</th>")
        for e in with_tests:
            out.append(f"<th>{_col_header(entries.index(e), e)}</th>")
        out.append("</tr>")
        for name in cat_names:
            human, question, how = CHECK_INFO.get(name, (name, "", ""))
            out.append(f"<tr><td><div class='check-name'>{escape(human)}</div><div class='check-q'>{escape(question)}</div>"
                       f"<details><summary class='muted'>how it is measured</summary><div class='muted'>{escape(how)}</div></details></td>")
            for e in with_tests:
                r = by_entry[e.label].get(name)
                if not r:
                    out.append("<td>–</td>")
                    continue
                m = _primary(r)
                cell = _badge(r["status"])
                if r.get("error"):
                    cell += f"<div class='reading'>{escape(r['error'].splitlines()[0])}</div>"
                elif m:
                    cell += f"<div class='reading'>{escape(_reading(m, r.get('context', {})))}</div>{_meter(m)}"
                    others = [x for x in r["metrics"][1:] if x["value"] is not None]
                    if others:
                        cell += ("<details><summary class='muted'>more</summary>" + "".join(
                            f"<div class='muted'>{escape(_reading(x, r.get('context', {})))}"
                            + (f" — limit {_plain(x['threshold'])} ({_sci(x['threshold'])})" if x["threshold"] is not None and x["threshold"] < 1
                               else f" — limit {svg.fmt(x['threshold'])}" if x["threshold"] is not None else "")
                            + f" {'✓' if x['ok'] else '✗'}</div>" for x in others) + "</details>")
                out.append(f"<td>{cell}</td>")
            out.append("</tr>")
        out.append("</table></div>")
    out.append("<p class='muted'>All limits come from <code>config/thresholds.json</code>; the raw values are stored in "
               "<code>results/&lt;commit&gt;/&lt;build&gt;/correctness.json</code>.</p>")
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
    return """
<div class='howto'><p><b>How to read these charts.</b> Every chart shows an <i>error</i>: the difference between what the
module computed and what is exactly right. Errors are never zero in a simulation; the question is whether they stay small.</p></div>
<div class='guide'>
<div><b>Down is good.</b> The vertical axis is the size of the error. The lower a curve, the more accurate the module. The
curves of different versions overlap exactly when their numerics are identical, so you may see only one colour.</div>
<div><b>The red zone is failure.</b> The dashed red line is the limit from the configuration; the shaded area above it is
"too much error". A curve that enters the zone fails the check. The worst point of the newest version is circled with
its value.</div>
<div><b>The scale is logarithmic.</b> Each horizontal grid line is 10× the one below it: 10⁻⁵ is 0.001%, 10⁻⁷ is
0.00001%. A curve that looks flat at 10⁻¹⁶ is at the precision limit of the computer, which is the best possible result.</div>
<div><b>Left to right is time,</b> in orbits or simulated days, except in the two charts that say otherwise in their title
(time step and number of bodies).</div>
</div>"""


def _section_charts(entries: list[Entry], with_tests: list[Entry]) -> str:
    out = ["<h2>Error charts</h2>",
           "<p class='lead'>The checks of the previous page, seen as curves over time. They show not just whether a version "
           "passed, but how the error builds up, which is what reveals a subtle change between versions.</p>",
           _chart_guide()]
    by_entry = {e.label: {r["name"]: r for r in e.correctness["results"]} for e in with_tests}
    latest = with_tests[-1]
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
                out.append("</div>")
            out.append(f"<h3>{escape(group)}</h3><div class='charts'>")
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
                if threshold:
                    ratio = wy / threshold
                    status = _badge(r_latest["status"])
                    verdict = (f"{status} Newest version ({_version_name(entries.index(latest))}): worst error "
                               f"{_pct(wy)} {where}, " + (f"{ratio:,.0f}× over the limit." if ratio > 1 else
                                                          f"{ratio * 100:.0f}% of the limit." if ratio >= 0.001 else
                                                          "far below the limit."))
                else:
                    verdict = (f"{_badge(r_latest['status'])} Newest version ({_version_name(entries.index(latest))}): "
                               f"largest value {_pct(wy)} {where}.")
        out.append("<figure class='chart-block'>")
        out.append(svg.line_chart(series, title, X_WORDS.get(meta["xlabel"], meta["xlabel"]),
                                  Y_WORDS.get(meta["ylabel"], meta["ylabel"]), log_x=meta.get("log_x", False),
                                  log_y=meta.get("log_y", True), threshold=threshold, annotate=annotate))
        if verdict:
            out.append(f"<div class='chart-verdict'>{verdict}</div>")
        out.append(f"<figcaption><b>What it shows:</b> {escape(what)}<br><b>What a healthy curve looks like:</b> "
                   f"{escape(healthy)}</figcaption></figure>")
    if current_group is not None:
        out.append("</div>")
    return "\n".join(out)


NAV_JS = r"""
<script>
(function () {
  var pages = Array.prototype.slice.call(document.querySelectorAll('.page'));
  var links = Array.prototype.slice.call(document.querySelectorAll('nav.topbar a'));
  function show(id) {
    if (!document.getElementById(id)) { id = pages[0].id; }
    pages.forEach(function (p) { p.classList.toggle('active', p.id === id); });
    links.forEach(function (a) { a.classList.toggle('active', a.getAttribute('href') === '#' + id); });
    setTimeout(function () { window.scrollTo(0, 0); }, 0);
  }
  window.addEventListener('hashchange', function () { show(location.hash.slice(1)); });
  show(location.hash.slice(1) || pages[0].id);
})();
</script>"""


def _section_method(latest: Entry) -> str:
    c = latest.correctness or {}
    ref = c.get("reference", {})
    out = ["<h2>Method and glossary</h2>",
           f"<p>All checks use a time step of {c.get('dt', '?')} s, the value the engine uses, and G = {c.get('G', '?')} "
           f"km³·kg⁻¹·s⁻² (kilometres, kilograms, seconds). Masses go through the same float mantissa × 10ⁿ encoding "
           f"as in the engine, so the exact solutions use exactly the masses the module sees. The reference simulation "
           f"is a classical RK4 integrator in 80-bit floating point with {ref.get('substeps', '?')} sub-steps per frame "
           f"and no softening; its own error is several orders of magnitude below what is measured here.</p>",
           "<dl>"]
    for term, definition in GLOSSARY:
        out.append(f"<dt>{escape(term)}</dt><dd>{escape(definition)}</dd>")
    out.append("</dl>")
    return "\n".join(out)


PAGES = [("overview", "Overview"), ("speed", "Speed"), ("checks", "Checks"), ("charts", "Error charts"),
         ("method", "Method & glossary")]


def render(entries: list[Entry], plan: dict) -> str:
    """One self-contained file; the navbar switches between pages (hash links, works offline and when printed)."""
    entries = list(entries)
    with_bench = [e for e in entries if e.benchmark and e.benchmark.get("results")]
    with_tests = [e for e in entries if e.correctness]
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    nav = "<nav class='topbar'><span class='brand'>Physics QA</span>" + "".join(
        f"<a href='#{pid}'>{escape(title)}</a>" for pid, title in PAGES) + "</nav>"
    pages = {
        "overview": "\n".join(["<h1>Physics module QA report</h1>",
                               f"<p class='muted'>Generated {now} · {len(entries)} version(s) · "
                               f"{escape(entries[-1].build_type)} build · use the bar above to move between pages</p>",
                               _section_howto(entries), _section_glance(entries, with_bench, plan),
                               _section_versions(entries)]),
        "speed": _section_speed(entries, with_bench, plan),
        "checks": _section_checks(entries, with_tests),
        "charts": _section_charts(entries, with_tests) if with_tests else "<h2>Error charts</h2><p>No correctness data stored.</p>",
        "method": _section_method(entries[-1]),
    }
    parts = ["<!doctype html><html lang='en'><head><meta charset='utf-8'>",
             "<meta name='viewport' content='width=device-width, initial-scale=1'>",
             "<title>Physics QA report</title>", f"<style>{CSS}</style>",
             "<noscript><style>.page{display:block} nav.topbar a{pointer-events:none}</style></noscript></head><body>",
             nav, "<main>"]
    for pid, _ in PAGES:
        parts.append(f"<section class='page' id='{pid}'>{pages[pid]}</section>")
    parts.append("</main>" + NAV_JS + "</body></html>")
    return "\n".join(parts)
