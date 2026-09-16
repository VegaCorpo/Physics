"""Dependency-free SVG charts (line and bar) for the HTML report.

Colors are CSS custom properties (--series-N, --text-*, --grid) so the same SVG
adapts to the light and dark themes declared by the report stylesheet.
"""

from __future__ import annotations

import math
from html import escape

SERIES_SLOTS = 8


def _nice_ticks(lo: float, hi: float, count: int = 5) -> list[float]:
    if hi <= lo:
        hi = lo + 1.0
    span = hi - lo
    raw = span / max(1, count)
    mag = 10 ** math.floor(math.log10(raw))
    for step in (1, 2, 2.5, 5, 10):
        if raw <= step * mag:
            step *= mag
            break
    start = math.floor(lo / step) * step
    ticks = []
    value = start
    while value <= hi + step * 1e-9:
        if value >= lo - step * 1e-9:
            ticks.append(round(value, 12))
        value += step
    return ticks


def _log_ticks(lo: float, hi: float) -> list[float]:
    lo_e = math.floor(math.log10(lo))
    hi_e = math.ceil(math.log10(hi))
    if hi_e - lo_e <= 1:
        ticks = [m * 10 ** e for e in range(lo_e, hi_e + 1) for m in (1, 2, 5)]
    else:
        step = max(1, (hi_e - lo_e) // 6)
        ticks = [10 ** e for e in range(lo_e, hi_e + 1, step)]
    return [t for t in ticks if lo / 1.0001 <= t <= hi * 1.0001]


_SUP = str.maketrans("0123456789-", "⁰¹²³⁴⁵⁶⁷⁸⁹⁻")


def pow10(value: float) -> str:
    """Tick label for a log axis: 10⁻⁵, 2×10⁻⁵, 100, 1 000…"""
    if value <= 0:
        return fmt(value)
    exponent = math.floor(math.log10(value) + 1e-9)
    mantissa = value / 10 ** exponent
    if 1e-3 <= value < 1e4:
        return f"{value:g}" if value < 1e3 else f"{value:,.0f}"
    lead = "" if abs(mantissa - 1) < 1e-6 else f"{mantissa:g}×"
    return f"{lead}10{str(exponent).translate(_SUP)}"


def fmt(value: float) -> str:
    if value == 0:
        return "0"
    if abs(value) >= 1e4 or abs(value) < 1e-3:
        text = f"{value:.1e}"
        mantissa, exponent = text.split("e")
        mantissa = mantissa.rstrip("0").rstrip(".")
        return f"{mantissa}e{int(exponent)}"
    return f"{value:.4g}"


class _Scale:
    def __init__(self, lo: float, hi: float, px0: float, px1: float, log: bool):
        self.log = log
        if log:
            lo = max(lo, 1e-300)
            hi = max(hi, lo * 10)
            self.lo, self.hi = math.log10(lo), math.log10(hi)
        else:
            if hi == lo:
                hi = lo + 1
            self.lo, self.hi = lo, hi
        self.px0, self.px1 = px0, px1

    def __call__(self, v: float) -> float:
        x = math.log10(v) if self.log else v
        return self.px0 + (x - self.lo) / (self.hi - self.lo) * (self.px1 - self.px0)


def line_chart(series: list[dict], title: str, xlabel: str, ylabel: str, log_x: bool = False,
               log_y: bool = False, width: int = 720, height: int = 340, threshold: float | None = None,
               threshold_label: str = "limit", annotate: list[tuple] | None = None) -> str:
    """series: [{"name": str, "x": [...], "y": [...], "slot": int}] ; slot picks --series-N.

    annotate: [(x, y, text)] points to call out (e.g. the worst value of the newest version).
    When a threshold is given the region above it is shaded so "too much error" is visible at a glance.
    """
    left, right, top, bottom = 72, 24, 40, 56
    if len(series) <= 4:
        right += 12 + 7 * max((len(s["name"]) for s in series), default=0)
    pts = []
    for s in series:
        for x, y in zip(s["x"], s["y"]):
            if y is None or (log_y and y <= 0) or (log_x and x <= 0) or not math.isfinite(y):
                continue
            pts.append((x, y))
    if not pts:
        return f'<div class="chart-empty">{escape(title)}: no data</div>'
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts] + ([threshold] if threshold and (not log_y or threshold > 0) else [])
    xlo, xhi = min(xs), max(xs)
    ylo, yhi = min(ys), max(ys)
    if log_y:
        ylo, yhi = ylo / 2, yhi * 2
    else:
        pad = (yhi - ylo) * 0.08 or abs(yhi) * 0.1 or 1
        ylo, yhi = (0 if ylo >= 0 else ylo - pad), yhi + pad
    if log_x:
        xlo, xhi = xlo / 1.5, xhi * 1.5
    sx = _Scale(xlo, xhi, left, width - right, log_x)
    sy = _Scale(ylo, yhi, height - bottom, top, log_y)

    out = [f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title)}">',
           f'<text class="chart-title" x="{left}" y="22">{escape(title)}</text>']
    yticks = _log_ticks(ylo, yhi) if log_y else _nice_ticks(ylo, yhi)
    for t in yticks:
        y = sy(t)
        out.append(f'<line class="grid" x1="{left}" x2="{width - right}" y1="{y:.1f}" y2="{y:.1f}"/>')
        out.append(f'<text class="tick" x="{left - 8}" y="{y + 4:.1f}" text-anchor="end">{pow10(t) if log_y else fmt(t)}</text>')
    xticks = _log_ticks(xlo, xhi) if log_x else _nice_ticks(xlo, xhi, 6)
    for t in xticks:
        x = sx(t)
        out.append(f'<line class="grid" y1="{top}" y2="{height - bottom}" x1="{x:.1f}" x2="{x:.1f}"/>')
        out.append(f'<text class="tick" x="{x:.1f}" y="{height - bottom + 18}" text-anchor="middle">{pow10(t) if log_x else fmt(t)}</text>')
    out.append(f'<line class="axis" x1="{left}" x2="{width - right}" y1="{height - bottom}" y2="{height - bottom}"/>')
    out.append(f'<text class="axis-label" x="{(left + width - right) / 2:.0f}" y="{height - 12}" '
               f'text-anchor="middle">{escape(xlabel)}</text>')
    out.append(f'<text class="axis-label" transform="translate(16 {(top + height - bottom) / 2:.0f}) rotate(-90)" '
               f'text-anchor="middle">{escape(ylabel)}</text>')
    if threshold and (not log_y or threshold > 0) and ylo <= threshold <= yhi:
        y = sy(threshold)
        out.append(f'<rect class="over-zone" x="{left}" y="{top}" width="{width - right - left}" height="{max(0.0, y - top):.1f}"/>')
        out.append(f'<line class="threshold" x1="{left}" x2="{width - right}" y1="{y:.1f}" y2="{y:.1f}"/>')
        out.append(f'<text class="threshold-label" x="{left + 6}" y="{y - 5:.1f}">'
                   f'{escape(threshold_label)} {pow10(threshold) if log_y else fmt(threshold)} — above this line: too much error</text>')

    labels = []
    for s in series:
        slot = (s.get("slot", 0) % SERIES_SLOTS) + 1
        color = f"var(--series-{slot})"
        coords = [(sx(x), sy(y)) for x, y in zip(s["x"], s["y"])
                  if y is not None and math.isfinite(y) and (not log_y or y > 0) and (not log_x or x > 0)]
        if not coords:
            continue
        path = " ".join(f"{'M' if i == 0 else 'L'}{x:.1f} {y:.1f}" for i, (x, y) in enumerate(coords))
        out.append(f'<path class="line" d="{path}" stroke="{color}"/>')
        for (px, py), x, y in zip(coords, s["x"], s["y"]):
            out.append(f'<circle class="marker" cx="{px:.1f}" cy="{py:.1f}" r="4" fill="{color}">'
                       f'<title>{escape(s["name"])}\n{escape(xlabel)}: {fmt(x)}\n{escape(ylabel)}: {fmt(y)}</title></circle>')
        labels.append((coords[-1][1], s["name"], color))
    for ax, ay, text in (annotate or []):
        if (log_y and ay <= 0) or (log_x and ax <= 0):
            continue
        px, py = sx(ax), sy(ay)
        anchor = "end" if px > (left + width - right) / 2 else "start"
        tx = px - 10 if anchor == "end" else px + 10
        out.append(f'<circle class="callout-ring" cx="{px:.1f}" cy="{py:.1f}" r="8"/>')
        out.append(f'<text class="callout" x="{tx:.1f}" y="{py - 10:.1f}" text-anchor="{anchor}">{escape(text)}</text>')
    if len(series) <= 4:
        used = []
        for y, name, color in sorted(labels):
            while any(abs(y - u) < 14 for u in used):
                y += 14
            used.append(y)
            out.append(f'<text class="direct-label" x="{width - right + 4}" y="{y + 4:.1f}">'
                       f'<tspan fill="{color}">●</tspan> {escape(name)}</text>')
    out.append("</svg>")
    if len(series) >= 2:
        out.append('<div class="legend">' + "".join(
            f'<span class="legend-item"><span class="swatch" style="background:var(--series-{(s.get("slot", 0) % SERIES_SLOTS) + 1})"></span>{escape(s["name"])}</span>'
            for s in series) + "</div>")
    return "\n".join(out)


def bar_chart(categories: list[str], series: list[dict], title: str, ylabel: str, width: int = 720,
              height: int = 320, threshold: float | None = None, threshold_label: str = "limit",
              symmetric: bool = False) -> str:
    """Grouped bars. series: [{"name", "values": [...], "slot"}], values aligned with categories."""
    left, right, top, bottom = 72, 24, 40, 56
    values = [v for s in series for v in s["values"] if v is not None and math.isfinite(v)]
    if not values:
        return f'<div class="chart-empty">{escape(title)}: no data</div>'
    lo, hi = min(values + [0]), max(values + [0])
    if threshold is not None:
        hi = max(hi, threshold)
        lo = min(lo, -threshold if symmetric else lo)
    pad = (hi - lo) * 0.1 or 1
    lo, hi = (lo - pad if lo < 0 else 0), hi + pad
    sy = _Scale(lo, hi, height - bottom, top, False)
    out = [f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title)}">',
           f'<text class="chart-title" x="{left}" y="22">{escape(title)}</text>']
    for t in _nice_ticks(lo, hi):
        y = sy(t)
        out.append(f'<line class="grid" x1="{left}" x2="{width - right}" y1="{y:.1f}" y2="{y:.1f}"/>')
        out.append(f'<text class="tick" x="{left - 8}" y="{y + 4:.1f}" text-anchor="end">{fmt(t)}</text>')
    zero = sy(0)
    out.append(f'<line class="axis" x1="{left}" x2="{width - right}" y1="{zero:.1f}" y2="{zero:.1f}"/>')
    out.append(f'<text class="axis-label" transform="translate(16 {(top + height - bottom) / 2:.0f}) rotate(-90)" '
               f'text-anchor="middle">{escape(ylabel)}</text>')
    for t, label in ((threshold, threshold_label), (-threshold if symmetric and threshold else None, threshold_label)):
        if t is not None and lo <= t <= hi:
            y = sy(t)
            out.append(f'<line class="threshold" x1="{left}" x2="{width - right}" y1="{y:.1f}" y2="{y:.1f}"/>')
            out.append(f'<text class="tick" x="{width - right}" y="{y - 4:.1f}" text-anchor="end">{escape(label)} {fmt(t)}</text>')
    n_cat = max(1, len(categories))
    group_w = (width - left - right) / n_cat
    bar_w = max(4, min(28, (group_w * 0.7) / max(1, len(series))))
    for ci, cat in enumerate(categories):
        gx = left + ci * group_w + group_w / 2
        out.append(f'<text class="tick" x="{gx:.1f}" y="{height - bottom + 18}" text-anchor="middle">{escape(str(cat))}</text>')
        total_w = bar_w * len(series) + 2 * (len(series) - 1)
        for si, s in enumerate(series):
            v = s["values"][ci] if ci < len(s["values"]) else None
            if v is None or not math.isfinite(v):
                continue
            slot = (s.get("slot", 0) % SERIES_SLOTS) + 1
            x = gx - total_w / 2 + si * (bar_w + 2)
            y0, y1 = sorted((sy(v), zero))
            h = max(1.0, y1 - y0)
            out.append(f'<rect class="bar" x="{x:.1f}" y="{y0:.1f}" width="{bar_w:.1f}" height="{h:.1f}" rx="3" '
                       f'fill="var(--series-{slot})"><title>{escape(s["name"])}\n{escape(str(cat))}: {fmt(v)}</title></rect>')
    out.append("</svg>")
    if len(series) >= 2:
        out.append('<div class="legend">' + "".join(
            f'<span class="legend-item"><span class="swatch" style="background:var(--series-{(s.get("slot", 0) % SERIES_SLOTS) + 1})"></span>{escape(s["name"])}</span>'
            for s in series) + "</div>")
    return "\n".join(out)
