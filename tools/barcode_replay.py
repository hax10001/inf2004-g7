"""Replay saved barcode passes offline and score decoders against labels.

    python tools/barcode_replay.py tools/barcode_data            # all passes
    python tools/barcode_replay.py tools/barcode_data/2026-11-06 -v

Any saved serial log works, one pass or many (each pass runs from
[BC WAVE START] to [BC WAVE RAW END]). The expected letter comes from the
file name, size_LETTER_method[_NN].log, e.g. big_F_push.log; "unlabelled"
means unknown.

Decoders compared:
  firmware  what the robot itself decided ([BC WAVE RESULT] in the log)
  layout    experimental: the six strong wide bars give scale, offset and
            direction (from the two '*' characters) and narrow the letter to
            its wide-bar group; the faint narrow-bar signal, averaged over
            each candidate's narrow-bar positions, picks within the group.

Distance: the firmware's units are left + right wheel counts. --sensor-left
MM (test-mode logs only, which carry a left-wheel column) re-measures
distance at a sensor MM left of centre instead, e.g. --sensor-left 70.
"""
import argparse
import collections
import itertools
import math
import pathlib
import re

TRACK_MM = 110.0
PATTERNS = [0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
            0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
            0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0]
STAR = 0x094
RATIO = 3.0  # wide:narrow of the supplied prints


# --------------------------------------------------------------- parsing
def load(path):
    """Split a log into passes: (rows, sigma, result, ignored) each.

    A pass runs from [BC WAVE START] to [BC WAVE RAW END]; a log saved
    straight from a serial monitor may hold any number of them.
    """
    passes, sigma = [], 4
    cur = None

    def new():
        return {"rows": [], "sigma": sigma, "result": None, "ignored": False}

    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.search(r"READY\] white_mean=\d+ sigma=(\d+)", line)
        if m:
            sigma = int(m.group(1))
        if "[BC WAVE START]" in line or cur is None:
            if cur is not None and cur["rows"]:
                passes.append(cur)
            cur = new()
        m = re.search(r"\[BC WAVE RAW\] ([\d,]+)", line)
        if m:
            vals = m.group(1).split(",")
            # Skip rows cut short or interleaved with other prints; rows
            # of the wrong width are dropped when the pass is finished.
            if all(vals) and len(vals) >= 8:
                cur["rows"].append([int(v) for v in vals])
            continue
        m = re.search(r"\[BC WAVE RESULT\].*?valid=(\d) letter=(\d+).*?source=(\w+)", line)
        if m:
            cur["result"] = (int(m.group(1)), int(m.group(2)), m.group(3))
        if "[BARCODE] Ignored" in line:
            cur["ignored"] = True
        if "[BC WAVE RAW END]" in line:
            passes.append(cur)
            cur = None
    if cur is not None and cur["rows"]:
        passes.append(cur)
    for p in passes:
        widths = collections.Counter(len(r) for r in p["rows"])
        if widths:
            width = widths.most_common(1)[0][0]
            p["rows"] = [r for r in p["rows"] if len(r) == width]
    return [(p["rows"], p["sigma"], p["result"], p["ignored"]) for p in passes]


def expected_letter(path):
    parts = path.stem.split("_")
    if len(parts) >= 2 and len(parts[1]) == 1 and parts[1].isalpha():
        return parts[1].upper()
    return None


def group_of(path):
    return re.sub(r"_\d+$", "", path.stem)


def positions(rows, sensor_left_mm):
    """Distance per sample in firmware units (left+right counts)."""
    units = [r[3] for r in rows]
    if sensor_left_mm is None or len(rows[0]) < 9:
        return units
    # Unwrap the 16-bit left column, then weight the wheels for a sensor
    # sensor_left_mm left of centre: s = (L+R)/2 - y (R-L)/track.
    left, base, prev = [], 0, rows[0][8]
    for r in rows:
        if r[8] < prev:
            base += 65536
        prev = r[8]
        left.append(base + r[8])
    k = sensor_left_mm / TRACK_MM
    out = []
    for u, l in zip(units, left):
        r = u - l
        out.append(2.0 * ((l + r) / 2.0 - k * (r - l)))
    return out


# ------------------------------------------------------------ geometry
def elements(pattern):
    return [RATIO if (pattern >> (8 - i)) & 1 else 1.0 for i in range(9)]


def symbol_layout(letter):
    """Module intervals of every element of *X*, with black flags."""
    seq = []
    for p in (STAR, PATTERNS[letter], STAR):
        if seq:
            seq.append((1.0, False))  # inter-character gap
        seq += [(w, i % 2 == 0) for i, w in enumerate(elements(p))]
    out, x = [], 0.0
    for w, black in seq:
        out.append((x, x + w, black, w > 1.0))
        x += w
    return out, x


def wide_bar_centres(letter):
    lay, total = symbol_layout(letter)
    return [(a + b) / 2 for a, b, black, wide in lay if black and wide], total


def strong_bars(pos, sig):
    peak = max(sig)
    on, off = peak / 2, peak * 3 / 8
    bars, black, start = [], False, 0
    for i, v in enumerate(sig):
        if not black and v >= on:
            black, start = True, i
        elif black and (v <= off or i == len(sig) - 1):
            black = False
            seg = range(start, i + 1)
            wsum = sum(sig[j] for j in seg) or 1
            bars.append((sum(pos[j] * sig[j] for j in seg) / wsum,
                         pos[i] - pos[start]))
    return bars, peak


def top_bars(pos, sig, sep=60):
    """The six largest separated peaks (wide bars fade along some passes).

    Returns (bars, ratio) where bars are (centre, height) sorted by position
    and ratio is 6th / 7th peak height (how clearly the six stand out).
    """
    peaks = []
    for i in range(1, len(sig) - 1):
        if sig[i] >= sig[i - 1] and sig[i] > sig[i + 1]:
            if peaks and pos[i] - pos[peaks[-1]] < sep:
                if sig[i] > sig[peaks[-1]]:
                    peaks[-1] = i
            else:
                peaks.append(i)
    peaks.sort(key=lambda i: -sig[i])
    top = peaks[:6]
    if len(top) < 6:
        return [], 0.0
    seventh = sig[peaks[6]] if len(peaks) > 6 else 1
    bars = []
    for i in sorted(top):
        half = sig[i] / 2
        j0 = j1 = i
        while j0 > 0 and sig[j0 - 1] >= half:
            j0 -= 1
        while j1 < len(sig) - 1 and sig[j1 + 1] >= half:
            j1 += 1
        w = sum(sig[j] for j in range(j0, j1 + 1)) or 1
        bars.append((sum(pos[j] * sig[j] for j in range(j0, j1 + 1)) / w,
                     sig[i]))
    return bars, sig[top[5]] / max(seventh, 1)


def _phi(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def model_rmse(pos, sig, letter, reverse, a, b, blur=0.7):
    """Least-squares misfit of the blurred *X* model, best small shift."""
    lay, total = symbol_layout(letter)
    blacks = [(x0, x1) for x0, x1, black, _ in lay if black]
    best = None
    # Firmware budget: at most MODEL_SAMPLES samples per fit (every
    # stride-th sample of those over the symbol).
    inside = sum(1 for p in pos if -3 < (p - b) / a < total + 3)
    stride = max(1, -(-inside // MODEL_SAMPLES))
    for scale in (0.985, 1.0, 1.015):
        xs0 = [(p - b) / (a * scale) for p in pos]
        for shift in [k * 0.1 for k in range(-5, 6)]:
            idx, model, ys = [], [], []
            for i, x in enumerate(xs0):
                if i % stride:
                    continue
                x += shift
                if not -3 < x < total + 3:
                    continue
                if reverse:
                    x = total - x
                v = 0.0
                for x0, x1 in blacks:
                    if x0 - 3 < x < x1 + 3:
                        v += _phi((x1 - x) / blur) - _phi((x0 - x) / blur)
                model.append(v)
                ys.append(sig[i])
            n = len(model)
            if n < 20:
                continue
            mx, my = sum(model) / n, sum(ys) / n
            sxx = sum((m - mx) ** 2 for m in model) or 1e-9
            g = sum((m - mx) * (y - my) for m, y in zip(model, ys)) / sxx
            if g <= 0:
                continue
            c = my - g * mx
            rmse = math.sqrt(sum((y - g * m - c) ** 2
                                 for m, y in zip(model, ys)) / n)
            if best is None or rmse < best:
                best = rmse
    return best


def fit_line(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return a, my - a * mx


# Narrow-module size in L+R units, measured: small prints ~37, big ~73.
MODULE_UNITS = ((33.0, 41.0), (66.0, 80.0))
FIT_TOL = 0.8      # worst wide-bar position error accepted, modules
CLASS_MARGIN = 0.25  # a different wide-bar layout this close = ambiguous
MAX_CANDIDATES = 10
MODEL_SAMPLES = 512
COMBINED_LEFT_MM = 70.0  # sensor offset used by the default combined mode


def candidate_bars(pos, sig, sep=60):
    """Centres of dark bars strong enough to be wide bars.

    The threshold comes from the sixth-tallest peak, not the tallest, so one
    much darker mark (sensor lifted, track line) cannot hide real bars.
    Split peaks inside one wide bar merge within sep units.
    """
    peaks = []
    for i in range(1, len(sig) - 1):
        if sig[i] >= sig[i - 1] and sig[i] > sig[i + 1]:
            if peaks and pos[i] - pos[peaks[-1]] < sep:
                if sig[i] > sig[peaks[-1]]:
                    peaks[-1] = i
            else:
                peaks.append(i)
    heights = sorted((sig[i] for i in peaks), reverse=True)
    if len(heights) < 6:
        return []
    ref = heights[5]
    clip = [min(v, 1.5 * ref) for v in sig]
    on, off = ref / 2, ref * 3 / 8
    bars, black, start = [], False, 0
    for i, v in enumerate(clip):
        if not black and v >= on:
            black, start = True, i
        elif black and (v <= off or i == len(clip) - 1):
            black = False
            seg = range(start, i + 1)
            w = sum(clip[j] for j in seg) or 1
            bars.append((sum(pos[j] * clip[j] for j in seg) / w,
                         max(sig[j] for j in seg)))
    if len(bars) > MAX_CANDIDATES:
        keep = sorted(sorted(bars, key=lambda t: -t[1])[:MAX_CANDIDATES])
        bars = keep
    return [c for c, _ in bars]


def layout_decode(pos, sig, sigma):
    """Return (letter or None, note) using wide bars then narrow evidence."""
    bars = candidate_bars(pos, sig)
    if len(bars) < 6:
        return None, f"{len(bars)} candidate bars (need 6)"
    # Extra dark marks (track line, a wobble lifting the sensor) can sit
    # before, after or inside the symbol: try every ordered choice of six
    # candidates and keep the layouts that fit.
    best = []
    for pick in itertools.combinations(range(len(bars)), 6):
        centres = [bars[i] for i in pick]
        for reverse in (False, True):
            cs = centres[::-1] if reverse else centres
            for letter in range(26):
                model, total = wide_bar_centres(letter)
                m = [total - x for x in model] if reverse else model
                a, b = fit_line(m, cs)
                if not any(lo <= a <= hi for lo, hi in MODULE_UNITS):
                    continue
                err = max(abs((cs[i] - b) / a - m[i]) for i in range(6))
                best.append((err, letter, reverse, a, b))
    if not best:
        return None, "no wide-bar layout fits"
    best.sort()
    if best[0][0] > FIT_TOL:
        return None, f"no wide-bar layout fits (best err {best[0][0]:.2f} modules)"
    # Letters sharing a wide-bar layout form one class; the model fit picks
    # within it. Two classes fitting about equally well: refuse to guess.
    # Layout key: wide-bar centres as seen along the scan, shifted to start
    # at 0. Some letters read backwards match others read forwards up to a
    # shift (Z backwards = G forwards); the wide bars cannot tell them
    # apart, so they share a group and the full model decides.
    def layout_key(t):
        centres, total = wide_bar_centres(t[1])
        if t[2]:
            centres = sorted(total - x for x in centres)
        return tuple(round(x - centres[0], 3) for x in centres)
    close = [t for t in best if t[0] <= best[0][0] + CLASS_MARGIN]
    if len({layout_key(t) for t in close}) > 1:
        names = "".join(sorted({chr(65 + t[1]) for t in close}))
        return None, f"ambiguous wide-bar layouts ({names})"
    key = layout_key(best[0])
    group, seen = [], set()
    for t in best:
        if layout_key(t) == key and (t[1], t[2]) not in seen:
            seen.add((t[1], t[2]))
            group.append(t)
    # Within the wide-bar group, the blurred full-symbol model decides: it
    # accounts for wide bars bleeding into neighbouring narrow positions.
    scored = []
    for err, letter, reverse, a, b in group:
        rmse = model_rmse(pos, sig, letter, reverse, a, b)
        if rmse is not None:
            scored.append((rmse, letter, reverse, err))
    if not scored:
        return None, "model fit failed"
    scored.sort()
    top = scored[0]
    names = "".join(chr(65 + s[1]) for s in scored)
    if len(scored) == 1:
        return chr(65 + top[1]), "unique layout"
    gap = 100.0 * (scored[1][0] - top[0]) / top[0]
    return chr(65 + top[1]), f"group {names}, misfit gap {gap:.1f}%"


# ---------------------------------------------------------------- main
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--sensor-left", type=float, default=None, metavar="MM")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    files = []
    for p in map(pathlib.Path, args.paths):
        files += sorted(p.rglob("*.log")) if p.is_dir() else [p]
    files = [f for f in files if not f.name.startswith("session_")]
    tally = collections.defaultdict(lambda: collections.Counter())
    for f in files:
        want = expected_letter(f)
        for n, (rows, sigma, result, ignored) in enumerate(load(f), 1):
            if not rows:
                continue
            sig = [max(0, r[5] - r[6]) for r in rows]
            fw = "ignored" if ignored else (
                chr(65 + result[1]) if result and result[0] else "fail")
            if args.sensor_left is not None or len(rows[0]) < 9:
                lay, note = layout_decode(
                    positions(rows, args.sensor_left), sig, sigma)
            else:
                # Combined: wheel-centre distance and distance at the sensor
                # (COMBINED_LEFT_MM left of centre). Either may read; if both
                # read and disagree, refuse.
                l1, n1 = layout_decode(positions(rows, None), sig, sigma)
                l2, n2 = layout_decode(
                    positions(rows, COMBINED_LEFT_MM), sig, sigma)
                if l1 and l2 and l1 != l2:
                    lay, note = None, f"centre says {l1}, sensor says {l2}"
                elif l1:
                    lay, note = l1, "centre: " + n1
                else:
                    lay, note = l2, ("sensor: " + n2) if l2 else n1
            g = tally[group_of(f)]
            g["passes"] += 1
            if want:
                g["fw_ok"] += fw == want
                g["layout_ok"] += lay == want
                g["layout_wrong"] += lay is not None and lay != want

            def mark(got):
                return "" if not want else (" OK" if got == want else " WRONG")

            print(f"{f.name + ' #' + str(n):28s} want={want or '?'}  "
                  f"firmware={fw}{mark(fw)}  layout={lay or '-'}{mark(lay)}  "
                  f"({note})")
    print("\nSummary (correct / passes):")
    for name, g in sorted(tally.items()):
        print(f"  {name:22s} firmware {g['fw_ok']}/{g['passes']}   "
              f"layout {g['layout_ok']}/{g['passes']} "
              f"(wrong {g['layout_wrong']})")


if __name__ == "__main__":
    main()
