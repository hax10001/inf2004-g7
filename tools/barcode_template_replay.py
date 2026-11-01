"""Explore template geometry on saved ADC dumps; never executes firmware."""
import argparse
import bisect
import json
import re
from pathlib import Path


def trunc(n, d):
    return abs(n) // abs(d) * (-1 if (n < 0) != (d < 0) else 1)


def analyze(path, ratios, raw=False):
    text = path.read_text(encoding="utf-8-sig")
    rows = [list(map(int, m.groups())) for m in re.finditer(
        r"\[BC WAVE RAW\]\s+(\d+),(\d+),(\d+),(\d+),"
        r"(\d+),(\d+),(\d+),(\d+)", text)]
    end = re.findall(r"\[BC WAVE RAW END\].*rows=(\d+)", text)
    if not rows or not end or len(rows) != int(end[-1]):
        raise ValueError("Requires a complete single-pass raw dump")
    pos = [r[3] for r in rows]
    if pos != sorted(pos) or [r[1] for r in rows] != list(range(len(rows))):
        raise ValueError("Requires chronological samples and monotonic travel")
    signal = [r[4 if raw else 5] - r[6] for r in rows]
    noise = max(4, int(re.search(r"white_mean=\d+ sigma=(\d+)", text)[1]))
    prefix = [0]
    for i in range(len(rows) - 1):
        prefix.append(prefix[-1] + signal[i] * (pos[i + 1] - pos[i]))

    def mean(centre, radius):
        if centre < radius or centre + radius > pos[-1] * 256:
            return None
        lo = bisect.bisect_left(pos, (centre - radius) // 256)
        hi = bisect.bisect_left(pos, (centre + radius) // 256 + 1)
        if hi <= lo or pos[hi - 1] == pos[lo]:
            return None
        return trunc(prefix[hi - 1] - prefix[lo], pos[hi - 1] - pos[lo])

    root = Path(__file__).resolve().parents[1]
    code = (root / "app_program/modules/barcode_scanning/bc_code.c").read_text()
    table = code.split("g_bc_patterns[26]", 1)[1].split("};", 1)[0]
    patterns = [int(v, 16) for v in re.findall(r"\{0x([0-9A-Fa-f]+)U\}", table)]
    models = {}
    support = [0] * 26
    levels = []
    previous = None
    for percent in [15, 20, 25, 40, 55, 70]:
        threshold = max(max(signal) * percent // 100, noise * 4)
        peaks = []
        active = False
        start = last = weighted = weights = 0
        for i in range(len(rows) + 1):
            p = pos[i] if i < len(rows) else last
            s = signal[i] if i < len(rows) else 0
            if i < len(rows) and s > threshold:
                if not active:
                    active, start, weighted, weights = True, p, 0, 0
                w = (s - threshold) * max(0, p - last)
                weighted += p * w
                weights += w
                last = p
            elif active:
                if last - start >= 3 and weights:
                    peaks.append(weighted // weights)
                active = False
            if i < len(rows):
                last = p
        levels.append(dict(percent=percent, threshold=threshold, peaks=peaks))
        duplicate = threshold == previous
        previous = threshold
        if duplicate or not 6 <= len(peaks) <= 12:
            continue
        scores = [-1000000] * 26
        for letter, mask in enumerate(patterns):
            for reverse in (0, 1):
                for ratio in ratios:
                    widths = []
                    for symbol, m in enumerate([0x94, mask, 0x94]):
                        if symbol:
                            widths.append(16)
                        widths.extend(ratio if m & (1 << (8 - i)) else 16
                                      for i in range(9))
                    if reverse:
                        widths.reverse()
                    cursor = 0
                    centres = []
                    for i, w in enumerate(widths):
                        if i % 2 == 0 and w > 16:
                            centres.append(cursor + w // 2)
                        cursor += w
                    sx = sum(centres)
                    xx = sum(x * x for x in centres)
                    for first in range(len(peaks) - 5):
                        ys = peaks[first:first + 6]
                        scale = trunc((6 * sum(x * y for x, y in zip(centres, ys))
                                       - sx * sum(ys)) * 256, 6 * xx - sx * sx)
                        if scale <= 0:
                            continue
                        origin = trunc(sum(ys) * 256 - scale * sx, 6)
                        end_q8 = origin + scale * sum(widths)
                        if not (1300 * 256 <= scale * sum(widths) <= 4800 * 256
                                and origin >= 60 * 256
                                and end_q8 + 60 * 256 <= pos[-1] * 256):
                            continue
                        if ((first and peaks[first - 1] * 256 > origin)
                                or (first + 6 < len(peaks)
                                    and peaks[first + 6] * 256 < end_q8)):
                            continue
                        geom = sum(abs(y * 256 - origin - scale * x)
                                   for x, y in zip(centres, ys)) * 1000 // (6 * scale * 16)
                        if geom > 250:
                            continue
                        for shift in (-1, 0, 1):
                            offset = origin + trunc(shift * scale * 16 * 15, 100)
                            geom = sum(abs(y * 256 - offset - scale * x)
                                       for x, y in zip(centres, ys)) * 1000 // (6 * scale * 16)
                            if geom > 250:
                                continue
                            radius = max(256, scale * 16 // 5)
                            cursor = score = positive = sf = sl = 0
                            deltas = []
                            for i, w in enumerate(widths):
                                if i % 2 == 0 and w == 16:
                                    c = offset + scale * (cursor + 8)
                                    left = offset + scale * (cursor - (widths[i - 1] // 2 if i else 8))
                                    right = offset + scale * (cursor + 16 + (widths[i + 1] // 2 if i < 28 else 8))
                                    black, wl, wr = [mean(u, radius) for u in (c, left, right)]
                                    if None in (black, wl, wr):
                                        score = -1000000
                                        break
                                    delta = black - trunc(wl * (right - c) + wr * (c - left), right - left)
                                    z = max(-600, min(600, trunc(delta * 100, noise)))
                                    score += z
                                    deltas.append(delta)
                                    if z >= 50:
                                        positive += 1
                                        sf += i < 9
                                        sl += i >= 20
                                cursor += w
                            score -= geom // 10
                            scores[letter] = max(scores[letter], score)
                            if score > models.get(letter, {}).get("score", -1000000):
                                models[letter] = dict(letter=chr(65 + letter), score=score,
                                    reverse=reverse, ratio=ratio, scale=scale, offset=offset,
                                    positive=positive, eligible=positive >= 6 and sf > 0 and sl > 0,
                                    deltas=deltas, geometry=geom)
        rank = sorted(range(26), key=lambda x: scores[x], reverse=True)
        if scores[rank[0]] >= 450 and scores[rank[0]] > scores[rank[1]]:
            support[rank[0]] += 1
    rank = sorted(models, key=lambda x: models[x]["score"], reverse=True)
    top = [dict(models[i], support=support[i]) for i in rank[:3]]
    margin = top[0]["score"] - top[1]["score"] if len(top) > 1 else 0
    valid = bool(top and top[0]["eligible"] and top[0]["score"] >= 450
                 and margin >= 300 and top[0]["support"] >= 2)
    return dict(log=str(path), signal="raw" if raw else "smoothed",
                levels=levels, top=top, margin=margin, valid=valid)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--dense", action="store_true")
    parser.add_argument("--raw", action="store_true",
                        help="Use original ADC minus saved floor, not smoothing")
    args = parser.parse_args()
    ratios = list(range(32, 65)) if args.dense else [40, 48, 56]
    print(json.dumps([analyze(p, ratios, args.raw) for p in args.logs], indent=2))
