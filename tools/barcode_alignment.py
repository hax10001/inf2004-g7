"""Analyze saved BC WAVE RAW logs; never builds or accesses the robot."""
import argparse
import bisect
import csv
import json
import re
import statistics
from pathlib import Path


def trunc_div(numerator, denominator):
    magnitude = abs(numerator) // abs(denominator)
    return -magnitude if (numerator < 0) != (denominator < 0) else magnitude


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    text = args.log.read_text(encoding="utf-8-sig")
    rows = [list(map(int, match.groups())) for match in re.finditer(
        r"\[BC WAVE RAW\]\s+(\d+),(\d+),(\d+),(\d+),"
        r"(\d+),(\d+),(\d+),(\d+)", text)]
    ends = re.findall(r"\[BC WAVE RAW END\]\s+pass=(\d+)\s+rows=(\d+)", text)
    if not rows or not ends or len(rows) != int(ends[-1][1]):
        raise ValueError("A complete single-pass RAW BEGIN/END dump is required")
    if [row[1] for row in rows] != list(range(len(rows))):
        raise ValueError("Sample indices must be complete and chronological")
    positions = [row[3] for row in rows]
    if positions != sorted(positions):
        raise ValueError("Travel must be monotonic")
    values = [row[5] - row[6] for row in rows]
    prefix = [0]
    for i in range(len(rows) - 1):
        prefix.append(prefix[-1] + values[i] * (positions[i + 1] - positions[i]))
    half_prefix = prefix[::2]
    mismatches = 0
    queries = 0

    def area_before(index):
        area = half_prefix[index // 2]
        if index % 2:
            area += values[index - 1] * (positions[index] - positions[index - 1])
        return area

    def mean(centre, radius):
        nonlocal mismatches, queries
        if centre < radius or centre + radius > positions[-1] * 256:
            return None
        lo = bisect.bisect_left(positions, (centre - radius) // 256)
        hi = bisect.bisect_left(positions, (centre + radius) // 256 + 1)
        if hi <= lo or positions[hi - 1] == positions[lo]:
            return None
        direct = sum(values[i] * (positions[i + 1] - positions[i])
                     for i in range(lo, hi - 1))
        cached = area_before(hi - 1) - area_before(lo)
        queries += 1
        mismatches += direct != cached
        return trunc_div(cached, positions[hi - 1] - positions[lo])

    root = Path(__file__).resolve().parents[1]
    code = (root / "app_program/modules/barcode_scanning/bc_code.c").read_text()
    table = code.split("g_bc_patterns[26]", 1)[1].split("};", 1)[0]
    patterns = [int(value, 16) for value in re.findall(r"\{0x([0-9A-Fa-f]+)U\}", table)]
    models = []
    for match in re.finditer(
            r"\[BC MATCH MODEL\]\s+letter=([A-Z])\s+offset_q8=(-?\d+)\s+"
            r"scale_q8=(\d+)\s+ratio_x16=(\d+)\s+geom_permille=(\d+)", text):
        letter, offset, scale, ratio, geometry = match.groups()
        reverse_match = re.search(
            rf"\[BC MATCH FEATURE\]\s+letter={letter}\s+rev=(\d+)", text)
        models.append(dict(letter=letter, offset_q8=int(offset),
                           scale_q8=int(scale), ratio=int(ratio),
                           reverse=int(reverse_match[1]) if reverse_match else 0,
                           geometry=int(geometry)))
    if not models:
        raise ValueError("No logged MATCH MODEL parameters were found")

    features = []
    for model in models:
        widths = []
        for symbol, mask in enumerate([0x94, patterns[ord(model["letter"]) - 65], 0x94]):
            if symbol:
                widths.append(16)
            widths.extend(model["ratio"] if mask & (1 << (8 - i)) else 16
                          for i in range(9))
        if model["reverse"]:
            widths.reverse()
        cursor = 0
        scale = model["scale_q8"]
        radius = max(256, scale * 16 // 5)
        for i, width in enumerate(widths):
            if i % 2 == 0 and width == 16:
                centre = model["offset_q8"] + scale * (cursor + 8)
                left = model["offset_q8"] + scale * (cursor - (widths[i - 1] // 2 if i else 8))
                right = model["offset_q8"] + scale * (
                    cursor + 16 + (widths[i + 1] // 2 if i + 1 < 29 else 8))

                def contrast(shift):
                    black = mean(centre + shift, radius)
                    wl = mean(left + shift, radius)
                    wr = mean(right + shift, radius)
                    if None in (black, wl, wr):
                        return None
                    white = trunc_div(wl * (right - centre) + wr * (centre - left), right - left)
                    return black - white

                delta = contrast(0)
                trials = []
                for step in range(-7, 8):
                    shift = trunc_div(scale * 16 * step, 20)
                    value = contrast(shift)
                    if value is not None:
                        trials.append((value, step * 0.05))
                local_delta, local_shift = max(trials, key=lambda item: (item[0], -abs(item[1])))
                features.append(dict(letter=model["letter"], i=i,
                                     centre_units=centre / 256,
                                     half_width_units=radius / 256,
                                     delta=delta, local_best_delta=local_delta,
                                     local_shift_narrow=round(local_shift, 2)))
            cursor += width

    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "waveform.csv").open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["pass", "i", "dt_us", "units", "adc", "smooth", "floor", "contrast", "corrected"])
        writer.writerows(row + [value] for row, value in zip(rows, values))
    with (args.output / "features.csv").open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=list(features[0]))
        writer.writeheader()
        writer.writerows(features)
    (args.output / "models.json").write_text(json.dumps(models, indent=2))
    report = ["# Saved waveform alignment analysis", "",
              f"Source: `{args.log.name}`; {len(rows)} complete samples.",
              f"Travel: {positions[-1]} units; duration: {rows[-1][2] / 1e6:.3f} s.",
              f"Exact spatial-integral comparisons: {queries}; differences: {mismatches}.", "",
              "Local offsets below are exploratory maxima within +/-0.35 narrow widths.",
              "Searching for a maximum can select noise; these are not accepted barcode edges.", "",
              "| Model | Element | Current contrast | Best local contrast | Local shift / narrow width |",
              "|---|---:|---:|---:|---:|"]
    for feature in features:
        report.append(f"| {feature['letter']} | {feature['i']} | {feature['delta']} | "
                      f"{feature['local_best_delta']} | {feature['local_shift_narrow']:+.2f} |")
    for model in models:
        subset = [feature for feature in features if feature["letter"] == model["letter"]]
        stars = [feature for feature in subset if feature["i"] < 9 or feature["i"] >= 20]
        report += ["", f"{model['letter']}: median star offset "
                   f"{statistics.median(f['local_shift_narrow'] for f in stars):+.2f} narrow widths; "
                   f"star offset range {min(f['local_shift_narrow'] for f in stars):+.2f} to "
                   f"{max(f['local_shift_narrow'] for f in stars):+.2f}."]
    report += ["", "Firmware was not built or executed. No acceptance limits were changed.", ""]
    (args.output / "alignment.md").write_text("\n".join(report))
    print(json.dumps(dict(rows=len(rows), queries=queries, differences=mismatches,
                          features=features), indent=2))


if __name__ == "__main__":
    main()
