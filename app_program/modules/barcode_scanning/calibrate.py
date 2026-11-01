#!/usr/bin/env python3
"""Convert supplied Code 39 artwork sizes into a calibration worksheet.

Drawing dimensions assume 100% printing. --span-mm overrides them with the
actual first-to-last black bar measurement. No firmware settings are written.
"""
import argparse
import math


# From BARCODE.pptx picture extents and Barcode Sample.pdf vector transforms.
# Small A and Z differ by about 0.04 mm across the complete barcode.
SPANS_MM = {'small': 164.90, 'medium': 242.50, 'full': 328.50}


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layout', choices=SPANS_MM,
                        help='small: PDF 2/4; medium: PPT 3/7/11; '
                             'full: PDF 1/3 (all at 100%% scale)')
    parser.add_argument('--span-mm', type=positive,
                        help='actual printed first-to-last bar span, preferred')
    parser.add_argument('--left-delta', type=int,
                        help='signed left count change during forward rolling')
    parser.add_argument('--right-delta', type=int,
                        help='signed right count change over the same distance')
    parser.add_argument('--distance-mm', type=positive, default=100.0)
    parser.add_argument('--gap-us', type=positive,
                        help='measured worst valid capture gap in microseconds')
    args = parser.parse_args()
    if args.span_mm is not None and args.layout is not None:
        parser.error('choose --span-mm or --layout, not both')
    if args.span_mm is None and args.layout is None:
        parser.error('supply the actual --span-mm or an explicit --layout')
    if (args.left_delta is None) != (args.right_delta is None):
        parser.error('supply both wheel deltas together')
    if args.left_delta == 0 or args.right_delta == 0:
        parser.error('both wheels must record travel during the forward pass')

    span = (args.span_mm if args.span_mm is not None else
            SPANS_MM[args.layout])
    # *letter*: 20 narrow + 9 wide runs, including the two symbol gaps.
    # Source artwork uses 3:1 wide/narrow, hence 20 + 9*3 = 47 units.
    narrow = span / 47.0
    print('Actual printed span' if args.layout is None else
          'Drawing span at 100% scale; verify on your print')
    print(f'Bar span: {span:.3f} mm')
    print(f'Nominal narrow bar/gap: {narrow:.3f} mm')
    print(f'Nominal wide bar/gap: {3 * narrow:.3f} mm')
    print('Run structure: 29 runs, 20 narrow + 9 wide, ratio 3:1')
    if args.gap_us is not None:
        speed = narrow / (4 * args.gap_us / 1000000)
        print(f'Four-sample speed ceiling at supplied gap: {speed:.3f} mm/s')
        print('This is a sampling calculation; validate speed on the robot.')
    if args.left_delta is not None:
        scale = (abs(args.left_delta) + abs(args.right_delta)) / args.distance_mm
        print(f'Sum encoder units/mm: {scale:.6f}')
        print(f'LEFT_FORWARD_SIGN: {1 if args.left_delta > 0 else -1:+d}')
        print(f'RIGHT_FORWARD_SIGN: {1 if args.right_delta > 0 else -1:+d}')
        print(f'Expected bar span: {span * scale:.3f} units')
        print(f'Expected narrow run: {narrow * scale:.3f} units')
        print(f'Expected wide run: {3 * narrow * scale:.3f} units')
        bound = math.floor(narrow * scale / 4)
        print(f'Conservative MAX_SAMPLE_UNITS ceiling: {bound}')
        if bound < 1:
            print('Insufficient encoder resolution for this conservative bound.')
    else:
        print('Wheel counts/mm remain unmeasured; encoder thresholds stay unset.')
    print('Measure polarity, quiet zones, noise, step/skew and jitter separately.')
    print('Choose acceptance tolerances from repeated valid hardware passes.')


if __name__ == '__main__':
    main()
