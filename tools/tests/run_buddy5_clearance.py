"""Run the reviewed scan planner and current side-tracking regressions.

The side state machine is tested using the production obstacle.c source
with unused embedded-only functions dropped by the host linker.
"""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "tools" / "tests"
APP = ROOT / "app_program"
OUT = ROOT / "tmp" / "buddy5_clearance"


def run(command):
    print("Building:", command[-1])
    subprocess.run(command, check=True)
    subprocess.run([command[-1]], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    run([args.cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
         "-I", str(APP),
         str(APP / "modules/obstacle/scan_plan.c"),
         str(TESTS / "buddy5_planner.c"),
         "-o", str(OUT / "buddy5_planner")])
    run([args.cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
         "-fwhole-program", "-ffunction-sections", "-fdata-sections",
         "-Wl,--gc-sections",
         "-I", str(TESTS / "stubs"), "-I", str(APP),
         str(TESTS / "buddy5_side_clearance.c"), "-lm",
         "-o", str(OUT / "buddy5_side_clearance")])
    print("PASS: reviewed planner and side-tracking regressions.")


if __name__ == "__main__":
    main()
