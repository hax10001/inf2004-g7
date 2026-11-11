"""Run deterministic IMU startup tests without robot hardware."""
from pathlib import Path
import argparse
import subprocess

ROOT = Path(__file__).resolve().parents[4]
APP = ROOT / "app_program"
HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    out = ROOT / "tmp/terrain_tests"
    out.mkdir(parents=True, exist_ok=True)
    exe = out / "calibration.exe"
    subprocess.run([args.cc, "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(ROOT / "tools/tests/stubs"), "-I" + str(APP),
                    *[str(APP / "modules/terrain" / name) for name in
                      ("task_imu.c", "terrain.c", "tilt_math.c")],
                    str(HERE / "test_calibration.c"), "-lm", "-o", str(exe)],
                   check=True)
    for case in ("steady", "read_failure", "movement", "magnitude", "persistent"):
        subprocess.run([str(exe), case], check=True, timeout=5)


if __name__ == "__main__":
    main()
