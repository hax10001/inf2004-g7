"""Host-only regression tests; no physical movement is performed."""
from pathlib import Path
import argparse
import subprocess

ROOT = Path(__file__).resolve().parents[4]
APP = ROOT / "app_program"
OUT = ROOT / "tmp/obstacle_tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    sources = ["app/task_controller.c", "modules/line_barcode/line_follow.c"]
    sources += ["modules/obstacle/" + name + ".c" for name in
                ("obstacle", "obs_state", "scan_plan", "obs_geom", "task_obstacle")]
    exe = OUT / "obstacle.exe"
    subprocess.run([args.cc, "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-DAPP_OBSTACLE_ENABLED=1", "-DOBS_GEOMETRY_VERIFIED=1",
                    "-include", str(Path(__file__).with_name("rtos_flags.h")),
                    "-I" + str(ROOT / "tools/tests/stubs"), "-I" + str(APP),
                    *[str(APP / name) for name in sources],
                    str(Path(__file__).with_name("test_obstacle.c")),
                    "-o", str(exe)], check=True)
    cases = ["bad", "no_echo", "zero", "near20", "near25", "beyond", "spike",
             "startup", "fault", "no_route", "odometry_failure", "init_failure",
             "best_left", "best_right"]
    for case in cases:
        subprocess.run([str(exe), case], check=True, timeout=10)
    print(f"PASS: {len(cases)} host scenarios; hardware unverified.")


if __name__ == "__main__":
    main()
