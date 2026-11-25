"""Run obstacle planner, clearance, controller, motor and telemetry checks."""
import argparse
import os
from pathlib import Path
import tempfile
import subprocess
import sys

from host_compiler_flags import dead_code_flags

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app_program"
OUT = Path(tempfile.gettempdir()) / "g7-obstacle-tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    subprocess.run([sys.executable, str(ROOT / "tools/tests/run_planner_side_tracking.py"),
                    "--cc", args.cc], cwd=ROOT, check=True)

    def build_run(name, sources, flags=()):
        executable = OUT / (name + (".exe" if os.name == "nt" else ""))
        command = [args.cc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-I", str(ROOT / "tools/tests/stubs"), "-I", str(ROOT),
                   "-I", str(APP), "-I", str(APP / "modules/telemetry"),
                   *flags, *[str(ROOT / path) for path in sources],
                   "-lm", "-o", str(executable)]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=ROOT, check=True)

    build_run("obstacle_watch", ["tools/tests/obstacle_watch.c"],
              dead_code_flags(args.cc, whole_program=True))
    build_run("obstacle_motion", ["tools/tests/obstacle_motion.c",
              "app_program/modules/motion/motion.c",
              "app_program/modules/motion/downhill.c",
              "app_program/modules/motion/steering.c",
              "app_program/modules/motion/wheel_speed.c"])
    build_run("telemetry", ["app_program/modules/telemetry/buddy1_protocol.c",
              "app_program/modules/telemetry/buddy1.c",
              "app_program/modules/telemetry/host/test_tel.c"], ["-pthread"])
    build_run("autonav", ["app_program/app/nav_core.c", "tools/test_autonav.c"])
    build_run("obstacle_return", ["tools/tests/obstacle_return.c"],
              dead_code_flags(args.cc, whole_program=True))
    build_run("obstacle_rejoin_sim", ["tools/tests/obstacle_rejoin_sim.c"],
              dead_code_flags(args.cc, whole_program=True))
    print("PASS: obstacle integration and existing navigation regressions.")


if __name__ == "__main__":
    main()
