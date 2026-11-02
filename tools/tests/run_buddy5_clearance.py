"""Run the Buddy 5 host checks.

1. Geometry, servo mapping and planner checks, built with the robot's
   configuration and again with an offset mount and 75 degree limits.
2. Sequencer checks of side readings, rear-edge evidence, clearance and
   the return decision, for left and right bypasses.
3. The closed-loop simulation: the production obstacle task, sequencer,
   planner, geometry, state and servo code against a simulated car, beam
   and course, every scenario in both directions, in three builds.

These are host simulations. They do not qualify timing, peripherals or
the physical robot.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
TESTS = ROOT / "tools" / "tests"
APP = ROOT / "app_program"
OBS = APP / "modules" / "obstacle"
OUT = ROOT / "tmp" / "buddy5_clearance"
EXE = ".exe" if os.name == "nt" else ""

SCENARIOS = [
    "open", "left_blocked", "outward_corridor", "departure_late", "widens",
    "short", "long", "side_dropout", "quiet_face", "quiet_long",
    "reappears", "return_blocked", "crossing", "repeated", "both_blocked",
    "sensor_dead", "sensor_dead_pass", "servo_slow", "servo_stuck",
    "spikes",
]
OFFSET_MOUNT = ["-DOBS_SERVO_ARM_MM=20", "-DOBS_SENSOR_LATERAL_MM=10"]
SERVO_75 = ["-DOBS_SERVO_LEFT_LIMIT_DEG=75", "-DOBS_SERVO_RIGHT_LIMIT_DEG=75"]
SIM_BUILDS = {
    "default": [],
    "servo75": SERVO_75,
    "offset_mount": OFFSET_MOUNT,
}


def build(cc, name, sources, defines=(), libs=()):
    """Compile one host test; stop with the compiler output on failure."""
    OUT.mkdir(parents=True, exist_ok=True)
    executable = OUT / (name + EXE)
    command = [cc, "-std=c99", "-O1", "-Wall", "-Wextra", "-Werror",
               *defines, "-I", str(TESTS / "stubs"), "-I", str(APP),
               *map(str, sources), "-lm", *libs, "-o", str(executable)]
    built = subprocess.run(command, capture_output=True, text=True)
    if built.returncode:
        sys.exit("Build failed: " + name + "\n" + built.stdout + built.stderr)
    return executable


def run_unit(executable):
    """Run a self-checking test; return its output and success."""
    result = subprocess.run([str(executable)], capture_output=True, text=True)
    print(result.stdout.rstrip())
    if result.returncode:
        print(result.stderr.rstrip())
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    failures = []

    planner_sources = [OBS / "scan_plan.c", OBS / "obs_geometry.c",
                       OBS / "servo.c", TESTS / "buddy5_planner.c"]
    for name, defines in (("buddy5_planner", []),
                          ("buddy5_planner_mount75",
                           OFFSET_MOUNT + SERVO_75)):
        print("== " + name)
        if not run_unit(build(args.cc, name, planner_sources, defines)):
            failures.append(name)

    print("== buddy5_side_clearance")
    sequencer = build(args.cc, "buddy5_side_clearance",
                      [OBS / "obs_state.c", OBS / "obs_geometry.c",
                       OBS / "scan_plan.c", TESTS / "buddy5_side_clearance.c"])
    if not run_unit(sequencer):
        failures.append("buddy5_side_clearance")

    sim_sources = [OBS / "obstacle.c", OBS / "task_obstacle.c",
                   OBS / "scan_plan.c", OBS / "obs_geometry.c",
                   OBS / "obs_state.c", OBS / "servo.c",
                   TESTS / "buddy5_sim.c"]
    runs = 0
    for variant, defines in SIM_BUILDS.items():
        print("== simulation, " + variant + " build")
        simulator = build(args.cc, "buddy5_sim_" + variant, sim_sources,
                          defines, ["-lpthread"])
        for scenario in SCENARIOS:
            for mirror in ("0", "1"):
                result = subprocess.run([str(simulator), scenario, mirror],
                                        capture_output=True, text=True,
                                        timeout=600)
                runs += 1
                print(result.stdout.rstrip())
                if result.returncode:
                    failures.append(variant + "/" + scenario + "/" + mirror)

    print()
    if failures:
        print("FAILED: " + ", ".join(failures))
        sys.exit(1)
    print("PASS: Buddy 5 unit checks and %d closed-loop simulation runs "
          "(%d scenarios x left/right x %d builds)." %
          (runs, len(SCENARIOS), len(SIM_BUILDS)))
    print("Host simulations only: not a timing, peripheral or robot test.")


if __name__ == "__main__":
    main()
