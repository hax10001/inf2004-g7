"""Run deterministic Buddy 5 integration tests with the real C state machines.

Sensor values, odometry, motion execution and RTOS scheduling are simulated.
This does not establish hardware timing, clearance or mission performance.
"""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app_program"
OUT = ROOT / "tmp/buddy5_tests"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    executable = OUT / "buddy5.exe"
    sources = ["app/task_controller.c", "modules/line_barcode/ir_shared.c",
               "modules/line_barcode/line_follow.c", "modules/obstacle/obstacle.c",
               "modules/obstacle/obs_state.c", "modules/obstacle/scan_plan.c",
               "modules/obstacle/obs_geom.c",
               "modules/obstacle/test_obs.c",
               "modules/obstacle/task_obstacle.c"]
    command = [args.cc, "-std=c99", "-Wall", "-Wextra", "-Werror",
               "-DAPP_OBSTACLE_ENABLED=1", "-DOBS_GEOMETRY_VERIFIED=1",
               "-I" + str(ROOT / "tools/tests/stubs"),
               "-I" + str(APP), *[str(APP / p) for p in sources],
               str(ROOT / "tools/tests/buddy5_harness.c"), "-o", str(executable)]
    build = subprocess.run(command, capture_output=True, text=True)
    (OUT / "build.log").write_text(build.stdout + build.stderr)
    if build.returncode:
        raise RuntimeError(build.stdout + build.stderr)
    scenarios = ["profiles", "monitor", "monitor_bad", "monitor_recovered",
                 "guard", "guard_bad",
                 "scan_route", "scan_clear",
                 "scan_bad", "scan_abort", "left", "right", "wrap", "clear",
                 "no_route", "stall", "scan_timeout", "stale", "fault",
                 "odometry", "lost", "junction", "blocked_junction",
                 "encoder_wrap", "guard_all", "live_close", "live_bad",
                 "live_resume", "guard_timeout", "no_seen", "blind_outer",
                 "odom_stale", "one_wheel_stall", "return_close", "pass_close",
                 "front_wait", "front_recover", "rear_reappears"]
    output = []
    for scenario in scenarios:
        result = subprocess.run([str(executable), scenario], capture_output=True,
                                text=True, timeout=10)
        (OUT / (scenario + ".log")).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(scenario + ":\n" + result.stdout + result.stderr)
        print(result.stdout.strip())
        output.append(result.stdout)
    io_executable = OUT / "buddy5_io.exe"
    io_command = command[:command.index(str(APP / sources[0]))] + [
        str(APP / "modules/obstacle/ultrasonic.c"),
        str(APP / "modules/obstacle/servo.c"),
        str(APP / "modules/obstacle/obs_state.c"),
        str(APP / "modules/obstacle/scan_plan.c"),
        str(APP / "modules/obstacle/obs_geom.c"),
        str(ROOT / "tools/tests/buddy5_io_harness.c"), "-o", str(io_executable)]
    build = subprocess.run(io_command, capture_output=True, text=True)
    (OUT / "io_build.log").write_text(build.stdout + build.stderr)
    if build.returncode:
        raise RuntimeError(build.stdout + build.stderr)
    io_cases = ["echo", "echo_wrap", "no_echo", "stuck_echo", "too_far",
                "long_echo", "servo"]
    for scenario in io_cases:
        result = subprocess.run([str(io_executable), scenario], capture_output=True,
                                text=True, timeout=10)
        (OUT / (scenario + ".log")).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(scenario + ":\n" + result.stdout + result.stderr)
        print(result.stdout.strip())
        output.append(result.stdout)
    total = len(scenarios) + len(io_cases)
    uncalibrated = OUT / "buddy5_uncalibrated.exe"
    uncalibrated_command = [
        "-DOBS_GEOMETRY_VERIFIED=0" if arg == "-DOBS_GEOMETRY_VERIFIED=1"
        else str(uncalibrated) if arg == str(executable) else arg
        for arg in command]
    build = subprocess.run(uncalibrated_command, capture_output=True, text=True)
    if build.returncode:
        raise RuntimeError(build.stdout + build.stderr)
    result = subprocess.run([str(uncalibrated), "uncalibrated"],
                            capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    output.append(result.stdout)
    print(result.stdout.strip())
    total += 1
    summary = f"PASS: {total} Buddy 5 host scenarios. Hardware unverified."
    print(summary)
    (OUT / "results.txt").write_text("".join(output) + summary + "\n")


if __name__ == "__main__":
    main()
