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
    sources = ["app/task_controller.c", "modules/line_barcode/line_follow.c",
               "modules/line_barcode/ir_shared.c"]
    sources += ["modules/obstacle/" + name + ".c" for name in
                ("obstacle", "obs_state", "scan_plan", "obs_geom", "task_obstacle")]
    exe = OUT / "obstacle.exe"
    command = [args.cc, "-std=c99", "-Wall", "-Wextra", "-Werror",
                    "-DAPP_OBSTACLE_ENABLED=1", "-DOBS_GEOMETRY_VERIFIED=1",
                    "-include", str(Path(__file__).with_name("rtos_flags.h")),
                    "-I" + str(ROOT / "tools/tests/stubs"), "-I" + str(APP),
                    *[str(APP / name) for name in sources],
                    str(Path(__file__).with_name("test_obstacle.c")),
                    "-o", str(exe)]
    build = subprocess.run(command, capture_output=True, text=True)
    (OUT / "build.log").write_text(build.stdout + build.stderr)
    build.check_returncode()
    cases = ["profiles", "front_freshness", "boundary_200", "boundary_250",
             "boundary_251", "boundary_299", "boundary_300", "boundary_301",
             "init_bad", "task_bad", "left", "right", "no_route", "untrusted_plan", "fault",
             "monitor_bad", "monitor_zero", "monitor_range_bad", "front_wait",
             "front_recover", "guard_all", "live_close", "live_bad", "live_resume",
             "guard_timeout", "no_seen", "blind_outer", "odom_stale",
             "one_wheel_stall", "return_close", "pass_close", "rear_reappears",
             "pass_drift", "sideways_drift", "verify_white", "lost",
             "early_line", "junction", "transverse_line", "stale", "odometry", "blocked_junction",
             "encoder_wrap", "wrap", "clear", "stall", "scan_timeout",
             "scan_route", "scan_clear", "scan_bad", "scan_abort", "guard",
             "guard_bad", "monitor_recovered", "monitor"]
    failed = []
    for case in cases:
        result = subprocess.run([str(exe), case], capture_output=True,
                                text=True, timeout=10)
        (OUT / (case + ".log")).write_text(result.stdout + result.stderr)
        if result.returncode:
            print(f"FAIL {case}:\n{result.stdout}{result.stderr}")
            failed.append(case)
        else:
            print(result.stdout.strip())
    if failed:
        (OUT / "results.txt").write_text(
            f"FAIL: {len(failed)}/{len(cases)} host scenarios: " +
            ", ".join(failed) + "\n")
        raise RuntimeError("Failed host scenarios: " + ", ".join(failed))
    summary = f"PASS: {len(cases)} host scenarios; hardware unverified."
    (OUT / "results.txt").write_text(summary + "\n")
    print(summary)


if __name__ == "__main__":
    main()
