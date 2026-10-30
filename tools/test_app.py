"""Build host regression tests; optionally compare pre-refactor call traces.

Usage on Windows: python tools/test_app.py --vcvars PATH/TO/vcvars64.bat
On POSIX: python3 tools/test_app.py --cc cc
"""
import argparse
import difflib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app_program"
TESTS = ROOT / "tools/tests"
OUT = ROOT / "tmp/app_tests"


def build(sources, includes, name, legacy, args, env, app_test=0):
    folder = OUT / name
    folder.mkdir(parents=True, exist_ok=True)
    executable = folder / (name + (".exe" if os.name == "nt" else ""))
    compiler = args.cc or ("cl" if args.vcvars else "cc")
    compiler = shutil.which(compiler, path=env.get("Path", env.get("PATH"))) or compiler
    if Path(compiler).stem.lower() == "cl":
        command = [compiler, "/nologo", "/std:c11", "/W3", "/D_CRT_SECURE_NO_WARNINGS",
                   f"/DLEGACY={int(legacy)}", f"/DAPP_TEST={app_test}",
                   *["/I" + str(p) for p in includes],
                   *map(str, sources), str(TESTS / "robot_harness.c"),
                   str(TESTS / "obstacle_harness.c"),
                   "/Fe" + str(executable), "/Fo" + str(folder) + os.sep]
    else:
        command = [compiler, "-std=c99", "-Wall", "-Wextra",
                   f"-DLEGACY={int(legacy)}", f"-DAPP_TEST={app_test}",
                   *["-I" + str(p) for p in includes],
                   *map(str, sources), str(TESTS / "robot_harness.c"),
                   str(TESTS / "obstacle_harness.c"),
                   "-lm", "-o", str(executable)]
    built = subprocess.run(command, env=env, capture_output=True, text=True)
    (folder / "build.log").write_text(built.stdout + built.stderr)
    if built.returncode:
        raise RuntimeError(built.stdout + built.stderr)
    return executable


def run(executable, scenario, variant=0):
    result = subprocess.run([str(executable), scenario, str(variant)],
                            capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc")
    parser.add_argument("--vcvars", type=Path)
    parser.add_argument("--baseline-json", type=Path)
    args = parser.parse_args()
    env = os.environ.copy()
    if args.vcvars:
        # The batch file configures the compiler, SDK headers and linker paths.
        setup = subprocess.run(
            f'cmd.exe /d /s /c "call "{args.vcvars}" >nul && set"',
            capture_output=True, text=True)
        if setup.returncode:
            raise RuntimeError(setup.stdout + setup.stderr)
        env.update(line.split("=", 1) for line in setup.stdout.splitlines()
                   if "=" in line and not line.startswith("="))

    production = sorted(p for p in APP.rglob("*.c")
                        if not p.name.startswith("test_")
                        and "tests" not in p.relative_to(APP).parts)
    includes = [TESTS / "stubs", APP]
    current = build(production, includes, "current", False, args, env)
    unit_output = run(current, "units")
    print(unit_output.strip())

    test_build = build(sorted(APP.rglob("*.c")), includes, "test_enabled",
                       False, args, env, app_test=1)
    on_target_simulation = run(test_build, "startup")
    assert "FAIL" not in on_target_simulation
    assert on_target_simulation.count(": PASS") == 15 #11
    print("PASS: TEST=1 host link and 11 non-mutating subsystem checks.")

    scenarios = ([("startup", i) for i in range(10)] + [("motor", 0)] +
                 [("controller", i) for i in range(10)] +
                 [("ir", i) for i in range(4)] +
                 [("imu", i) for i in range(10)])
    if args.baseline_json:
        snapshot = json.loads(args.baseline_json.read_text())
        legacy_dir = OUT / "baseline/app_program"
        legacy_dir.mkdir(parents=True, exist_ok=True)
        for name, content in snapshot.items():
            if "/" not in name and name.endswith((".c", ".h")):
                (legacy_dir / name).write_text(content, newline="\n")
        device = legacy_dir.parent / "device/include"
        device.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(TESTS / "stubs/dev_i2c.h", device / "dev_i2c.h")
        old_sources = [legacy_dir / name for name in
                       ("app_main.c", "controller_task.c", "ir_shared.c",
                        "ir_task.c", "line_follow.c", "motor_controller.c",
                        "imu_task.c")]
        baseline = build(old_sources, [TESTS / "stubs", legacy_dir],
                         "legacy", True, args, env)
        for scenario, variant in scenarios:
            before = run(baseline, scenario, variant)
            after = run(current, scenario, variant)
            (OUT / f"{scenario}_{variant}_before.txt").write_text(before)
            (OUT / f"{scenario}_{variant}_after.txt").write_text(after)
            if before != after:
                diff = "".join(difflib.unified_diff(
                    before.splitlines(True), after.splitlines(True),
                    fromfile="baseline", tofile="current"))
                raise AssertionError(f"{scenario}/{variant}:\n{diff}")
        print(f"PASS: {len(scenarios)} baseline/current call traces match.")
    else:
        expected = json.loads((TESTS / "expected_traces.json").read_text())
        for scenario, variant in scenarios:
            output = run(current, scenario, variant)
            key = f"{scenario}_{variant}"
            if output != expected[key]:
                diff = "".join(difflib.unified_diff(
                    expected[key].splitlines(True), output.splitlines(True),
                    fromfile="captured working baseline", tofile="current"))
                raise AssertionError(f"{key}:\n{diff}")
        print(f"PASS: {len(scenarios)} captured baseline call traces match.")
    print("Host tests do not qualify timing, peripherals, or physical robot behaviour.")


if __name__ == "__main__":
    main()
