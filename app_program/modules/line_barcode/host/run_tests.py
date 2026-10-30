"""Compile and run barcode tests in a temporary directory, without make.

Usage: python app_program/modules/line_barcode/host/run_tests.py
Use --cc PATH when GCC is not on PATH. No firmware or repository build
artifacts are changed. These simulations do not replace hardware trials.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("CC", "gcc"))
    args = parser.parse_args()
    compiler = shutil.which(args.cc)
    if compiler is None:
        raise SystemExit("GCC not found; pass --cc with the GCC executable")

    host = Path(__file__).resolve().parent
    module = host.parent
    repository = module.parents[2]
    with tempfile.TemporaryDirectory(prefix="barcode-host-") as temporary:
        executable = Path(temporary) / ("core.exe" if os.name == "nt" else "core")
        command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                   "-DBC_HOST_TEST", "-I", str(module),
                   str(module / "bc_decode.c"), str(module / "bc_scan.c"),
                   str(host / "test_core.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)

        service_sources = [module / name for name in
                           ("line_barcode.c", "ir_shared.c", "line_follow.c",
                            "bc_scan.c", "bc_decode.c")]
        service_sources.append(host / "test_service.c")
        service_options = [
            ("enabled", ["-DBC_CAL_READY=1", "-DBC_LEFT_FORWARD_SIGN=1",
                         "-DBC_RIGHT_FORWARD_SIGN=-1",
                         "-DBC_DIAGNOSTICS=0"]),
            ("invalid", ["-DBC_CAL_READY=1", "-DBC_LEFT_FORWARD_SIGN=0",
                         "-DBC_RIGHT_FORWARD_SIGN=0",
                         "-DBC_DIAGNOSTICS=0"]),
            ("uncalibrated", ["-DBC_CAL_READY=0", "-DBC_LEFT_FORWARD_SIGN=0",
                              "-DBC_RIGHT_FORWARD_SIGN=0",
                              "-DBC_DIAGNOSTICS=0"]),
            ("diagnostics", ["-DBC_CAL_READY=1", "-DBC_LEFT_FORWARD_SIGN=1",
                             "-DBC_RIGHT_FORWARD_SIGN=-1",
                             "-DBC_DIAGNOSTICS=1"]),
        ]
        for name, defines in service_options:
            executable = Path(temporary) / (
                name + (".exe" if os.name == "nt" else ""))
            command = [compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                       "-O2", "-DBC_HOST_SERVICE_TEST",
                       "-I", str(repository / "tools" / "tests" / "stubs"),
                       "-I", str(repository / "app_program")]
            command += defines + [str(path) for path in service_sources]
            command += ["-o", str(executable)]
            print("Service variant:", name, flush=True)
            subprocess.run(command, check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
