"""Analyse application C for Cortex-M0+ against the real kernel/BSP headers.

Pass --clang-tidy and optionally --standard-include for a target sysroot's
standard headers. With no sysroot, use minimal freestanding C99 integer and
Boolean headers defined from Clang's target builtins, never host CRT types.
This checks parsing and selected analyzer diagnostics; it does not link firmware.
"""
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tmp/arm_analysis"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang-tidy", default=shutil.which("clang-tidy"))
    parser.add_argument("--standard-include", type=Path)
    args = parser.parse_args()
    if not args.clang_tidy:
        parser.error("Install clang-tidy or pass --clang-tidy PATH")
    OUT.mkdir(parents=True, exist_ok=True)
    standard = args.standard_include or OUT / "freestanding"
    if not args.standard_include:
        standard.mkdir(exist_ok=True)
        definitions = ["#ifndef VALIDATION_STDINT_H", "#define VALIDATION_STDINT_H"]
        for width in (8, 16, 32, 64):
            for prefix in ("", "U"):
                spelling = "uint" if prefix else "int"
                definitions.append(
                    f"typedef __{prefix}INT{width}_TYPE__ {spelling}{width}_t;")
                definitions.append(
                    f"#define {prefix}INT{width}_MAX __{prefix}INT{width}_MAX__")
        definitions.append("#endif")
        (standard / "stdint.h").write_text("\n".join(definitions) + "\n")
        (standard / "stdbool.h").write_text(
            "#ifndef VALIDATION_STDBOOL_H\n#define VALIDATION_STDBOOL_H\n"
            "#define bool _Bool\n#define true 1\n#define false 0\n#endif\n")

    sources = sorted((ROOT / "app_program").rglob("*.c"))
    for smp in (0, 1):
        for usb in (0, 1):
            profile = f"smp{smp}_usb{usb}"
            command = [args.clang_tidy, *map(str, sources),
                       "--checks=-*,clang-analyzer-core.*", "--quiet", "--",
                       "--target=arm-none-eabi", "-mcpu=cortex-m0plus", "-mthumb",
                       "-ffreestanding", "-std=c99", "-nostdinc",
                       "-Wall", "-Wextra", "-Wconversion", "-Wsign-conversion",
                       "-Wshadow", "-Wmissing-prototypes", "-D_PICO_RP2040_",
                       "-DAPP_TEST=1", f"-DCNF_SMP={smp}",
                       f"-DTM_CONSOLE_USB_CDC={usb}", "-isystem", str(standard),
                       "-isystem", str(ROOT / "include"),
                       "-isystem", str(ROOT / "config"),
                       "-I" + str(ROOT / "kernel/knlinc"),
                       "-I" + str(ROOT / "app_program"),
                       "-I" + str(ROOT / "device/include")]
            result = subprocess.run(command, capture_output=True, text=True)
            log = result.stdout + result.stderr
            (OUT / f"{profile}.log").write_text(log)
            if result.returncode or "warning:" in log or "error:" in log:
                raise RuntimeError(f"{profile}:\n{log}")
            print(f"PASS: {profile}, {len(sources)} application translation units.")


if __name__ == "__main__":
    main()
