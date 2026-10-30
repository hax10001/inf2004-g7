"""Measure compact payload sizes; compare with unpacked equivalent structures.

These are sizeof measurements, not a claim about whole-firmware RAM savings.
--arm-cc and --arm-nm additionally measure the real ARM ABI without a board.
"""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "tmp/buddy5_tests"
SOURCE = r'''
#include <stdint.h>
#include <stdbool.h>
#include "modules/obstacle/obstacle.h"
#include "modules/line_barcode/ir_shared.h"
typedef struct { int32_t angle, range; } unpacked_point_t;
typedef struct { uint32_t phase, route, faults, count, distance, width; }
    unpacked_status_t;
typedef struct { uint32_t result; bool left; uint32_t angle, width, near,
    lateral, diagonal, fallback, left_clear, right_clear, direction; }
    unpacked_plan_t;
typedef struct { int32_t left, right; bool stop, follow, done, fault; }
    old_drive_t;
typedef struct { uint32_t left, right, barcode, timestamp; bool valid; }
    old_ir_t;
typedef struct { uint32_t ranges[3], valid, reason, epoch, timestamp; }
    unpacked_guard_t;
#define MEASURE(name, before, after) \
    uint8_t name##_before[sizeof(before)]; \
    uint8_t name##_after[sizeof(after)]
MEASURE(scan_sample, unpacked_point_t, scan_plan_point_t);
MEASURE(scan_array, unpacked_point_t[15], scan_plan_point_t[15]);
MEASURE(status, unpacked_status_t, uint32_t);
MEASURE(plan, unpacked_plan_t, scan_plan_bypass_t);
MEASURE(drive, old_drive_t, obstacle_drive_t);
MEASURE(ir, old_ir_t, ir_shared_snapshot_t);
MEASURE(live_guard, unpacked_guard_t, obs_guard_snapshot_t);
#ifndef ARM_SIZES
#include <stdio.h>
int main(void) {
#define PRINT(name) printf("%s %zu %zu\n", #name, sizeof(name##_before), \
    sizeof(name##_after))
    PRINT(scan_sample); PRINT(scan_array); PRINT(status);
    PRINT(plan); PRINT(drive); PRINT(ir); PRINT(live_guard);
    return 0;
}
#endif
'''


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--arm-cc")
    parser.add_argument("--arm-nm")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    source = OUT / "sizes.c"
    source.write_text(SOURCE)
    includes = ["-I" + str(ROOT / "tools/tests/stubs"),
                "-I" + str(ROOT / "app_program")]
    executable = OUT / "sizes.exe"
    run([args.cc, "-std=c99", *includes, str(source), "-o", str(executable)])
    host = {line.split()[0]: dict(zip(["before", "after"],
            map(int, line.split()[1:]))) for line in run([str(executable)]).splitlines()}
    output = {"basis": "Unpacked equivalents; drive and IR match the original layouts",
              "host": host}
    if args.arm_cc or args.arm_nm:
        if not (args.arm_cc and args.arm_nm):
            parser.error("Supply both --arm-cc and --arm-nm")
        obj = OUT / "sizes.o"
        run([args.arm_cc, "-std=c99", "-mcpu=cortex-m0plus", "-mthumb",
             "-DARM_SIZES", *includes, "-c", str(source), "-o", str(obj)])
        arm = {}
        for line in run([args.arm_nm, "-S", str(obj)]).splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[3].endswith(("_before", "_after")):
                name, side = parts[3].rsplit("_", 1)
                arm.setdefault(name, {})[side] = int(parts[1], 16)
        output["arm"] = arm
        assert arm == host, (arm, host)
    (OUT / "sizes.json").write_text(json.dumps(output, indent=2) + "\n")
    print("Payload bytes: unpacked/old -> compact (host" +
          (" and ARM agree" if "arm" in output else "") + ")")
    for name, sizes in host.items():
        print(f"{name}: {sizes['before']} -> {sizes['after']}")


if __name__ == "__main__":
    main()
