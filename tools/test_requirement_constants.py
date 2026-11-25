"""Trace every numeric value in the Tribunal 2 requirements to its source.

docs/tribunal2/g7_pretribunal_requirements_proposed.md states numbers
(periods, thresholds, priorities, limits). This test reads the #define
that implements each one and fails if the code and the requirement drift
apart. It only reads source files; it changes nothing.
See docs/tribunal2/requirements_traceability.md.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

CFG = "app_program/app/app_config.h"
BOARD = "app_program/board/board_config.h"
OBS = "app_program/modules/obstacle/obs_config.h"
IMU = "app_program/modules/terrain/task_imu.c"
MOTION = "app_program/modules/motion/motion.c"
STEER = "app_program/modules/motion/steering.c"
B1 = "app_program/modules/telemetry/buddy1.c"
B1H = "app_program/modules/telemetry/buddy1.h"
B1P = "app_program/modules/telemetry/buddy1_protocol.h"
B1RT = "app_program/modules/telemetry/buddy1_rt.c"
MQTT = "lib/libnet/lwip/mqtt_utk.c"
WAVE = "app_program/modules/barcode_scanning/bc_wave.c"
FASTQ = "app_program/modules/barcode_scanning/bc_fastq.h"
FAST = "app_program/modules/barcode_scanning/bc_fast.c"
CAL = "app_program/modules/barcode_scanning/bc_cal.h"
ENC = "app_program/drivers/encoder.c"
CTRL = "app_program/app/task_controller.c"

# (requirement, file, macro, value stated by the requirement)
TRACE = [
    ("FR1", CFG, "APP_IR_MAX_AGE_MSEC", 200),
    ("FR3", IMU, "IMU_CALIBRATION_SAMPLES", 50),
    ("FR3", IMU, "IMU_CALIBRATION_MIN_MG", 800),
    ("FR3", IMU, "IMU_CALIBRATION_MAX_MG", 1200),
    ("FR3", OBS, "OBS_SERVO_FRONT_USEC", 1540),
    ("FR3", WAVE, "BC_WAVE_WHITE_SAMPLES", 250),
    ("FR4", STEER, "LINE_TRACK_TAPE_MM", 18),
    ("FR4", MOTION, "MOTION_TARGET_RPM", 30.0),
    ("FR4", MOTION, "MOTION_SEARCH_ROLL_MM", 50),
    ("FR4", MOTION, "MOTION_SEARCH_AFTER_MM", 100),
    ("FR5", BOARD, "BOARD_IR_BARCODE_PIN", 27),
    ("FR5", CFG, "APP_BARCODE_MIN_DARK_UNITS", 1000),
    ("FR6", CFG, "APP_NAV_JUNCTION_CONFIRM_MM", 7),
    ("FR6", CFG, "APP_NAV_BARCODE_TO_JUNCTION_MIN_COUNTS", 120),
    ("FR6", CFG, "APP_NAV_IR_TO_AXLE_MM", 125),
    ("FR6", CFG, "APP_NAV_DECODE_WAIT_MSEC", 8000),
    ("FR6", CFG, "APP_NAV_DECODE_GRACE_MSEC", 1500),
    ("FR7", MOTION, "MOTION_KP", 0.1),
    ("FR7", MOTION, "MOTION_KI", 0.5),
    ("FR7", MOTION, "MOTION_RAMP_RPM", 8.0),
    ("FR7", BOARD, "BOARD_ENC_COUNTS_PER_REV", 1056),
    ("FR7", BOARD, "BOARD_WHEEL_DIAMETER_MM", 70.0),
    ("FR7", CFG, "APP_JUNCTION_BRAKE_MSEC", 200),
    ("FR8", IMU, "HUMP_START_DEG", 3.0),
    ("FR8", IMU, "HUMP_TILT_CONFIRM_SAMPLES", 3),
    ("FR8", IMU, "IMU_SAMPLE_PERIOD_MSEC", 20),
    ("FR8", IMU, "HUMP_LEVEL_DEG", 1.5),
    ("FR8", IMU, "HUMP_LEVEL_CONFIRM_SAMPLES", 15),
    ("FR8", IMU, "HUMP_PUBLISH_MAX_MM", 127),
    ("FR8", IMU, "HUMP_MIN_CLIMB_MM", 0.0),  # 24 Nov: minimum removed
    ("FR8", IMU, "HUMP_HEIGHT_BIAS_MM", 2.0),  # 25 Nov: chosen by Linus
    ("FR8", IMU, "HUMP_MIN_TRAVEL_MM", 40),
    ("FR8", IMU, "HUMP_AFTER_LIFT_MSEC", 3000),
    ("FR9", OBS, "OBS_STOP_MM", 250),
    ("FR9", OBS, "OBS_CONFIRM_HITS", 3),
    ("FR9", OBS, "OBS_MEDIAN_SAMPLES", 3),
    ("FR10", OBS, "OBS_SERVO_LEFT_LIMIT_DEG", 60),
    ("FR10", OBS, "OBS_SERVO_RIGHT_LIMIT_DEG", 70),
    ("FR10", OBS, "OBS_SCAN_FINE_STEP_DEG", 5),
    ("FR10", OBS, "OBS_SCAN_STEP_DEG", 10),
    ("FR10", OBS, "OBS_EDGE_VOTE_MIN_DEG", 20),
    ("FR10", OBS, "OBS_EDGE_VOTE_MAX_DEG", 45),
    ("FR10", OBS, "OBS_EDGE_VOTE_PINGS", 5),
    ("FR10", OBS, "OBS_SAFETY_MARGIN_MM", 40),
    ("FR10", OBS, "OBS_MIN_LATERAL_MM", 245),
    ("FR10", OBS, "OBS_SIDE_PREFERENCE_THRESHOLD_MM", 15),
    ("FR10", OBS, "OBS_RESUME_HITS", 3),
    ("FR10", OBS, "OBS_RESUME_CLEAR_MM", 400),
    ("FR11", OBS, "OBS_SIDE_CLEAR_HITS", 5),
    ("FR11", OBS, "OBS_PASS_MAX_MM", 600),
    ("FR11", OBS, "OBS_RETURN_DEG", 60),
    ("FR11", OBS, "OBS_VERIFY_CYCLES", 10),
    ("FR11", OBS, "OBS_ALIGN_CHECK_DEG", 30),
    ("FR12", B1, "B1_STATUS_PERIOD_MS", 1000),
    ("FR12", B1, "B1_SNAPSHOT_PERIOD_MS", 250),
    ("FR12", B1, "B1_HEARTBEAT_PERIOD_MS", 2000),
    ("FR12", B1H, "B1_CHECKPOINT_MM", 500),
    ("FR13", B1P, "B1_CMD_MAPPING", 1),
    ("FR13", B1P, "B1_CMD_RECAL_BARCODE", 6),
    ("FR14", CFG, "APP_NAV_TRACK_MM", 130),
    ("FR15", MQTT, "B1_PING_PERIOD_MS", 5000),
    ("FR15", MQTT, "B1_LINK_SILENCE_MS", 15000),
    ("FR15", B1H, "B1_MOTION_DEPTH", 32),
    ("FR15", B1H, "B1_EVENT_DEPTH", 24),
    ("FR16", CFG, "APP_LIFT_RESUME_MSEC", 1000),
    ("FR17", CFG, "APP_LIFT_STOP_HEIGHT_MM", 40),
    ("FR17", CFG, "APP_LIFT_RESUME_MSEC", 1000),
    ("FR18", IMU, "HUMP_DESCENT_CLIMB_MM", 3.0),
    ("FR18", CFG, "APP_DOWNHILL_REDUCED_PERCENT", 65),
    ("FR18", CFG, "APP_DOWNHILL_RUNAWAY_RPM", 40.0),
    ("FR18", CFG, "APP_DOWNHILL_NOSE_DOWN_SIGN", 0),
    ("NFR2", CFG, "APP_IR_PRIORITY", 5),
    ("NFR2", CFG, "APP_IR_PERIOD_MSEC", 1),
    ("NFR2", FASTQ, "BC_FAST_PERIOD_USEC", 2000),
    ("NFR2", FASTQ, "BC_FASTQ_CAPACITY", 64),
    ("NFR3", CFG, "APP_CONTROLLER_PRIORITY", 6),
    ("NFR3", CFG, "APP_CONTROLLER_PERIOD_MSEC", 50),
    ("NFR3", CFG, "APP_REMOTE_LOCK_WAIT_MSEC", 2),
    ("NFR3", CFG, "APP_REMOTE_LOCK_MISS_LIMIT", 4),
    ("NFR4", CFG, "APP_IMU_PRIORITY", 7),
    ("NFR4", IMU, "IMU_SAMPLE_PERIOD_MSEC", 20),
    ("NFR4", IMU, "IMU_FILTER_ALPHA", 0.2),
    ("NFR4", CTRL, "CTRL_IMU_FRESH_MSEC", 200),
    ("NFR4", CTRL, "CTRL_IMU_LOST_MSEC", 1000),
    ("NFR5", OBS, "OBS_ECHO_TIMEOUT_MSEC", 30),
    ("NFR5", OBS, "OBS_ECHO_CLEAR_MSEC", 50),
    ("NFR5", OBS, "OBS_PING_PERIOD_MSEC", 60),
    ("NFR6", CFG, "APP_BARCODE_ADC_THRESHOLD", 2200),
    ("NFR6", CFG, "APP_BARCODE_ADC_HYSTERESIS", 50),
    ("NFR6", CFG, "APP_BARCODE_TRIGGER_RISE", 600),
    ("NFR8", B1RT, "B1_TASK_PRIORITY", 9),
    ("NFR8", CAL, "BC_CAL_DIAG_PRIORITY", 9),
    ("NFR9", CFG, "APP_JUNCTION_BRAKE_MSEC", 200),
    ("NFR10", ENC, "ENC_INT_PRIORITY", 2),
    ("NFR10", FAST, "BC_FAST_PRIORITY", 2),
    ("NFR13", B1P, "B1_HEADER", 13),
]

NUMBER = re.compile(r"^\(?\s*(-?\d+(?:\.\d+)?)[uUlLfF]*\s*\)?$")


def define_values(path):
    """Return every literal value #defined for each macro in a file."""
    values = {}
    text = (ROOT / path).read_text()
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    for match in re.finditer(r"^\s*#\s*define\s+(\w+)[ \t]+([^\n]+)$",
                             text, re.M):
        literal = NUMBER.match(match.group(2).strip())
        if literal:
            values.setdefault(match.group(1), []).append(
                float(literal.group(1)))
    return values


def main():
    cache = {}
    failures = []
    for req, path, macro, expected in TRACE:
        values = cache.setdefault(path, define_values(path)).get(macro)
        if not values:
            failures.append(f"{req}: {macro} not found in {path}")
        elif all(abs(v - expected) > 1e-9 for v in values):
            failures.append(f"{req}: {macro} is {values} in {path}, "
                            f"requirement says {expected}")
    if failures:
        print("\n".join(failures))
        sys.exit(1)
    reqs = sorted({t[0] for t in TRACE},
                  key=lambda r: (r.startswith("NFR"), int(r.lstrip("NFR"))))
    print(f"PASS requirement constants: {len(TRACE)} values traced to "
          f"source for {len(reqs)} requirements ({', '.join(reqs)})")


if __name__ == "__main__":
    main()
