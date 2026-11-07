"""Record barcode test passes from the robot's USB serial port.

Flash out/g7-barcode-test.uf2, connect USB, then run for example:

    python tools/barcode_capture.py COM5 big_A_push

Push the robot over the barcode. Each completed pass (through
[BC WAVE RAW END]) is saved as its own file:

    tools/barcode_data/<date>/big_A_push_01.log, _02.log, ...

While it runs, type a new label and press Enter to switch, for example
"small_A_push". Labels are size_LETTER_method; the replay tool reads the
letter from the second part to score the result. Ctrl+C stops.

Needs pyserial: python -m pip install pyserial
"""
import argparse
import datetime
import pathlib
import re
import sys
import threading

ROOT = pathlib.Path(__file__).resolve().parents[1]
LABEL_RE = re.compile(r"^[A-Za-z0-9-]+_[A-Za-z?]_[A-Za-z0-9-]+$")


def next_index(folder, label):
    used = [int(m.group(1)) for p in folder.glob(label + "_*.log")
            for m in [re.search(r"_(\d+)\.log$", p.name)] if m]
    return max(used, default=0) + 1


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("label", help="size_LETTER_method, e.g. big_A_push")
    parser.add_argument("--out", default=None,
                        help="output folder (default tools/barcode_data/<date>)")
    args = parser.parse_args()
    try:
        import serial  # noqa: PLC0415 - optional dependency
    except ImportError:
        sys.exit("pyserial is missing: python -m pip install pyserial")

    if not LABEL_RE.match(args.label):
        sys.exit("label must look like big_A_push (size_LETTER_method)")
    folder = pathlib.Path(args.out) if args.out else (
        ROOT / "tools" / "barcode_data" / datetime.date.today().isoformat())
    folder.mkdir(parents=True, exist_ok=True)
    session = folder / ("session_" + datetime.datetime.now().strftime("%H%M%S") + ".log")
    state = {"label": args.label}

    def read_labels():
        for line in sys.stdin:
            label = line.strip()
            if not label:
                continue
            if LABEL_RE.match(label):
                state["label"] = label
                print(f"--> label is now {label}")
            else:
                print("--> label must look like big_A_push")

    threading.Thread(target=read_labels, daemon=True).start()
    port = serial.Serial(args.port, 115200, timeout=0.5)
    print(f"Listening on {args.port}; saving to {folder}")
    print(f"Label: {state['label']} (type a new label + Enter to switch)")
    ready = ""
    current = None
    with open(session, "a", encoding="utf-8") as log:
        while True:
            try:
                raw = port.readline()
            except KeyboardInterrupt:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            log.write(line + "\n")
            if "[BC WAVE READY]" in line:
                ready = line
            if "[BC WAVE START]" in line:
                current = [ready] if ready else []
                print("... capturing")
            if current is not None:
                current.append(line)
                if "[BC WAVE RESULT]" in line or "[BARCODE] Ignored" in line:
                    print("   ", line.strip())
                if "[BC WAVE RAW END]" in line:
                    label = state["label"]
                    path = folder / f"{label}_{next_index(folder, label):02d}.log"
                    path.write_text("\n".join(current) + "\n", encoding="utf-8")
                    rows = sum(1 for l in current if "[BC WAVE RAW]" in l)
                    print(f"saved {path.name} ({rows} samples) - ready for the next pass")
                    current = None


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
