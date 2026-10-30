"""Format application C files, retaining BARR-C's final blank line."""
import argparse
from pathlib import Path
import shutil
import subprocess
import re

ROOT = Path(__file__).resolve().parents[1]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--clang-format", default=shutil.which("clang-format"))
args = parser.parse_args()
if not args.clang_format:
    parser.error("Install clang-format or pass --clang-format PATH")
sources = sorted(list((ROOT / "app_program").rglob("*.c")) +
                 list((ROOT / "app_program").rglob("*.h")))
excluded = {"demo_tasks.c", "demo_tasks.h", "usb_console_compat.h",
            "core/event.c", "core/event.h"}
sources = [p for p in sources if
           p.relative_to(ROOT / "app_program").as_posix() not in excluded]
subprocess.run([args.clang_format, "-i", *map(str, sources)], check=True)
for source in sources:
    text = source.read_text(encoding="utf-8")
    # clang-format 14 indents case-level break statements as case contents.
    # BARR-C 8.3.a instead aligns these breaks with their labels. A break
    # inside a nested loop or conditional is deliberately left untouched.
    case_indent = None
    lines = []
    for line in text.splitlines():
        label = re.match(r"^(\s*)(?:case\s+.+|default):\s*$", line)
        if label:
            case_indent = len(label.group(1))
        if (case_indent is not None and line.strip() == "break;" and
                len(line) - len(line.lstrip()) == case_indent + 4):
            line = " " * case_indent + "break;"
        lines.append(line)
    text = "\n".join(lines).rstrip() + "\n\n"
    source.write_text(text, encoding="utf-8", newline="\n")
print(f"Formatted {len(sources)} application files.")
