"""Check mechanically enforceable BARR-C:2018 rules in application sources.

This is a focused project checker, not a complete BARR-C compliance tool.
It deliberately excludes upstream kernel/BSP code and reference text files.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app_program"


def check():
    errors = []
    stems = {}
    files = sorted(list(APP.rglob("*.c")) + list(APP.rglob("*.h")))
    for path in files:
        if "host" in path.relative_to(APP).parts:
            continue
        name = str(path.relative_to(ROOT))
        data = path.read_bytes()
        text = data.decode("utf-8")
        if re.fullmatch(r"[a-z][a-z0-9_]*", path.stem) is None:
            errors.append(f"{name}: invalid module name (4.1)")
        prefix = path.stem[:8]
        if prefix in stems and stems[prefix] != path.stem:
            errors.append(f"{name}: first-eight-character collision (4.1.b)")
        stems[prefix] = path.stem
        if b"\t" in data or b"\r" in data:
            errors.append(f"{name}: tabs or CR line endings (3.5, 3.6)")
        if not text.endswith("/*** end of file ***/\n\n"):
            errors.append(f"{name}: missing EOF marker/blank line (3.3.c)")
        for index, line in enumerate(text.splitlines(), 1):
            if len(line) > 80:
                errors.append(f"{name}:{index}: line exceeds 80 columns (1.2)")
            if line.rstrip() != line:
                errors.append(f"{name}:{index}: trailing whitespace")
            if any(ord(ch) < 32 and ch != "\f" for ch in line):
                errors.append(f"{name}:{index}: nonprinting character (3.6)")
        if "@file " + path.name not in text or "@brief" not in text:
            errors.append(f"{name}: missing module documentation (2.2.h)")
        if path.suffix == ".c":
            if not path.with_suffix(".h").exists():
                errors.append(f"{name}: missing matching header (4.2.a)")
            if f'#include "{path.stem}.h"' not in text:
                errors.append(f"{name}: missing own header include (4.3.c)")
        else:
            guard = path.stem.upper() + "_H"
            if f"#ifndef {guard}\n#define {guard}" not in text:
                errors.append(f"{name}: missing header guard (4.2.b)")

        # Ignore comments and string literals before checking C tokens.
        code = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', " ",
                      text, flags=re.S)
        for token in ("auto", "register", "short", "long", "goto", "continue"):
            if re.search(r"\b" + token + r"\b", code):
                errors.append(f"{name}: forbidden/review-required token {token}")
        if re.search(r"\b(?:abort|exit|setjmp|longjmp)\s*\(", code):
            errors.append(f"{name}: forbidden jump function (8.5)")
        if re.search(r"\bwhile\s*\(\s*(?:1|true)\s*\)", code):
            errors.append(f"{name}: use for (;;) for infinite loops (8.4.c)")
        for macro in re.findall(r"^#define\s+(\w+)", text, flags=re.M):
            if macro != macro.upper():
                errors.append(f"{name}: lowercase macro {macro} (6.1.f)")
        for target in re.findall(r'^#include\s*[<"]([^>"]+)', text, re.M):
            if (target.startswith(("pico/", "hardware/")) or
                    "pico-sdk" in target or target.endswith(".c")):
                errors.append(f"{name}: forbidden SDK/source include {target}")
        if re.search(r"\b(?:sleep_ms|sleep_us|stdio_init_all|pwm_set_gpio_level|"
                     r"gpio_init|gpio_get|gpio_put|i2c_init)\s*\(", code):
            errors.append(f"{name}: Pico SDK peripheral API is forbidden")
    return files, errors


if __name__ == "__main__":
    checked, problems = check()
    for problem in problems:
        print(problem)
    print(f"Application standard checks: {len(checked)} files, "
          f"{len(problems)} errors.")
    sys.exit(bool(problems))
