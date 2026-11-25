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
    # Team-written MQTT client kept beside the lwIP port in lib/.
    files += [ROOT / "lib/libnet/lwip/mqtt_utk.c",
              ROOT / "lib/libnet/lwip/include/mqtt_utk.h"]
    # Host unit tests and their stubs follow the same layout rules.
    tests = sorted(list((ROOT / "tools").glob("*.c")) +
                   list((ROOT / "tools/tests").rglob("*.c")) +
                   list((ROOT / "tools/tests").rglob("*.h")))
    files += tests
    for path in files:
        in_app = APP in path.parents
        host = ((in_app and "host" in path.relative_to(APP).parts) or
                (ROOT / "tools") in path.parents)
        name = str(path.relative_to(ROOT))
        data = path.read_bytes()
        text = data.decode("utf-8")
        if not host and re.fullmatch(r"[a-z][a-z0-9_]*", path.stem) is None:
            errors.append(f"{name}: invalid module name (4.1)")
        prefix = path.stem[:8]
        if not host and prefix in stems and stems[prefix] != path.stem:
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
        if path.suffix == ".c" and not host:
            if in_app and not path.with_suffix(".h").exists():
                errors.append(f"{name}: missing matching header (4.2.a)")
            if f'#include "{path.stem}.h"' not in text:
                errors.append(f"{name}: missing own header include (4.3.c)")
        elif path.suffix == ".h" and host:
            if not re.search(r"#ifndef (\w+)\n#define \1", text):
                errors.append(f"{name}: missing header guard (4.2.b)")
        elif path.suffix == ".h":
            guard = path.stem.upper() + "_H"
            if f"#ifndef {guard}\n#define {guard}" not in text:
                errors.append(f"{name}: missing header guard (4.2.b)")

        # Ignore comments and string literals before checking C tokens.
        code = re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|'
                      r"'(?:\\.|[^'\\])*'",
                      lambda m: re.sub(r"[^\n]", " ", m.group(0)),
                      text, flags=re.S)
        # Host test harnesses may exit, setjmp and use plain types.
        for token in (() if host else
                      ("auto", "register", "short", "long", "goto",
                       "continue")):
            if re.search(r"\b" + token + r"\b", code):
                errors.append(f"{name}: forbidden/review-required token {token}")
        if not host and re.search(r"\b(?:abort|exit|setjmp|longjmp)\s*\(",
                                  code):
            errors.append(f"{name}: forbidden jump function (8.5)")
        if re.search(r"\bwhile\s*\(\s*(?:1|true)\s*\)", code):
            errors.append(f"{name}: use for (;;) for infinite loops (8.4.c)")
        for macro in re.findall(r"^#define\s+(\w+)", text, flags=re.M):
            # Test stubs mirror vendor macro names (e.g. set_w, Inline).
            if macro != macro.upper() and "/stubs/" not in name:
                errors.append(f"{name}: lowercase macro {macro} (6.1.f)")
        for target in re.findall(r'^#include\s*[<"]([^>"]+)', text, re.M):
            if (target.startswith(("pico/", "hardware/")) or
                    "pico-sdk" in target or
                    (target.endswith(".c") and not host)):
                # Tests may include a module's .c to reach its statics.
                errors.append(f"{name}: forbidden SDK/source include {target}")
        if re.search(r"\b(?:sleep_ms|sleep_us|stdio_init_all|pwm_set_gpio_level|"
                     r"gpio_init|gpio_get|gpio_put|i2c_init)\s*\(", code):
            errors.append(f"{name}: Pico SDK peripheral API is forbidden")
        errors.extend(structure_errors(name, text, code))
    return files, errors


def _match(code, start, opening, closing):
    """Index of the bracket closing the one at start, or -1."""
    depth = 0
    for index in range(start, len(code)):
        if code[index] == opening:
            depth += 1
        elif code[index] == closing:
            depth -= 1
            if depth == 0:
                return index
    return -1


def _skip_space(code, index):
    while index < len(code) and code[index] in " \n":
        index += 1
    return index


def _in_macro(text, index):
    start = text.rfind("\n", 0, index) + 1
    if text[start:].lstrip().startswith("#"):
        return True
    previous = text.rfind("\n", 0, max(start - 1, 0)) + 1
    return start > 0 and text[previous:start - 1].rstrip().endswith("\\")


def structure_errors(name, text, code):
    """Rules checked on comment- and string-free code (same offsets)."""
    errors = []
    line_of = lambda index: code.count("\n", 0, index) + 1
    for match in re.finditer(r"\b(if|for|while|else|switch)\b", code):
        keyword = match.group(1)
        if _in_macro(text, match.start()):
            continue
        body = match.end()
        if keyword != "else":
            paren = _skip_space(code, match.end())
            if paren >= len(code) or code[paren] != "(":
                continue
            close = _match(code, paren, "(", ")")
            if close < 0:
                continue
            condition = code[paren + 1:close]
            if keyword in ("if", "while") and re.search(
                    r"(?<![=!<>+\-*/%&|^])=(?!=)", condition):
                errors.append(f"{name}:{line_of(match.start())}: "
                              f"assignment in {keyword} condition (8.2.c)")
            body = close + 1
        start = _skip_space(code, body)
        if start >= len(code):
            continue
        if keyword == "switch":
            end = _match(code, start, "{", "}")
            if not re.search(r"\bdefault\s*:", code[start:end]):
                errors.append(f"{name}:{line_of(match.start())}: "
                              "switch without default (8.3.b)")
            continue
        if keyword == "else" and re.match(r"if\b", code[start:]):
            continue
        # '#' : the body starts after a preprocessor branch (documented).
        if code[start] not in "{;#":
            errors.append(f"{name}:{line_of(match.start())}: "
                          f"{keyword} body without braces (1.3.a)")
    no_strings = re.sub(r'/\*.*?\*/|"(?:\\.|[^"\\])*"|'
                        r"'(?:\\.|[^'\\])*'",
                        lambda m: re.sub(r"[^\n]", " ", m.group(0)),
                        text, flags=re.S)
    for match in re.finditer(r"//", no_strings):
        errors.append(f"{name}:{line_of(match.start())}: "
                      "C++ style comment (2.1.a)")
    for index, line in enumerate(code.splitlines(), 1):
        stripped = line.strip()
        if re.match(r"^(?:static\s+|const\s+)*(?:u?int(?:8|16|32|64)_t|"
                    r"int|unsigned|bool|float|double|char)\s+[A-Za-z_]\w*\s*"
                    r"(?:=[^,;(){}]*)?,\s*[A-Za-z_]\w*[^;(){}]*;$",
                    stripped):
            errors.append(f"{name}:{index}: more than one declaration "
                          "per line (7.1)")
    for match in re.finditer(r"^#define\s+\w+\s+(-\d[\w.]*)\s*$", text,
                             re.M):
        errors.append(f"{name}:{line_of(match.start())}: unparenthesised "
                      "negative macro body (6.1.c)")
    if ("/host/" not in name and "/test_" not in name and
            not name.startswith("tools/")):
        for index, line in enumerate(code.splitlines(), 1):
            if (re.search(r"\b(?:int|unsigned|signed|short|long)\b", line)
                    and not re.search(r"\bint\s+main\s*\(|tm_netlog_|"
                                      r"unsigned int dropped|p_dropped|"
                                      r"\(int\)\(B1_LOG_CHUNK", line)):
                # Exceptions: main(), and the lib/libtm tm_netlog_take()
                # prototype and call, whose types are fixed by that API.
                errors.append(f"{name}:{index}: plain integer type; use "
                              "<stdint.h> fixed-width types (5.2.a)")
    errors.extend(function_header_errors(name, text, code))
    return errors


def function_header_errors(name, text, code):
    """Every function definition has a comment directly above it, or above
    its prototype in this file or its header (Barr-C 6.x function docs)."""
    errors = []
    texts = [text]
    header = ROOT / name
    if header.suffix == ".c" and header.with_suffix(".h").exists():
        texts.append(header.with_suffix(".h").read_text())
    for match in re.finditer(r"^(?:static\s+)?(?:inline\s+)?[A-Za-z_]"
                             r"[\w \t*]*?\b([A-Za-z_]\w*)\s*\(", code, re.M):
        function = match.group(1)
        if function in ("if", "while", "for", "switch", "return", "sizeof"):
            continue
        close = _match(code, match.end() - 1, "(", ")")
        if close < 0:
            continue
        brace = _skip_space(code, close + 1)
        if brace >= len(code) or code[brace] != "{":
            continue
        if text[:match.start()].rstrip().endswith("*/"):
            continue
        documented = False
        for source in texts:
            for proto in re.finditer(r"^[^\n;{}#]*\b" + re.escape(function) +
                                     r"\s*\([^;{]*\);", source, re.M):
                if source[:proto.start()].rstrip().endswith("*/"):
                    documented = True
        if not documented:
            errors.append(f"{name}:{code.count(chr(10), 0, match.start()) + 1}"
                          f": function {function} has no header comment")
    return errors


if __name__ == "__main__":
    checked, problems = check()
    for problem in problems:
        print(problem)
    print(f"Application standard checks: {len(checked)} files, "
          f"{len(problems)} errors.")
    sys.exit(bool(problems))
