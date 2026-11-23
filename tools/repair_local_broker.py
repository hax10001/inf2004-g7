#!/usr/bin/env python3
"""Repair the legacy generated local broker header without displaying secrets.

Runs only on a user-owned, gitignored config. Backs it up with 0600 permissions,
never copies it into project archives, and refuses unrecognized source directives.
"""
from pathlib import Path
import os
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "app_program/modules/telemetry/buddy1_broker_local.h"
GUARDS = ("B1_BROKER_LOCAL_H", "BUDDY1_BROKER_LOCAL_H")


def split_c_escaped(value):
    """Split one quoted C literal without bisecting backslash escapes."""
    assert value.startswith('"') and value.endswith('"')
    body = value[1:-1]
    tokens = re.findall(r"\\(?:x[0-9A-Fa-f]{2}|u[0-9A-Fa-f]{4}|[0-7]{1,3}|.)|[^\\]", body)
    if "".join(tokens) != body:
        raise ValueError("Cannot safely tokenize MQTT broker string")
    chunks = []
    current = ""
    for token in tokens:
        if current and len(current) + len(token) > 38:
            chunks.append('"' + current + '"')
            current = ""
        current += token
    if current:
        chunks.append('"' + current + '"')
    return chunks


def main():
    if not HEADER.exists():
        print("No local MQTT broker header found; run npm run setup:wifi first.")
        return 0
    original = HEADER.read_text(encoding="utf-8")
    if ("@file buddy1_broker_local.h" in original and
            "#ifndef BUDDY1_BROKER_LOCAL_H\n#define BUDDY1_BROKER_LOCAL_H" in original and
            original.endswith("/*** end of file ***/\n\n") and
            all(len(line) <= 80 for line in original.splitlines())):
        print("Existing local MQTT header is already standardized.")
        return 0
    defs = []
    for line in original.splitlines():
        stripped = line.strip()
        if (not stripped or stripped.startswith(("/*", "*", "//", "*/")) or
                stripped in ("#endif", "/*** end of file ***/")):
            continue
        if stripped in {f"#ifndef {g}" for g in GUARDS}:
            continue
        if stripped in {f"#define {g}" for g in GUARDS}:
            continue
        match = re.fullmatch(r"#define\s+(B1_[A-Z0-9_]+)\s+(.+)", stripped)
        if match is None:
            raise ValueError("Unrecognized local broker directive; left it unchanged")
        name, value = match.groups()
        if not (value.startswith('"') and value.endswith('"')):
            if len(f"#define {name} {value}") > 80:
                raise ValueError("Unrecognized long non-string macro; left it unchanged")
            defs.append(f"#define {name} {value}")
            continue
        segments = split_c_escaped(value)
        if len(segments) == 1 and len(f"#define {name} {value}") <= 80:
            defs.append(f"#define {name} {value}")
        else:
            defs.append(f"#define {name} \\")
            for index, segment in enumerate(segments):
                defs.append("    " + segment + (" \\" if index < len(segments) - 1 else ""))
    revised = ("/** @file buddy1_broker_local.h\n"
               " * @brief Private, locally configured MQTT connection values.\n"
               " */\n"
               "#ifndef BUDDY1_BROKER_LOCAL_H\n"
               "#define BUDDY1_BROKER_LOCAL_H\n" +
               "\n".join(defs) +
               "\n#endif\n/*** end of file ***/\n\n")
    if original == revised:
        print("Existing local MQTT header is already standardized.")
        return 0
    backup = HEADER.with_suffix(".h.before_style_fix.bak")
    if backup.exists():
        raise ValueError(f"Backup already exists: {backup.name}; refusing to overwrite")
    fd = os.open(backup, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(original)
    temp = HEADER.with_suffix(".h.style_tmp")
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(revised)
    os.replace(temp, HEADER)
    os.chmod(HEADER, 0o600)
    print("Fixed local MQTT header layout; secret values unchanged (backup retained).")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError) as exc:
        print(f"Local broker header not modified: {exc}", file=sys.stderr)
        sys.exit(1)
