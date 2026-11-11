#!/usr/bin/env python3
"""Invalidate legacy RP2040 Make objects if firmware input bytes changed.

Only touches generated .o/.d and image files under build_make; never touches
tracked .mk build rules, application source, SDK, Wi-Fi or MQTT secrets.
"""
from pathlib import Path
import hashlib
import os
import sys

SOURCES = ("app_program", "lib", "kernel", "device", "include", "config", "etc", "build_make")
SUFFIXES = {".c", ".h", ".s", ".S", ".inc", ".mk", ".ld", ".cpp", ".cxx"}


def fingerprint(root):
    digest = hashlib.sha256()
    paths = [root / "build_uf2.sh", root / "tools/g7_build_cache_guard.py"]
    for dirname in SOURCES:
        top = root / dirname
        if not top.is_dir():
            continue
        for path in top.rglob("*"):
            if not path.is_file() or path.is_symlink():
                continue
            if path.suffix not in SUFFIXES and path.name != "makefile":
                continue
            paths.append(path)
    for path in sorted(paths, key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "little"))
        digest.update(relative)
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def generated_objects(root):
    objects = root / "build_make" / "mtkernel_3"
    if objects.is_dir():
        for path in objects.rglob("*"):
            if path.is_file() and not path.is_symlink() and path.suffix in {".o", ".d"}:
                yield path
    build = root / "build_make"
    if build.is_dir():
        for path in build.iterdir():
            if (path.is_file() and not path.is_symlink()
                    and path.name.startswith("mtk3pico_")
                    and path.suffix in {".elf", ".uf2", ".map"}):
                yield path


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in {"prepare", "commit"}:
        raise SystemExit("usage: g7_build_cache_guard.py prepare|commit PROJECT_ROOT")
    root = Path(sys.argv[2]).expanduser().resolve()
    build = root / "build_make"
    if not build.is_dir() or not (build / "makefile").is_file():
        raise SystemExit("ERROR: build_make/makefile not found; check project root")
    state = build / ".g7_firmware_sources.sha256"
    current = fingerprint(root)
    if sys.argv[1] == "prepare":
        previous = state.read_text().strip() if state.is_file() else ""
        if previous != current:
            count = 0
            for path in generated_objects(root):
                path.unlink()
                count += 1
            if state.exists():
                state.unlink()
            print("G7 build cache: firmware inputs changed or not previously verified;")
            print("  removed", count, "generated ARM object/dependency/image files")
            print("  preserved source, .mk build rules and private Wi-Fi/MQTT headers")
        else:
            print("G7 build cache: verified firmware inputs unchanged")
    else:
        temp = state.with_name(state.name + ".tmp")
        temp.write_text(current + "\n")
        os.replace(temp, state)
        print("G7 build cache: verified source fingerprint recorded")


if __name__ == "__main__":
    main()
