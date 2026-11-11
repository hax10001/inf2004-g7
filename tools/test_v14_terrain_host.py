#!/usr/bin/env python3
"""Compile the actual terrain snapshot mutex/event logic on GCC or Clang."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from host_compiler_flags import dead_code_flags

ROOT=Path(__file__).resolve().parents[1]
cc=os.environ.get("CC", "clang" if sys.platform=="darwin" else "gcc")
with tempfile.TemporaryDirectory(prefix="g7-v14-terrain-") as out:
    binary=str(Path(out)/"terrain_latch")
    cmd=[cc,"-std=c99","-O2","-Wall","-Wextra","-Werror",
        *dead_code_flags(cc),"-I",str(ROOT/"tools/tests/stubs"),
        "-I",str(ROOT),"-I",str(ROOT/"app_program"),
        str(ROOT/"app_program/modules/terrain/terrain.c"),
        str(ROOT/"tools/tests/test_terrain_collision_latch.c"),
        "-lm","-o",binary]
    subprocess.run(cmd,check=True,cwd=ROOT)
    subprocess.run([binary],check=True,cwd=ROOT)
