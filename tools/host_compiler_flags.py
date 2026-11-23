"""Portable linker- and compiler-flags for host-side embedded tests.

The project's host fixtures include embedded production sources whose unused
firmware-only functions must be discarded at link time. GNU and Apple linkers
use different dead-stripping options, and Apple Clang rejects -fwhole-program.
This affects host tests only; Pico/ARM production build flags are untouched.
"""
import subprocess
import sys


def is_clang(compiler):
    """Detect whether a compiler executable is Clang (including macOS gcc)."""
    try:
        version = subprocess.run([compiler, "--version"], check=True,
                                 capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return sys.platform == "darwin"
    return "clang" in (version.stdout + version.stderr).lower()


def dead_code_flags(compiler, whole_program=False):
    """Select equivalent test dead-strip semantics for GNU and Apple hosts."""
    flags = []
    if whole_program and not is_clang(compiler):
        flags.append("-fwhole-program")
    flags.extend(["-ffunction-sections", "-fdata-sections"])
    if sys.platform == "darwin":
        flags.append("-Wl,-dead_strip")
    else:
        flags.append("-Wl,--gc-sections")
    return flags
