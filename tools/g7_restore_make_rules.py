#!/usr/bin/env python3
"""Restore the immutable micro T-Kernel subdir.mk build scaffold only.

V6 mistakenly removed build_make/mtkernel_3, which holds *rules* as well as .o
files. Recover the exact bundled rule files; never overwrite custom rules,
source, secrets, compiled ELF/UF2, or any existing project file.
"""
from pathlib import Path, PurePosixPath
import hashlib
import re
import sys
import zipfile

ARCHIVE_SHA256 = '784ec3be05c722c905092f6f7d4654cf0f8e68f6b231ed555517feb7d4921dca'

def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 tools/g7_restore_make_rules.py PROJECT_ROOT')
    root = Path(sys.argv[1]).expanduser().resolve()
    base = root / 'build_make'
    if not (base / 'makefile').is_file() or not (base / 'pico_rp2040.mk').is_file():
        raise SystemExit('STOP: expected legacy Makefiles missing; check project directory')
    archive = Path(__file__).resolve().with_name('g7_make_scaffold.zip')
    if not archive.is_file() or hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise SystemExit('STOP: bundled Makefile scaffold is missing or altered')
    restored = []
    preserved = []
    source_text = []
    with zipfile.ZipFile(archive) as z:
        names = z.namelist()
        if len(names) != 67 or len(set(names)) != 67:
            raise SystemExit('STOP: unexpected scaffold layout')
        for name in names:
            path = PurePosixPath(name)
            if (not name.startswith('build_make/mtkernel_3/')
                    or path.is_absolute() or '..' in path.parts or path.name != 'subdir.mk'):
                raise SystemExit('STOP: untrusted scaffold member')
            dest = root.joinpath(*path.parts)
            # No symlink traversal, including parent directories.
            if any(p.is_symlink() for p in (dest, *dest.parents) if p != root.parent and p != root.parent.parent):
                raise SystemExit('STOP: symlink within build scaffold target: ' + name)
            data = z.read(name)
            if dest.exists():
                if not dest.is_file():
                    raise SystemExit('STOP: unexpected non-file scaffold target: ' + name)
                # A custom Makefile is user-owned; don't overwrite it.
                preserved.append(name)
                source_text.append(dest.read_text(errors='replace'))
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
                restored.append(name)
                source_text.append(data.decode('utf-8', 'replace'))

    # Static legacy recipes omit @mkdir -p. Ensure every object destination
    # directory exists even after a clean/first-time build.
    source_text.append((base/'pico_rp2040.mk').read_text(errors='replace'))
    obj_re = re.compile(r'(?:\./)?mtkernel_3/[a-zA-Z0-9_./%-]+\.(?:o|d)')
    directories = set()
    for s in source_text:
        for match in obj_re.findall(s):
            rel = PurePosixPath(match.removeprefix('./'))
            if '..' in rel.parts or '%' in rel.parts:
                continue
            directory = base.joinpath(*rel.parent.parts)
            directories.add(directory)
    directories.add(base/'mtkernel_3/app_program/obj_test0')
    directories.add(base/'mtkernel_3/lib/libtm/sysdepend/pico_rp2040/usb')
    directories.add(base/'mtkernel_3/lib/libnet/lwip')
    for directory in sorted(directories):
        if directory.is_symlink():
            raise SystemExit('STOP: symlinked output directory: ' + str(directory))
        directory.mkdir(parents=True, exist_ok=True)
    print('Build rules: restored %d missing subdir.mk; preserved %d existing' % (len(restored), len(preserved)))
    print('Prepared %d build output directories; source/config/credentials unchanged' % len(directories))

if __name__ == '__main__':
    main()
