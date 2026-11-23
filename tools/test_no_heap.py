"""NFR7: application code shall not call malloc/calloc/realloc/free.

Scans the team's firmware sources under app_program/ (host-only test
harnesses and the dashboard are excluded). Reads files only.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
HEAP = re.compile(r"\b(malloc|calloc|realloc|free)\s*\(")


def strip(text):
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"', " ", text,
                  flags=re.S)


def main():
    hits = []
    files = 0
    for path in sorted((ROOT / "app_program").rglob("*.[ch]")):
        rel = path.relative_to(ROOT)
        if "host" in rel.parts or "dashboard" in rel.parts:
            continue
        if path.name.startswith("test_"):
            continue
        files += 1
        for n, line in enumerate(strip(path.read_text()).splitlines(), 1):
            if HEAP.search(line):
                hits.append(f"{rel}:{n}: {line.strip()}")
    if hits:
        print("\n".join(hits))
        sys.exit(1)
    print(f"PASS NFR7 no heap: no malloc/calloc/realloc/free in {files} "
          f"application files")


if __name__ == "__main__":
    main()
