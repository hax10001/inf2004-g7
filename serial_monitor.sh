#!/usr/bin/env bash
# Pico W serial console + dashboard mirror for macOS, Windows and Linux.
# Installs pyserial into an isolated local venv if it is missing.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$ROOT/.venv-serial"

if [[ -n "${SERIAL_PYTHON:-}" ]]; then
  PYTHON="$SERIAL_PYTHON"
elif [[ -x "$VENV/bin/python" ]]; then
  PYTHON="$VENV/bin/python"
elif [[ -x "$VENV/bin/python3" ]]; then
  PYTHON="$VENV/bin/python3"
elif python3 -c 'import serial; import serial.tools.list_ports' >/dev/null 2>&1; then
  PYTHON="python3"
else
  echo '[SERIAL] pyserial not installed; preparing a private project environment...' >&2
  if ! python3 -m venv "$VENV"; then
    echo '[SERIAL] Python venv setup failed. Check python3 and venv support.' >&2
    exit 2
  fi
  PYTHON="$VENV/bin/python"
fi

if ! "$PYTHON" -c 'import serial; import serial.tools.list_ports' >/dev/null 2>&1; then
  if [[ -n "${SERIAL_PYTHON:-}" ]]; then
    echo "[SERIAL] SERIAL_PYTHON lacks pyserial: $PYTHON" >&2
    echo "[SERIAL] Install it into that interpreter or unset SERIAL_PYTHON." >&2
    exit 2
  fi
  if [[ "$PYTHON" == "$VENV"/* ]]; then
    echo '[SERIAL] Installing pyserial into .venv-serial only (not system Python)...' >&2
    if ! "$PYTHON" -m pip install --disable-pip-version-check pyserial; then
      echo '[SERIAL] pyserial installation failed (check internet or pip).' >&2
      echo "[SERIAL] Retry: '$PYTHON' -m pip install pyserial" >&2
      exit 2
    fi
  else
    echo '[SERIAL] Install pyserial: python3 -m pip install pyserial' >&2
    exit 2
  fi
fi

echo '[SERIAL] Using:' "$PYTHON" >&2
echo '[SERIAL] Log is created by tee in your CURRENT folder, not a log directory.' >&2
exec "$PYTHON" "$ROOT/app_program/dashboard/tools/serial-to-dashboard.py" "$@"
