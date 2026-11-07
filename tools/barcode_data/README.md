# Barcode test data

Raw barcode passes for offline decoder work. `codex/` holds the five passes
recovered from earlier Codex sessions (big F, small F, small A, two
unlabelled; `unlabelled_2` is a non-barcode capture).

## Recording passes

1. Build the test image (motors and obstacle detection compiled out):

       G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=0 G7_BARCODE_TEST=1 bash build_uf2.sh

   Flash `out/g7-barcode-test.uf2`. Flash `out/g7-improve-movement-mqtt.uf2`
   again afterwards for normal running.
2. Close any serial monitor, then log straight to a file (no installs):

       powershell -ExecutionPolicy Bypass -File tools\barcode_log.ps1 COM8 big_F_push

   This writes `tools\barcode_data\<date>\big_F_push.log` and shows the
   result lines (not the raw rows). Ctrl+C stops; rerun with a new name for
   the next barcode. The Pico is the "USB Serial Device" port.
3. Place the robot on white with the barcode sensor clear of the code and
   wait for `[BC WAVE READY]`. Push the robot steadily and straight past the
   barcode, at least 5 cm of white beyond it. Wait for
   `[BC WAVE DONE] Test mode: re-arming` (the raw dump takes a few seconds)
   before the next pass.
4. Save the serial output as one log per barcode and method, named
   `size_LETTER_method.log`, e.g. `big_F_push.log`, `small_A_push.log`, into
   a folder here. One file may hold any number of passes.

Optional: `tools/barcode_capture.py PORT label` (needs pyserial) does the
same and saves one file per pass.

Suggested session: 10 passes each of `big_X_push` and `small_X_push` for one
or two letters; then 10 normal driven passes of each (normal image, serial
log saved) for comparison.

## Replaying

    python tools/barcode_replay.py tools/barcode_data
    python tools/barcode_replay.py tools/barcode_data --sensor-left 70

`--sensor-left 70` measures distance at a sensor 70 mm left of the robot's
centre (test-mode logs only; they carry a left-wheel column). By default
both distances are tried and a disagreement is refused, as the firmware
does.

The firmware decoder itself (`app_program/modules/barcode_scanning/
bc_layout.c`) replays the same logs on the PC; its decisions should match
the Python replay:

    gcc -std=c99 -O2 -Iapp_program/modules/barcode_scanning \
        tools/tests/barcode_layout.c \
        app_program/modules/barcode_scanning/bc_layout.c \
        app_program/modules/barcode_scanning/bc_code.c -o layout_replay
    ./layout_replay tools/barcode_data/*/*.log

On the robot each pass prints `[BC LAYOUT] ... valid=1 letter=F` (or the
reason it refused) just before `[BC WAVE RESULT]`.
