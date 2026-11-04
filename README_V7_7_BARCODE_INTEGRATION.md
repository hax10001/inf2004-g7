# V7.7 barcode owner integration and navigation cleanup

The user-supplied `inf2004-g7-barcode.zip` is authoritative for the
`app_program/modules/barcode_scanning/` engine. `bc_fast` captures GP27 and
wheel counts with TIMER ALARM2 every 2 ms (the incoming handoff references an
earlier 4 ms profile; the source currently specifies 2000 us).

- Kept existing motion, terrain, IR-sharing and MQTT ownership. No duplicate decoder.
- New barcode worker uses `bc_setup_init/start`; IR task calls `bc_setup_sample`
  to arm it once. It does not decode inside the IR sampling task.
- Telemetry consumes `bc_port_peek`, publishes real letters to MQTT, and ACKs
  its mailbox only after ingestion. `BC_CAL_AUTO_ACK` is disabled.
- New cumulative rejected/completed counters are sent with the existing
  28-byte barcode MQTT diagnostic packet; source is the worker's counter,
  not synthetic IR edges. The dashboard reports genuine scanner failures.
- Replaced the top-right Connect button with a compact Route mapping control.
  Connection remains automatic; clicking the MQTT status opens the same
  diagnostic dialog.
- Bench-only barcode flags: `BC_CAL_TEST_IGNORE_INPUT_ERRORS=1` and
  `BC_CAL_TEST_RELAXED_CAPTURE=1` are as supplied in the working barcode ZIP.
  They relax rejection rules and MUST be reviewed before production navigation.
- Full ARM compilation and live Pico hardware have not been verified here.
- Uploaded scanner's original native test suite has a failing assertion
  `extreme short narrow run still rejected`; this is a pre-existing test
  failure in the received ZIP, not evidence it passes every case.
- The scanner does NOT by itself trigger turns. The controller must still
  interpret validated barcode events only in an approved mission state.
- Backup credentials and never commit `config/wifi_secrets.h`,
  `config/wifi_credentials.h`, or `.lan-secrets.json`.

Run the dashboard with `bash run_live_dashboard.sh` from the project root.
Rebuild/flash the new firmware to enable barcode source changes.
