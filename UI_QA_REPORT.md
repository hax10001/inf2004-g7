# G7 Buddy 5 dashboard polish / UI QA

This change is limited to the **experimental Buddy 5 MQTT dashboard** and its simulator/docs. The verified `improve-movement` fallback is unchanged. It does not merge or flash untested obstacle avoidance firmware.

## Visual design / consistency

- Palette: navy `#182B42`, off-white `#FFFEFB`, mustard `#C49B3B`, light beige `#F0EADF`. Green/red are reserved for Start/Stop and meaningful state.
- Consistent panel inset (`--panel-pad`), button minimum height (`--control-h` = 42px), card radii/borders, and selector/input control heights.
- Start, Stop, Recalibrate IMU and Snapshot now align on 42px height with equal container clearance; mobile uses a tidy two-column action layout.
- Monitor export tools wrap cleanly, labels are aligned, auto-scroll checkbox is correctly sized, dialogs are themed, and the branded G7 logo is on one line.
- Removed a large blank gap below the overview's first card row (previous inherited grid-row sizing).
- Route/3D backgrounds and chassis model use the palette; semantic event colors remain legible.

## Browser layout checks

Static DOM + CSS rendered using headless Chromium at **1440×900, 1080×800, 768×900, 390×844 and 320×740**. Checked Overview, Map & robot and Monitor in each. No page-level horizontal overflow. All four Monitor action buttons measure exactly **42px** high at each viewport. First button left inset is at least **16px** from its panel border.

## Runtime / simulator verification

- Dashboard test suite: **14 passed** including binary protocol, command ACK/session, MQTT integration, CSV and UI contract.
- `./demo.sh`: verified it launched both localhost services automatically, generated **17 frames** and **198mm** of simulated movement by smoke-test observation, with two wheel speed readings of **120mm/s**. Values are test outputs, not calibrated real measurements.
- `STOP` returned target `sim`, received matching accepted ACK, and reduced simulated wheel speeds to zero.
- Tests are host-side only. A simulated STOP does **not** verify a physical emergency-stop mechanism.

## Known limitations

- Real Pico W TCP/MQTT integration and motor-safety performance require physical verification.
- Buddy 5 obstacle avoidance is still staged/experimental; this UI change does not claim bypass hardware works.
- Map points represent estimated travel and *detection positions*, not a surveyed SLAM map. The illustrative 3D chassis shows motion from transmitted wheel speeds, but does not invent IMU roll or servo angle not carried in protocol v1.
- The one-command script binds to `127.0.0.1` and exits if ports 3000 or 1883 are already in use. Press Ctrl+C to stop both processes.
