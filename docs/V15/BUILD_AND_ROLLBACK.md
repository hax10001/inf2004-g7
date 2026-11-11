# Build, flash and rollback

## Environment used

Verification ran on macOS arm64, with Clang, Python 3.14, Node, GNU make,
Arm GNU Toolchain 15.3.Rel1, and Pico SDK **2.2.0** including CYW43/lwIP/TinyUSB.
The brief's Windows assumption did not match this machine. Third-party SDK
internals were not edited. Raw build/test intermediates and private configuration
are excluded from the release ZIP.

## Windows

1. Install Git for Windows (Git Bash), Python 3, Node >=18, GNU make and ARM GCC.
   Keep both ARM GCC and make on the Git Bash PATH. PowerShell is sufficient to
   launch the dashboard; the firmware's existing make shell rules require Bash.
2. Install Pico SDK 2.2.0 with recursive submodules. In Git Bash verify
   `arm-none-eabi-gcc --version`, `make --version`, `python3 --version`,
   `node --version` and `bash --version`. On Windows installations where the
   interpreter is named `python`, provide a `python3` command on the Bash PATH
   before running the existing scripts.
3. Extract this release into a new folder. Configure private network values
   locally, never in an archive intended for sharing:

   ```powershell
   cd app_program/dashboard
   npm run setup:wifi
   ```

4. Build in Git Bash, using the actual SDK path:

   ```bash
   G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
     PICO_SDK_PATH=/c/pico/pico-sdk-2.2.0 bash build_uf2.sh
   ```

   The script enables the obstacle source module and verifies the linked MQTT
   owner. Only a successful build produces the current
   `out/g7-improve-movement-mqtt.uf2`. A failed build is not a reason to flash an
   older UF2. Locally configured images may contain private credentials: do not
   redistribute them.
5. Hold BOOTSEL while connecting the Pico W, then copy the newly built UF2 onto
   the RPI-RP2 volume. Start with the wheels supported and a physical cutoff.
   Test actual STOP before any floor/ramp trial.
6. Start the dashboard with `npm start` from `app_program/dashboard`. Restore
   the route mapping through the acknowledged mapping workflow, wait for READY,
   then issue START explicitly.

The portable host suite is `bash tools/run_host_tests.sh` (named `run_v15_tests.sh` at release). It retains the V14
runner's historical `_mac` filenames but selects GNU versus Apple linker flags.
Windows itself was not available for an execution trial; the documented Bash
workflow still needs Windows acceptance. Native PowerShell C compiler support
was not invented for Bash-based firmware make rules.

## macOS / Linux

```bash
bash tools/run_host_tests.sh  # replaces the version-named runner used at the time
G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

`tools/verify_v15.sh` combines these checks and saves evidence. Configure the
local SDK/compiler PATH first. Minimum-bit inventory reproduction is described
in [MINIMUM_BITS.md](MINIMUM_BITS.md).

## Rollback

The original adjacent `inf2004-g7-v14-recovery` directory is unchanged. Stop and
remove power before rollback. Use that source and its private local configuration,
or a separately retained known-good private V14 UF2. Its build needs
`B1_BUDDY5_MODULE=1` in the environment to include the required obstacle sources:

```bash
B1_BUDDY5_MODULE=1 G7_CONSOLE=usb_cdc G7_LIVE_CONTROL=1 \
  PICO_SDK_PATH="$HOME/pico/pico-sdk-2.2.0" bash build_uf2.sh
```

Flash only the freshly verified image, restart the matching dashboard, send the
mapping again, and explicitly START. Never restore a runtime START request.
The older attached checkout also has a separate pre-edit sanitized snapshot in
the parent workspace's `releases/baseline`; it is not the authoritative V14.
