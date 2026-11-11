# Single Console UI update

The three previous navigation buttons were aliases for the same monitoring
workspace. They are replaced with one **Console** button beside Overview and
Map & robot.

The barcode pane has one live-state badge and a timestamped list of decoded
letters and rejected attempts. Repeated totals, the historical-result spotlight,
the scanner diagnostics paragraph, the repeated navigation outcome and the
mapping shortcut are removed from this pane. Mapping remains in the main header;
arming state remains visible as Ready, Learning white, Rearming or Needs white.
An old packet changes the live-state badge to Offline while result timestamps
remain available. Simulated readiness is explicitly labelled.

The four feeds use consistent card headers, calmer borders, readable log bodies
and responsive spacing. START/STOP stay visible; additional maintenance actions
are in More controls. Panel actions contain copy, clear, pause and export options.
Command history is collapsed by default. Each log retains independent scroll
following and its Latest button, including across viewport changes.

Validation: 78 dashboard tests pass, including real command/ACK and MQTT tests.
Actual Chrome checks at 1440px and 390px cover single navigation, unique DOM IDs,
removed scanner clutter, rejection reasons, filtering, pause/resume, manual scroll
preservation, Latest, maintenance/action menus, command history, stale data and
view switching. No browser runtime errors or horizontal page overflow were found.
The preview uses synthetic data on a separate loopback broker; no commands were
sent to a physical robot.

Firmware control logic is unchanged by this UI update. The preceding scanner,
descent and obstacle fixes remain in the source; see
[control refinements](CONTROL_REFINEMENTS.md). Fresh host and ARM build evidence
is in [host tests](evidence/host-tests.txt), [target build](evidence/target-build.txt)
and [browser checks](evidence/ui-browser-check.json).
