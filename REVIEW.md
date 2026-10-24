# Reviewing Spooder changes

How a reviewer (Codex, a review agent or a person) reviews a change here. The `deliver-changes` workflow hands this file to every review; The `CLAUDE.md` files and the OpenSpec change say what the change should do.

## What failure looks like

| Area | It fails when |
|---|---|
| `core/movement`, `core/robot` | a joint moves faster than the speed budget, a foot jumps, a target leaves the servo's travel, or the 5 ms grid or the 20 ms tick drifts with load |
| `core/protection`, `servo_output`, `power_monitor`, power limits | output stays on when it must be off: protection fails open, a fault clears motion on its own, or a limit is read wrong |
| `core/protocol`, the links | the two boards or the desktop disagree on a byte, a lost or repeated frame changes behaviour, or a gap replays a request |
| Bridge setup mode and portal | WiFi and Bluetooth run together, the portal accepts what the robot would refuse, or input from the phone reaches the robot unchecked |
| Desktop calibrator | it shows a state the robot is not in, sends without control, or writes an export that does not round-trip |
| Tests, golden files | they pass for the wrong reason, or a golden file was regenerated to match a bug |
| Docs, specs, skills | they lie: a path, symbol, value or behaviour that the code does not have |
| Printed parts (`hardware/spooder-electronics-mount/`) | a part needs support or does not fit the bed, a check passes geometry it should fail (it samples past the bad case), a size the layout uses is assumed without being listed to measure, or an exported file differs from the checked part |
| Tooling, mise tasks, the workflow | they change the wrong tree, lose uncommitted work, or resolve an unpinned dependency |

## Severity

Tag every finding with one:
- **Important**: a bug, a spec departure, a missing test for a scenario, a doc that lies, a break of a rule in a `CLAUDE.md` (the root one or the area's). Each must be answered: fixed, or closed with a reason.
- **Nit**: style or naming with no behaviour. At most three per review; never a reason for another round.
- **Pre-existing**: present before the change (check with `git blame` or the base commit). Reported for later, not fixed in the change and never blamed on it.

Drop findings that are only style preferences, diagnostics or clean-ups the change does not need.

## Evidence

- Give a file and line for every finding, and the input or state that goes wrong. No guesses from a name.
- When a bug has a class (every caller of a function, every leg, every frame type), name the class, so the fix covers each instance.
- On a later round, review only the diff since the previous one, and do not repeat a finding already answered.

## Summary

Start with the count: "N findings: I important, n nits, p pre-existing", or "Nothing to fix." when there is none. Then the findings, Important first.

## Paths to skip

- `external/` (pinned SDKs; their pins in `.gitmodules` and the submodule commits are reviewed);
- `hardware/`'s reference folders (`aecert-hexapod-v1/`, `lilygo-t-display/`, `pimoroni-servo2040/`, `mg996r/`: copied unchanged), and the generated `print/`, `step/` and `images/` of `hardware/spooder-electronics-mount/` (its Python is reviewed);
- `.archify/`, `docs/images/`, `desktop/assets/` renders;
- `test/golden/` contents, except to check that a regeneration is deliberate and matches the change;
- `firmware/bridge/main/fonts/` (generated).

## Bug classes seen before

Look for these first; each has happened here:
- a zero-length time step that bypasses a governor or a rate limit;
- a pose fitted into the servo travel that the code then compares with the unfitted target, so a mode never finishes (Sleep never folded);
- NaN from the walking maths on edge inputs (zero stride, zero rotation);
- blocking waits in the control path;
- a leg entering its cycle part-way through and jumping onto the path;
- a servo enabled without a pulse going to mid-travel;
- non-ASCII debug text on the USB port, which carries binary frames too;
- a counter baseline missed after a link gap, replaying a request;
- a spec scenario or task with no measurable acceptance (no limit, conditions or units that can fail), or a requirement no task verifies;
- a printed part's size taken as given (a board's height, a module's thickness) without a source in the repository or a place in "Measure before printing".

## Security

Review separately where input arrives from outside: the link decoders (UART and USB), the setup portal's HTTP and DNS servers and its WiFi access point, and the desktop's serial port and file export. Look for unchecked lengths and indices, unbounded buffers, values passed on without the robot's guards, and paths written outside the checkout.

## Risk tiers

`tools/risk-tier.sh` sets a change's tier from the paths it touches, not from who wrote it:
- **high**: protection, the protocol and links (firmware and desktop), the control tick, servo output, power, the setup portal, dependency pins and lock files, build and delivery tooling. A person approves the exact commit before it merges. The approval carries over a rebase only when no conflict was resolved by hand and the change's patch id is unchanged; otherwise the rebased commit needs approving again.
- **medium**: other code, and the `CLAUDE.md` files and the skills (they steer later agents). It merges on green gates and resolved reviews.
- **low**: docs, specs and reference files only.
