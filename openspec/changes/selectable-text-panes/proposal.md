# Proposal

## Why

The calibrator's debug log can only be read: its text cannot be selected or copied, so a robot message has to be retyped to go into a bug report or a search. While the user scrolls up to read, the view also moves: once the log is full, every new line drops the oldest one, and the lines on screen slide up. The frames pane, which fills in seconds at 30 frames a second, does the same all the time.

## What Changes

- **Selection.** The debug log and the frames pane become selectable with the mouse, within a line and across lines: press and drag, Shift+click to extend. The selection is drawn in the Blackfyre accent, faded. The panes stay read-only.
- **Copy.** Ctrl+C (Cmd+C on macOS) copies the selected lines, as shown (timestamp, text, stray-byte marker), joined by newlines. With a modifier held for copying, C no longer lowers the body height; plain C still does.
- **Following.** The panes follow new lines only while their bottom is visible, as today. In addition, a selection holds the view: new lines move neither the view nor the selection, until the user scrolls back to the bottom or clears the selection there. Scrolled up, the view stays on the same lines even when the oldest ones are dropped.
- **Line identity.** The session numbers its log and frame lines, so the view and the selection stay on the same lines when the oldest are dropped.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `desktop-calibrator`: the log and frames panes are selectable and copyable, and hold their view while scrolled up or selecting.

## Impact

- `desktop/internal/ui`: a shared text-pane widget for the debug log and the frames pane (the export diff is a different view and is not changed); the copy shortcut in the window's key handling; the selection colour in the palette.
- `desktop/internal/session`: a sequence number on each log and frame line.
- No protocol, firmware or virtual-robot change.
