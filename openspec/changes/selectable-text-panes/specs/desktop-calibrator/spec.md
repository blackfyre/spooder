## ADDED Requirements

### Requirement: Selectable text panes
The debug log and the frames pane SHALL let the user select their text with the pointer, within a line and across lines, by dragging, and extend a selection with Shift and a click. The selection SHALL be drawn in the accent colour. The panes SHALL stay read-only. Ctrl+C, and Cmd+C on macOS, SHALL copy the selected text of the pane shown to the clipboard: the lines as shown, joined by newlines. That shortcut SHALL NOT change the walking settings. Selection SHALL NOT lay out more than the lines in view.

#### Scenario: Copy across lines
- **WHEN** the user drags from the middle of one log line to the middle of a later one and presses Ctrl+C
- **THEN** the clipboard holds the selected part of the first line, the lines between, and the selected part of the last line, separated by newlines

#### Scenario: Copy shortcut in control
- **WHEN** the app is in control in Stand and the user presses Ctrl+C
- **THEN** the body height is unchanged

### Requirement: Text panes follow the newest line
The debug log and the frames pane SHALL follow new lines only while their bottom is visible. While the user has scrolled up, or holds a selection, new lines SHALL move neither the view nor the selection, also when the oldest lines are dropped. While all the lines fit in the pane, they SHALL stay aligned to its bottom. Scrolling back to the bottom, or clearing the selection there, SHALL resume following.

#### Scenario: Reading back
- **WHEN** the user scrolls up in a full log while the robot keeps printing
- **THEN** the lines in view stay in place until the user scrolls back to the bottom, after which the log follows again

#### Scenario: Selecting at the bottom
- **WHEN** the user selects a line at the bottom of the log while new lines arrive
- **THEN** the view and the selection stay in place until the user scrolls to the bottom again
