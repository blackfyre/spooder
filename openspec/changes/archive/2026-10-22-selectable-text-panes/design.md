# Design

## Context

Both panes are a `material.List` of labels with `ScrollToEnd`, over the session's capped slices (2000 log lines, 200 frames). Gio's `ScrollToEnd` already follows only while the list is at its end. Labels cannot be selected, and Gio's `Selectable` selects within one label only, so a selection across lines needs the list's own handling. C with any modifier is the "height down" walking key.

## Decisions

### D1. One text pane widget, selection over the list

A `textPane` wraps the `widget.List`. One pointer handler covers the list (an ancestor of its scroll area, so the wheel and the scrollbar keep working; presses on the scrollbar are ignored). A position is (line sequence number, rune offset). Hit testing uses the rows laid out in the last frame and shapes only the line under the pointer; painting the selection shapes only the visible lines inside it. Nothing lays out the whole history: the per-frame cost stays that of the visible lines. The copied text is assembled only on copy.

Alternative: a read-only `widget.Editor` over the whole log. Rejected: it reshapes the full 2000-line text on every new line, loses the per-line colour of stray-byte lines, and its offsets shift when old lines are dropped.

### D2. Lines are numbered in the session

`LogLine` and `FrameLine` carry a sequence number. The pane derives how many lines were dropped since the last frame from the first line's number, moves its scroll position back by as many, and clips the selection to the lines still present. Without it, a full pane slides by one line per new line.

### D3. Hold and follow

The pane follows (Gio's `ScrollToEnd`) unless it is held. A press in the text holds it. While held, the list keeps its position. The hold ends when the view reaches the bottom from elsewhere (a scroll, the scrollbar or a resize; not new lines, which only move the end away), or when a click leaves no selection. So a selection made at the bottom stays put while lines arrive, and scrolling back down resumes following.

### D4. The copy shortcut

Ctrl+C, or Cmd+C on macOS (Gio's `key.ModShortcut`), copies the selection of the pane shown, and never acts as the height key, with or without a selection: an unexpected height change on a copy attempt is worse than a missing shortcut. Plain C and C with Shift still lower the height. The release still reaches C's key latch.

## Risks

- Hit testing re-creates the label's shaping parameters (font, size, width). If the label's parameters change, the two must change together; both are in the same file.
- Auto-scrolling while dragging past the pane's edge is not provided; Shift+click or the wheel during the drag extends a selection beyond the view.
- While the history is shorter than the pane, Gio's list stays bottom-aligned, so a held selection moves up a row per new line. It stays on the same lines; the case ends once the pane is full (seconds for the frames pane).
- Hit testing assumes left-to-right text: the robot's debug text is printable ASCII, and the frame summaries are ASCII.
