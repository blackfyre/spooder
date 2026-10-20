# Proposal

## Why

The legs pane draws one leg from the side and all six from above, at a size that fits the side pane. Judging a stance, a gait or a gesture needs the whole robot in three dimensions, large and beside the editor: whether the feet share a ground plane, how high the body sits, and how the legs move against each other. A second window can stay open on another screen while the main window is used for control and calibration.

## What Changes

- A **four-view window** opened from the calibrator's top bar. It draws the whole robot from the latest pose frame in four views:
  - top (front up);
  - front (seen from ahead);
  - left (seen from the robot's left, front to the right);
  - isometric (from the front left, above).
- All views use the same geometry as the legs pane:
  - the reported commanded angles, the link lengths, and the legs' mounts on the schematic body, drawn as a plate;
  - the ground at the lowest foot;
  - the selected leg highlighted, joints near or at their servo limits ringed, and the legs numbered.
- The three orthographic views share one scale. The isometric view draws far legs before near ones.
- The window shows why there is no pose, as the legs pane does. It redraws at the pose rate, and closing it leaves the main window running.
- The 3D geometry (body-frame points and the four projections) lives in `legview`, testable without a window.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `desktop-leg-view`: the four-view window.

## Impact

- `desktop/internal/legview`: body-frame 3D points of every leg, the body plate, and the top, front, left and isometric projections, with tests.
- `desktop/internal/ui`: the four-view window (its own Gio window and loop), the top-bar button, and screenshots in both themes.
- No firmware or protocol change.
