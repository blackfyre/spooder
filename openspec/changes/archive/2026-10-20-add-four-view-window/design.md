# Design

## Context

`legview` has the leg's forward kinematics in its own vertical plane (`SideView`), the map into the leg frame (`Plane`), the legs' mounts on the body (`Mount.ToBody`, body frame X right and Y forward, from above) and the schematic hexagonal outline. The legs pane draws one leg from the side and all six from above. The main window is a single Gio `app.Window` run from `ui.Run`. The session's change hook invalidates it through `Env.Redraw`.

## Goals / Non-Goals

**Goals:**
- One geometry for the legs pane and the four views.
- Views large enough to judge a stance, on any screen.

**Non-Goals:**
- Measured body geometry. The plate stays schematic until the frame is measured: the hexagon through the coxa pivots, 40 mm thick.
- Interaction: rotation, zoom, choosing a joint by clicking.
- Animation between pose frames. The views show each 10 Hz pose as it arrives.

## Decisions

### 3D in legview, projections as pure functions

`legview.Body3(pose)` gives every leg's four points in the body frame (X right, Y forward, Z up, coxa pivots at Z = 0): `Mount.ToBody` of the leg-frame point, with its z. Four projections map a body point to screen coordinates (x right, y up) and a depth:
- **top:** (X, Y);
- **front, seen from ahead:** (−X, Z);
- **left, seen from the robot's left:** (Y, Z);
- **isometric:** from the direction d = (−1, 1, 1)/√3 (front left, above). The screen axes are r = (1, 1, 0)/√2 and u = (1, −1, 2)/√6, and the depth is p·d.

The ground is the lowest set foot's Z. Everything is tested without a window, against the calibration pose and standing poses.

### One scale for the orthographic views

The scale is fitted once from fixed body extents: ±(pivot + full leg reach) horizontally, and from the lowest reachable foot to the highest reachable knee vertically. The smallest of the three views' fits is used for all three, so sizes compare across views. The isometric view uses its own fit.

### A second Gio window with its own loop

`viewsWindow` holds the window's state and a `frame(gtx)` method, like the main window, so tests and screenshots drive it without a display. The main window's top-bar button calls `Env.OpenViews`, which `Run` implements: it starts a goroutine with a new `app.Window` and its event loop, unless one is already running (guarded by an atomic flag), and it ends with that window. The views window reads `Session.Snapshot` each frame and asks for a frame every 100 ms (`op.InvalidateCmd`), the pose rate, so it needs no hook from the session. Closing the main window ends the process, and with it the views window. The selection comes from the main window through a shared atomic (the selected joint).

### Drawing

Each view is a canvas on the well, as in the legs pane:
- the body plate: in the top view the hexagon; in the others the projected prism's outline, with the top face filled;
- the ground as a dashed line, or in the isometric view a faint grid under the body;
- legs as in `topLeg`: coxa thick, femur and tibia thinner, joints as discs, the foot larger;
- the selected leg in the accent, limit rings from `limitRing`, and the leg number near each coxa pivot;
- a caption in each view's corner.

In the isometric view, legs are drawn far to near, by the depth of their coxa pivot.

## Risks / Trade-offs

- [A schematic body can mislead about clearances] → Its captions and the legs pane say "schematic". The mounts are already the leg pane's.
- [Two windows draw the same pose at different times] → Both read the session's latest pose, which is at most 100 ms apart.
