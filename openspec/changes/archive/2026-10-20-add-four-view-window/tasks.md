# Tasks

## 1. Geometry

- [x] 1.1 Add `legview.Body3` (every leg's points in the body frame), the body plate, the ground (the lowest set foot), and the top, front, left and isometric projections with depth. Verify with legview tests: the calibration pose in each view, a standing pose's feet on the ground, the isometric axes orthonormal, and depth order front-left to rear-right.

## 2. Window

- [x] 2.1 Add `viewsWindow` (state and `frame`), drawing the four views in a 2×2 grid with a shared orthographic scale, the selection, limit rings, leg numbers and the no-pose message. Verify with window tests (no pose, a pose drawn, a shared scale across sizes) and screenshots in both themes.
- [x] 2.2 Add the top-bar button and `Env.OpenViews`. `Run` starts the second `app.Window` once, and the selection is shared with the main window. Verify with a window test (the button calls `OpenViews`) and `mise run desktop-build`.

## 3. Integration and documentation

- [x] 3.1 Update `CHANGELOG.md`, `CLAUDE.md` (desktop layout) and the `spooder-hardware-check` skill. Run `mise run desktop-test`, `mise run desktop-build` and `mise run e2e`, and verify that all pass.
