# Tasks

## 1. Gait

- [x] 1.1 Superpose the straight and rotation displacements around the stance point in `gait::footTarget`, `liftEnd` and `propelEnd`, with the swing height as the higher of the two (design D1)
- [x] 1.2 Scale the combined stroke to the gait's stride limit, both parts by the same factor (D1)
- [x] 1.3 Host tests: translate-only and rotate-only paths unchanged against the current ones; forward stroke kept when a turn is added; a sweep of translate × rotate × gait × height stays within the stride limit, keeps the ratio and is reachable (D2); verify `mise run test` passes
- [x] 1.4 Regenerate any golden walking data only if it changes for combined input alone; verify single-input golden data is unchanged (no golden walking data exists; the body and gait suites pass unchanged)

## 2. Gamepad

- [x] 2.1 Add the steer and crab stick modes to `core/gamepad_mapping`, with `b` as the toggle, steer as the default, the reverse convention and right stick X added in both modes (D3); host tests for each scenario
- [x] 2.2 Show the stick mode on the status screen within its line length (D4); host test
- [ ] 2.3 Verify `mise run build`, `mise run build --serial` and `mise run build-bridge` pass

## 3. Documentation and hardware

- [x] 3.1 Update the readme's controls table, the `spooder-hardware-check` controls, the `spooder-leg-kinematics` Walk notes and the changelog
- [x] 3.2 On the robot: confirm the turn direction (right stick right turns clockwise, as derived from the strokes); walk forward and steer in both stick modes and in reverse, in Tri and Wave; confirm arcs, stride and that no target is rejected
