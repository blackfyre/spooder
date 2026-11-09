# Design

## Context

Every command frame carries the gait, so the gait the robot walks in is the controller's. The controllers are the bridge's gamepad mapping (`core/gamepad_mapping`, with the setup portal's controller copying its defaults), the handheld's Go port of it (`desktop/handheld/internal/padmap`, kept in step by `test/golden/gamepad_mapping.txt`) and the desktop calibrator's session (`desktop/internal/session`).

## Decisions

**D1. One named start gait.** `gamepad_mapping::START_GAIT` (1, checked against `gait::Gait::Wave` at compile time) sets the mapping's start; the portal, the Go port (`padmap.StartGait`) and the session (`commandGait`) take the same value. The gait order stays Tri first, so the wire values, the gait names and the next/previous wrap do not change; only the start moves.

**D2. The robot's own initial gait stays Tri.** `body`'s `currentGait` before the first command, and `MovementCommand`'s default, are overwritten by the first frame. Changing them would only move host test baselines.

**D3. The golden file moves by one step.** `gamepad_mapping.txt` is hand-written. Every expected gait moves one step on (mod 6), since each sequence starts from a fresh mapper. The dials sequence gains one `L1` press so it still wraps backwards (Wave, Tri, Hop) and forwards (Hop, Tri).

## Risks

- Wave is the slowest gait (speed multiplier 0.4), so walking is slower until another gait is chosen.
