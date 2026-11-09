# Design

## Context

Every command frame carries the gait, so the gait the robot walks in is the controller's. The controllers are the bridge's gamepad mapping (`core/gamepad_mapping`, with the setup portal's controller copying its defaults), the handheld's Go port of it (`desktop/handheld/internal/padmap`, kept in step by `test/golden/gamepad_mapping.txt`) and the desktop calibrator's session (`desktop/internal/session`).

## Decisions

**D1. One named start gait.** `gamepad_mapping::START_GAIT` (1, checked against `gait::Gait::Wave` at compile time) sets the mapping's start; the portal, the Go port (`padmap.StartGait`) and the session (`commandGait`) take the same value. The gait order stays Tri first, so the wire values, the gait names and the next/previous wrap do not change; only the start moves.

**D2. The robot starts in Wave too.** The first draft kept `body`'s initial gait and `MovementCommand`'s default at Tri, on the assumption that the first command frame replaces them. It does not always: the body takes the frame's gait only while it runs (`selectGait` in its sub-step), so with no servo supply (Initialise, stopped) it keeps its initial gait, and with no source in control `link_receiver::arbitrate` runs `MovementCommand()`. Telemetry, and so the bridge screen, the pak's robot panel and the calibrator, then showed Tri while every controller showed Wave. One constant, `gait::START_GAIT` (Wave), now sets `body`'s initial gait (as `BodyState`'s default as well as in `body::init`: the robot only runs `init` once the supply is accepted, so without a supply telemetry reports the default-constructed state), `MovementCommand`'s default (so the no-source command and the `--serial` keys) and `gamepad_mapping::START_GAIT`; the WiFi drive's command without a live handheld and the bridge's first frames take it too. The Go copies (`padmap.StartGait`, the session's and the probe's `commandGait`) hold 1, which a `static_assert` in the mapping pins.

**D3. The golden file moves by one step.** `gamepad_mapping.txt` is hand-written. Every expected gait moves one step on (mod 6), since each sequence starts from a fresh mapper. The dials sequence gains one `L1` press so it still wraps backwards (Wave, Tri, Hop) and forwards (Hop, Tri).

## Risks

- Wave is the slowest gait (speed multiplier 0.4), so walking is slower until another gait is chosen.
