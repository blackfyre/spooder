# Design

## Context

Telemetry is 32 bytes; bytes 23–31 are reserved and encoded as zero. `robot_tick::sendTelemetry` writes the link's own decoder counts into `validFrames`/`discardedFrames` for each link. The bridge link's freshness and its controller flag are known each tick from `link_receiver::take` on the bridge receiver.

## Decisions

### D1. Fields, without a version bump

- Flags byte 21: bit 3 `TELEMETRY_BRIDGE_LINK_UP` (a valid bridge command frame within `LINK_GAP_MS`: the bridge sends one every 20 ms in normal and setup mode, so commands are its heartbeat; calibration sets only ever follow a command), bit 4 `TELEMETRY_BRIDGE_GAMEPAD` (the newest bridge frame reports a connected controller, while the link is up).
- Bytes 23–24 `bridgeValidFrames`, 25–26 `bridgeDiscardedFrames` (u16, wrapping): the bridge decoder's counts, the same values the bridge itself receives in `validFrames`.
- Additive in reserved bytes: a version 3 decoder of either age accepts the frame, and an older robot sends zeros (link down, no counts), which is the truthful "unknown" reading. No version bump, so the boards need not be flashed together for this.
- In a `--serial` build there are no links: all zero.

### D2. Front-ends

- Desktop: a "Bridge" cell in the telemetry strip: `down`, `up · no pad` or `up · pad`, with `N ok · M bad` on its own line beneath, in every state.
- Console: telemetry lines end with `bridge down` or `bridge up (gamepad|no gamepad) N ok M bad`.

## Risks / Trade-offs

- A slightly fuller telemetry strip; the cell shares the existing weights.
