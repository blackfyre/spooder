# Design

## Context

The bridge's `loop()` holds the controller pointer, the mapper's desired state (including the gesture request counter) and `robot_link::status()` (telemetry freshness, faults, the gesture playing, the bridge's e-stop toggle). Bluepad32's `Controller::playDualRumble(delayedStartMs, durationMs, weakMagnitude, strongMagnitude)` plays one rumble; a new call replaces the one in progress.

## Decisions

### D1. A hardware-free event detector

`core/gamepad_mapping/rumble`: `update(State&, const Inputs&) → Request` (play or not; duration, weak and strong magnitude), stepped every loop with the time, whether a gamepad is connected, telemetry freshness, the latched faults, the bridge's e-stop, the latest gesture request (id and counter) and the gesture telemetry reports. Host-tested; the bridge only plays what it returns.

### D2. Events and patterns

| Event | Duration | Weak | Strong | Priority |
|---|---|---|---|---|
| Fault newly latched (any fault bit except the e-stop's) | 600 ms | 128 | 255 | 4 |
| Bridge e-stop on | 250 ms | 0 | 255 | 3 |
| Link lost (telemetry stops while a gamepad is connected) | 400 ms | 200 | 120 | 2 |
| Gesture refused (not playing 600 ms after the request) | 160 ms | 160 | 0 | 1 |
| Bridge e-stop off, link regained | 90 ms | 120 | 0 | 1 |

- Detected events wait in a set, each with when it happened. Each step the most important waiting event plays, unless a more important rumble is still playing; events wait at most 1 s, and an event happening again within 1 s of last playing is dropped. So simultaneous events play strongest first, and a lighter one plays after a stronger one.
- The rumble playing is tracked by its start and duration, so a long-finished one never looks active again after the millisecond counter wraps.
- Baselines: on a gamepad connecting, the current faults and e-stop; on telemetry becoming fresh again, the faults. A disconnect forgets everything, waiting events included.
- Gesture refusal: a request is acted on when telemetry shows that gesture start playing; if the same gesture was already playing when it was requested, it must first be seen not playing. A request made, or still pending, while the link is down is forgotten: the link loss has its own rumble.
- Nothing plays without a connected gamepad.

## Risks / Trade-offs

- Rumble support varies by pad and mode (the SF30 Pro rumbles in its Switch and Windows modes); without it the requests are no-ops, so behaviour does not depend on it.
