# Proposal

## Why

Standing from Sleep or Initialise moves all 18 MG996R servos at once. On the bench the supply current spiked to 11.8 A, and the overcurrent trip (9 A) latched on that one reading, so the robot could not stand. Each reading averages 8 ADC samples over about 25 µs, once per 20 ms tick, so the trip acts on a snapshot: any short motor surge that happens to coincide with a reading trips it. Raising the trip current would let the robot stand, but would also let a real stall or short draw well above the screw terminals' 10 A rating for as long as it lasts.

## What Changes

- **Persistent trip.** The overcurrent trip at the trip current (9 A) latches only once the current has stayed above it for a configured persistence time (60 ms, about three ticks). A start-up surge shorter than that passes; a stall, which persists, still trips well within a tenth of a second.
- **Instant ceiling.** A new, higher ceiling current (13 A) trips on a single reading above it, within one 20 ms evaluation, as the trip does today. It bounds what the persistence time allows.
- Both latch the same overcurrent fault. Clearing, the restart through Initialise, telemetry, the LEDs, the bridge's rumble and screen, and the link protocol are unchanged. The diagnostics frame keeps reporting the trip current; the ceiling and the persistence time are not sent.
- **New build-time limits** `SPOODER_CEILING_CURRENT_A` and `SPOODER_TRIP_PERSIST_MS` in the power limits file, with compile-time checks: sustained current < trip current < ceiling current ≤ 15 A, and a persistence time of at most 200 ms. A limits file without them does not build.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `power-protection`: the overcurrent trip gains a persistence time and an instant ceiling; supply monitoring names the two new limits.

## Impact

- `core/protection/protection`: `Limits` gains `tripPersistMs` and `ceilingCurrentA`; `State` tracks the run of readings above the trip current.
- `firmware/servo2040/src/config.h`, `power_limits.example.h` and `power_limits.h`: the new constants and checks. The virtual robot builds its limits from the same `config.h`.
- Host tests (`test_protection`, `test_robot_tick`) build `Limits` with the new fields.
- Behaviour on the robot: a stall above the trip current now takes up to about 80 ms to cut output instead of 20 ms; anything above the ceiling still cuts within 20 ms.
- Documentation: the `spooder-hardware-check` overcurrent step.
