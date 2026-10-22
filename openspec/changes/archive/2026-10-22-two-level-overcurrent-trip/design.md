# Design

## Context

`protection::step` evaluates one reading per 20 ms tick. A reading (`power_monitor`) averages 8 ADC samples, about 25 µs, so it is a snapshot of the current, not an average over the tick. Today `presentConditions` sets `OVERCURRENT` whenever `currentA > tripCurrentA`, and that latches on the same step. The undervoltage fault already has a time-based debounce (`belowMin`, `belowMinSinceMs`, `undervoltDebounceMs`), and the sustained-current fault a time-weighted window. On the bench, standing all 18 MG996Rs at once read 11.8 A once and tripped the 9 A limit.

## Goals / Non-Goals

**Goals:**
- A start-up surge above the trip current, shorter than a few ticks, does not trip.
- A stall or short above the trip current still cuts output within about 80 ms.
- Anything well beyond the board's rating cuts output within one evaluation, as today.

**Non-Goals:**
- Changing the link protocol or the diagnostics frame. `tripCurrentMa` keeps reporting the trip current; the LED load bar keeps its trip-current scale.
- A new fault type. Both levels latch `OVERCURRENT`, so telemetry, the LEDs, the rumble, the bridge screen and the desktop calibrator need no change.
- Faster sampling or averaging over the tick in `power_monitor`. That would also smooth the readings the ceiling acts on.
- Staggering the servos' start in Initialise or Stand.

## Decisions

### The persistence time is counted from the first reading of an unbroken run

`State` gains `aboveTrip` and `aboveTripSinceMs`, mirroring the undervoltage debounce. A reading above the trip current starts the run (if none is running) at its own time; a reading at or below the trip current ends it. The overcurrent fault latches on a reading above the trip current when `nowMs - aboveTripSinceMs >= tripPersistMs`.

At the board's 20 ms ticks and a 60 ms persistence, readings at 0, 20, 40 and 60 ms after the run starts: the fourth consecutive reading above the trip current latches, three evaluations after the first. The current may have risen up to one tick before the first reading that saw it, so the worst case from the rise to output off is the persistence time plus one evaluation period (80 ms). A persistence time of 0 latches on the first reading, which is today's behaviour.

Time, not a reading count, keeps the rule independent of the tick length, like the undervoltage debounce and the sustained window. The virtual robot's stepped mode and the host tests use other tick lengths.

The run is tracked whether or not the supply gate is open, as the trip itself is today: overcurrent applies before the supply is first accepted.

*Alternative:* count consecutive readings (three). Rejected: the protection elsewhere works in time, and a count would change meaning with the tick length. *Alternative:* trip on a short average (for example over 60 ms). Rejected: one huge reading followed by low ones could average above the trip current, and a sustained 9.5 A after a dip could average below it; the ceiling already handles the huge reading, and "stayed above" is the behaviour asked for.

### The ceiling is a plain instant trip

A reading above `ceilingCurrentA` latches `OVERCURRENT` on the same step, before the persistence check, regardless of any run. The ceiling is 13 A: above the measured 11.8 A stand surge with some margin, below the current sense's range (about 15.9 A). It is a short-term bound; the 10 A terminal rating is a continuous one, which the persistence time and the sustained current protect.

### Clearing uses the trip current, without persistence

`presentConditions` still reports the overcurrent condition as present while the current is above the trip current. A clear request therefore fails while the current is above 9 A, even for one reading, so a clear cannot be accepted mid-stall. Latching is computed separately from the condition used for clearing.

### Limits and their checks

`protection::Limits` gains `tripPersistMs` and `ceilingCurrentA` directly after `tripCurrentA`. Inserting them there, rather than appending, makes every positional initialiser written for the old layout fail to compile (a float narrowed into `tripPersistMs`), instead of silently leaving a zero ceiling that would trip on every reading.

`config.h` requires `SPOODER_CEILING_CURRENT_A` and `SPOODER_TRIP_PERSIST_MS` (the build fails without them, with the existing message) and checks:
- `SPOODER_SUSTAINED_CURRENT_A < SPOODER_TRIP_CURRENT_A < SPOODER_CEILING_CURRENT_A <= 15 A`;
- `SPOODER_TRIP_PERSIST_MS <= 200`: longer would let a stall above the terminals' rating run for a fifth of a second unchecked.

The trip current's own `<= 15 A` check is implied by the ordering and replaced by it.

## Risks / Trade-offs

- A real stall between 9 A and 13 A now runs up to about 80 ms instead of 20 ms → still far below the time constants of the servos' and terminals' heating; the ceiling covers anything larger.
- A stand surge longer than 60 ms above 9 A would still trip → the persistence time is a build-time limit, tunable up to 200 ms from bench measurements; staggering the start is a separate, possible later change.
- A bench limits file with a trip current at or above 13 A no longer builds → intended: the ceiling must sit above the trip current.
