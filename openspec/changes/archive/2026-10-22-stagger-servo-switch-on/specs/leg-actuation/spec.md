# Spec Delta

## MODIFIED Requirements

### Requirement: Servo output enable and disable
Servo output SHALL be switchable off for all eighteen outputs together, leaving the joints unpowered. While output is off, a new accepted foot target SHALL switch output back on before its pulses are produced, unless power protection is inhibiting output. While output is inhibited, output SHALL stay off whatever targets are accepted. When output switches on, the legs SHALL receive their first pulses in groups of two, one group per control tick, in leg order (legs 1–2, then 3–4, then 5–6); a leg not yet reached SHALL produce no pulse. Once a leg has been switched on, its pulses SHALL follow the body's targets until output switches off.

#### Scenario: Disable outputs
- **WHEN** servo output is switched off
- **THEN** no servo output produces a pulse

#### Scenario: Implicit wake
- **WHEN** servo output is off, power protection is not inhibiting output, and a leg accepts a new foot target
- **THEN** output is switched on and the leg's pulses are produced, in its group's tick

#### Scenario: Inhibited output stays off
- **WHEN** power protection is inhibiting output and a leg accepts a new foot target
- **THEN** no servo output produces a pulse

#### Scenario: Staggered switch-on
- **WHEN** servo output switches on with every leg's angles set
- **THEN** legs 1 and 2 produce pulses from that tick, legs 3 and 4 from the next tick and legs 5 and 6 from the tick after, and none before its tick
