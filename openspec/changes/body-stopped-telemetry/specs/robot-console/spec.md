## ADDED Requirements

### Requirement: Stopped body in the frame view
With `--frames`, while a telemetry frame reports the body stopped, the monitor's telemetry line SHALL show `Stopped` in place of the body mode and gait. A telemetry frame whose body-stopped flag differs from the last one printed SHALL always be printed, as a change of mode is under `--telemetry-every`.

#### Scenario: Stopped telemetry line
- **WHEN** `--frames` is given and a telemetry frame with body mode Walk reports the body stopped
- **THEN** its line shows `Stopped` and not `Walk`

#### Scenario: Stop under throttling
- **WHEN** `--frames --telemetry-every 10` is given and a telemetry frame reporting the body stopped in Walk follows frames reporting it running in Walk
- **THEN** that frame's line is printed

#### Scenario: Restart under throttling
- **WHEN** `--frames --telemetry-every 10` is given and a telemetry frame reporting the body running in Walk follows frames reporting it stopped in Walk
- **THEN** that frame's line is printed
