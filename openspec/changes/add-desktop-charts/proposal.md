# Proposal

## Why

Bench work on the new robot is about supply sag, current under load, link health and whether the servos keep up. The desktop calibrator shows only the latest telemetry values, so trends, spikes and their relation to what the robot was doing are invisible. Some of what matters is not on the link at all: how much the joint speed budget slows the motion, the output rate backstop, the control loop's time and the configured power limits.

## What Changes

- **Diagnostics frame (robot → USB only, about 10 Hz).** A new link frame type, like the pose frame, carries the robot's internals over each 100 ms window:
  - the smallest motion share the joint speed budget allowed;
  - the output rate backstop's tick count;
  - the control tick's longest and mean working time;
  - the protection's averaged current;
  - the configured power limits (trip and sustained current, minimum, maximum and brown-out voltage);
  - the servo profile's joint speed budget and rated speed.

  It is an additional frame type of protocol version 3. The bridge never receives it and needs no change, and an older calibrator discards it.
- **Charts tab in the desktop calibrator.** A fifth tab, beside the debug log, frames, export and legs, holds stacked charts on one shared time axis:
  - supply voltage, with the voltage limits;
  - current and averaged current, with the current limits;
  - accepted and rejected frames per second on the robot's link, the bridge's link and the host link;
  - a timeline of the body mode, servo output and faults;
  - the fastest joint speed, per joint kind, with the budget and the rated speed;
  - the selected leg's servo angles, with the 0° and 180° limits;
  - the budget's motion share;
  - the control loop's time, against the 20 ms period;
  - backstop hits per second.

  A window selector chooses the last 30 s, 2 min or 10 min (10 min is kept). Hovering shows the values at that moment on every chart.
- **CSV export.** A button writes the kept history to a CSV file, one row per telemetry sample, and names the file.

## Capabilities

### New Capabilities

- `diagnostics-telemetry`: the robot's diagnostics frame on USB.

### Modified Capabilities

- `desktop-calibrator`: the charts tab and its CSV export.

## Impact

- `core/protocol`: the diagnostics frame (encode and decode), with golden vectors in `test/golden/protocol_v3.txt`.
- `core/robot/robot_tick`: collecting the window's diagnostics and sending the frame. `Inputs` gains the previous tick's working time, which `firmware/servo2040/src/main.cpp` and the virtual robot measure.
- `desktop/internal/protocol`: the frame in Go, tested against the golden file.
- `desktop/internal/session`: the ten-minute history and the CSV writer.
- `desktop/internal/ui`: the charts tab.
- The robot is reflashed. The bridge is not affected.
