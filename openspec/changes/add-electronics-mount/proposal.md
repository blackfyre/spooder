# Proposal

## Why

Spooder's electronics have no home. The boards, the converters, the fuse and the battery connector need a printed mount on the frame top that:
- holds them;
- protects them from knocks, debris and an occasional rollover;
- keeps them cool;
- stays out of the legs' sweep;
- comes open easily, because calibrating and flashing are done with the cover off.

The sensors planned in `add-body-sensing` and the leg end-stops need room reserved now, or fitting them later means a redesign.

## What Changes

- **A generated, checked mount:** a printable deck screwed to the frame top's inserts, a cover over it, corner bumpers and small holders. A build script makes them from named measurements, checks them, and only then writes the print files.
- **What it holds:**
  - the Servo 2040 and the T-Display, both fed 5 V on their pins by one LM2596 module;
  - the SZBK07 servo converter, the XT60 socket, the inline 15 A fuse and the Schottky diode (inline on the LM2596's output lead, tied down).

  Every wire has a planned route and tie points.
- **Sensor provisions:**
  - a pad for the LSM6DSO IMU near the centre of gravity, clear of the SZBK07;
  - a pass-through for the VL6180X's wires to the underside;
  - entry for the six leg end-stop wires and their shared ground to the Servo 2040's sensor header.
- **Service:** the cover fastens with M3 screws into heat-set inserts and lifts off with nothing to unplug. Running with the cover off is a normal mode.
- **Cooling:** passive, ready for fans. Air comes in low on one side and leaves high on the other, past the SZBK07. There is a 30 mm exhaust fan position (blowing out), an optional 30 mm intake position, both in holders on the deck, and no roof vents.
- **Printing and look:**
  - everything in PETG on a Bambu P1P with an AMS, without supports;
  - a faceted, industrial body;
  - yellow PETG leaf-spring bumpers that are replaceable;
  - a black cover with flush yellow inlays on the lid, exported as a multi-colour 3MF.
- **Documentation:** the hardware-check skill's wiring names the LM2596 as the logic buck that feeds both boards on their pins, its dropout near an empty pack, and the fan supply.

**Non-goals:**
- sealing against water;
- an intrusion switch (unplug the XT60 before opening the cover instead);
- the VL6180X's bracket under the frame (with `add-body-sensing`);
- TPU parts (the bumper rails accept a TPU bumper later without other changes).

## Capabilities

### New Capabilities

- `electronics-mount`: The printed mount for the robot's electronics on the frame top:
  - its fit to the frame and the legs' sweep;
  - what it holds, and the sensor provisions;
  - the cable routes;
  - access and service;
  - cooling;
  - protection;
  - printability and colours;
  - the checks that the generated parts must pass.

### Modified Capabilities

None. The firmware's behaviour does not change; the power wiring changes are documentation in the hardware-check skill.

## Impact

- **New:** `hardware/spooder-electronics-mount/` (it does not exist yet; `CLAUDE.md` and the printed-parts skill already name it): the build script, the preview renderer, the README (printing, hardware, assembly, cable routes, measurements to take), and the generated STL, STEP and 3MF files.
- **Changed:**
  - `mise.toml`: the `cad` and `cad-preview` tasks;
  - `.claude/skills/spooder-hardware-check/SKILL.md`: power wiring;
  - `CLAUDE.md`: the hardware list;
  - `CHANGELOG.md`.
- **Dependencies:** build123d, numpy, trimesh and matplotlib, pinned in the scripts' inline metadata and run with `uv`. The repository's reference files: the frame top STL, the T-Display STEP and the Servo 2040 drawing.
- **Hardware to buy or measure:**
  - an LM2596 module;
  - optional 3010 5 V fans;
  - M3 heat-set inserts and screws;
  - the SZBK07, the LM2596, the Schottky diode and the LSM6DSO breakout, measured before printing.
- **Risk:** low. Nothing runs on the robot, and a misfit costs a reprint. The mount's checks guard against the misfits that can be computed.
