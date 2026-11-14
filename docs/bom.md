# Bill of materials

Everything needed to build Spooder: the electronics, the power train, the controllers, and the printed parts with their hardware. Two detailed tables stay with their parts and are summarised here: [Aecert's hardware list](../hardware/aecert-hexapod-v1/BOM.md) (bearings, inserts, screws, derived from the STLs) and [the electronics mount's hardware](../hardware/spooder-electronics-mount/README.md#hardware). Where this page and those tables disagree, those tables win.

**Status:** *fitted* parts are in the current build; *optional* parts are not needed to walk; *planned* parts belong to OpenSpec changes not yet done (`add-body-sensing` is suspended).

## Electronics

| Item | Qty | Notes | Status |
|---|---|---|---|
| Pimoroni Servo 2040 (RP2040) | 1 | the robot board; cut its "Separate USB and Ext. Power" trace (the servo rail is above 5 V) | fitted |
| 2.54 mm pin header for J6 | 1 | J6 has 17 pins and is unpopulated; solder a header | fitted |
| LilyGO TTGO T-Display v1.1 (ESP32) | 1 | the gamepad bridge, on the deck under the cover | fitted |
| MG996R servo, 180° | 18 | 4.8–7.2 V; check each clone's pulse end points before fitting horns | fitted |
| Latching switch, two terminals | 1 | the bridge's drive mode switch (GPIO32 to GND): open is Bluetooth drive, closed is WiFi drive | fitted |
| 3010 fan, 5 V, 2-pin | up to 2 | exhaust (blowing out) and intake (blowing in) positions in the mount | optional |
| LSM6DSO IMU breakout, with I²C pull-ups | 1 | on its pad under the Servo 2040 | planned |
| VL6180X distance sensor breakout, with I²C pull-ups | 1 | under the frame, wired through the deck's pass-through | planned |
| Leg end-stops | 6 | to the sensor header, with one shared ground | planned |

## Power

| Item | Qty | Notes | Status |
|---|---|---|---|
| 2S LiPo, 7.4 V, 5000 mAh, XT60 plug | 1 | Yowoo 5000 mAh 100C graphene, 284 g; fills the cavity under the frame | fitted |
| XT60 socket | 1 | on the robot, in the mount's left wall; the pack carries the plug | fitted |
| Inline ATO fuse holder, with a 15 A ATO fuse | 1 | at the XT60; essential, because a 100C pack delivers hundreds of amps into a short | fitted |
| SZBK07 (SPX-20A) step-down converter | 1 | the servo rail: 6–40 V in, 20 A, set to 6.0 V, current limit at maximum; its heatsinks are live | fitted |
| LM2596 step-down module | 1 | the logic rail: set to 5.0 V, fed from the fused pack, not from the servo rail | fitted |
| 1N5819 Schottky diode, in 6 mm heat-shrink | 1 | inline from the LM2596 to J6 pin 17, so a computer on USB is not back-fed | fitted |
| 2S BMS, 30–40 A | 1 | in the battery bay, a hard cut-off backstop | optional |
| Resistors 100 kΩ and 33 kΩ | 1 each | battery sense divider into a sensor input; the firmware does not read it yet | planned |
| Wire, 14–16 AWG silicone | about 0.5 m red and black | pack to the fuse, the SZBK07 and the Servo 2040's screw terminals | fitted |
| Wire, 22–24 AWG | about 1 m in several colours | the logic rail, the bridge link, the switch, the fans | fitted |
| Cable ties, at most 3 mm wide and 1.8 mm thick | 4 | the diode, the XT60 and the fuse holder (two), through the deck's tie holes | fitted |

For bring-up, a current-limited bench supply replaces the pack (the `spooder-hardware-check` skill).

## Controllers

| Item | Qty | Notes | Status |
|---|---|---|---|
| 8BitDo SF30 Pro, or another Bluepad32 gamepad | 1 | Bluetooth drive, through the bridge | fitted |
| TrimUI Smart Pro running NextUI | 1 | WiFi drive, with `Spooder.pak` ([the handheld controller](handheld.md)) | optional |
| USB-C cable | 1 | the robot's USB port: flashing, the console and the desktop calibrator | fitted |
| USB-C cable | 1 | flashing the bridge | fitted |

## Printed parts

| Set | Parts | Where |
|---|---|---|
| Walking robot (Aecert's Hexapod V1) | frame, six legs (coxa, femur, tibia links) and the sprung feet | [`hardware/aecert-hexapod-v1/`](../hardware/aecert-hexapod-v1/README.md#parts); armour, roll cage, spikes, cup holder and stand are not needed |
| Electronics mount | deck, cover, four bumpers, four spacers and the small holders, all in PETG | [`hardware/spooder-electronics-mount/`](../hardware/spooder-electronics-mount/README.md#parts-and-colours) |

## Mechanical hardware

Totals from the two detailed tables. Each one gives where every item goes and how sure the derivation is.

| Item | Robot | Mount | Total |
|---|---|---|---|
| 608 ball bearing (8 × 22 × 7), ZZ or 2RS | 18 | | 18 |
| M3 heat-set insert, for a Ø4.4 hole | 166, + 8 in Frame Top for the mount | 3 | 177 |
| M3 screws, countersunk or socket head, 6–25 mm | 166 (by length below) | | 166 |
| M3 button head (ISO 7380), 6 mm | | 4 | 4 |
| M3 button head (ISO 7380), 10 mm | | 7 | 7 |
| M2.5 × 6 self-tapping screw for plastics | | 6 | 6 |
| M2 × 5 self-tapping screw for plastics | | 2 (planned IMU) | 2 |
| Servo mounting screws, self-tapping, about ST2.2–2.6 × 8–10 | 72 | | 72 |
| Horn screws | 72 | | 72 |
| Horn centre screw, M3 × 10–12 | 18 | | 18 |
| Self-tapping M2 × 4–6 | 12 | | 12 |

The robot's M3 screws by length: 6 mm × 18, 8 mm × 6, 10 mm × 12, 12 mm × 72, 14 mm × 4, 22 mm × 36, 22–25 mm × 12, 25 mm × 6. The horn screws depend on the horn: M2 self-tapping for the stock plastic horn, M3 × 5–8 for a tapped aluminium 25T disc ([open question 1](../hardware/aecert-hexapod-v1/BOM.md#open-questions)). The servo screws usually come with the MG996R.
