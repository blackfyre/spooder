# Proposal

## Why

After `port-movement-to-servo2040`, the robot can only be driven from a USB serial terminal. Bluetooth gamepads need an original ESP32, which supports Bluetooth Classic, running Bluepad32's maintained ESP32 target. The LilyGO TTGO T-Display v1.1 provides that ESP32, and its colour screen can show robot status. With the status screen on the bridge, the robot needs no display of its own.

## What Changes

- Add a second firmware, for the TTGO T-Display v1.1. It runs Bluepad32 and maps a Bluetooth gamepad to movement requests, which it sends to the Servo 2040 over a UART link. It also shows robot status on its screen.
- Add a shared, hardware-free link protocol. Command frames go from the bridge to the robot at 50 Hz and telemetry frames go back at 10 Hz. Frames are versioned and CRC-checked, and both firmwares use the same encoder and decoder, which are host-tested.
- Define our own gamepad mapping:
  - **Stand/Walk:** a toggle.
  - **Sleep:** a toggle.
  - **Calibrate:** entered with a deliberate held combination.
  - **Gaits:** chosen with the shoulder buttons.
  - **Height and speed:** dials on the D-pad.
  - **Faults:** cleared with a two-button combination.
  - **Emergency stop:** a button on the bridge cuts servo output.
  
  None of upstream's controller logic is used.
- On the Servo 2040, receive commands on UART1 (GPIO20/21 via J6). Each command is stamped with its arrival time, so the existing stale-command rule puts the robot to sleep if the bridge goes quiet. The robot sends telemetry back: mode, gait, supply readings and faults.
- The USB serial command hook becomes a build option, off by default.

## Capabilities

### New Capabilities

- `controller-link`: The UART link between the bridge and the robot: framing and integrity, the command and telemetry contents and rates, how the robot treats bad or missing frames, and the emergency stop.
- `gamepad-control`: How gamepad input becomes movement requests: the button and axis mapping, dead zones, what happens on controller connect and disconnect, and fault clearing.
- `status-display`: What the bridge's screen shows and when it updates.

### Modified Capabilities

None. `locomotion`'s stale-command rule already covers a silent link, because the robot stamps each command on arrival.

## Impact

- **Code:**
  - New `firmware/bridge/`: a PlatformIO project on the Bluepad32 ESP-IDF + Arduino template, built with pioarduino and ESP-IDF 5.5.x.
  - New `core/protocol/`, shared and host-tested.
  - On the Servo 2040, a new `link` module and changes to `main`.
- **Toolchain:** the bridge builds through PlatformIO, which also runs the host tests. ESP-IDF is fetched by the platform, and new mise tasks cover building and flashing the bridge.
- **Hardware:**
  - The TTGO GPIO27 TX goes to Servo 2040 GPIO21 RX, and the TTGO GPIO26 RX to Servo 2040 GPIO20 TX, both 3.3 V.
  - Ground connects to the Servo 2040 logic GND on J6 pin 16, not the servo ground.
  - The bridge is powered from its own USB or from a 5 V logic supply.
- **Out of scope:**
  - Sensor fields in telemetry: their slots are reserved here and filled by `add-body-sensing`.
  - WiFi.
  - Controllers other than standard Bluepad32 gamepads.
