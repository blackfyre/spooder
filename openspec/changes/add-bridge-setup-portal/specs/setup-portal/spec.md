# Spec Delta

## Purpose

A setup mode on the gamepad bridge in which the operator uses a phone or laptop, through a WiFi captive portal, to calibrate the robot, without Bluetooth and WiFi ever sharing the radio.

## ADDED Requirements

### Requirement: Entering and leaving setup mode
The bridge SHALL enter setup mode when the left button (GPIO0) is held for at least 1 second while the boot logo is shown, by restarting into setup mode. It SHALL leave setup mode only by restarting into normal operation, either from the portal's exit control or from a reset. A normal power-up SHALL always start in normal operation.

#### Scenario: Enter setup mode
- **WHEN** the left button is held for 1 second during the boot logo
- **THEN** the bridge restarts and comes up in setup mode

#### Scenario: Power-up after setup mode
- **WHEN** the bridge is reset or powered up after being in setup mode
- **THEN** it starts in normal operation

### Requirement: One radio at a time
In setup mode the bridge SHALL NOT start Bluetooth. In normal operation it SHALL NOT start WiFi.

#### Scenario: No gamepad in setup mode
- **WHEN** the bridge is in setup mode
- **THEN** no gamepad can connect, and the WiFi access point is up

### Requirement: Access point and captive portal
In setup mode the bridge SHALL run a WPA2 access point. Its network name SHALL identify the bridge, and its password SHALL be generated randomly the first time setup mode is entered, kept on the bridge, and never derivable from anything the bridge broadcasts. It SHALL answer every DNS query with its own address and serve the portal for any HTTP request, so that phones open the portal on joining. It SHALL accept one portal client at a time.

#### Scenario: Phone joins
- **WHEN** a phone joins the access point with the password shown on the screen
- **THEN** the phone's captive-portal check opens the portal page

### Requirement: Portal as the controller
While a portal page is open and has polled the bridge within the last second, the bridge SHALL report a connected controller in its command frames. The portal's Stand, Calibrate and Sleep controls SHALL each count as a mode request. Translate and rotate SHALL always be zero in setup mode. When the page has not polled for more than 1 second, the bridge SHALL report no controller, so the robot sleeps.

#### Scenario: Portal requests Calibrate
- **WHEN** the operator presses Calibrate on the portal
- **THEN** the next command frames request Calibrate, with a new mode-request counter value and zero translate and rotate

#### Scenario: Phone goes away
- **WHEN** the portal page stops polling for more than 1 second
- **THEN** the bridge's command frames report no controller connected

### Requirement: Safety in setup mode
The command link, the emergency-stop button and the robot's protection SHALL work in setup mode exactly as in normal operation. The portal SHALL show the robot's telemetry and an emergency-stop indication. The portal SHALL offer a fault-clear control that acts as the gamepad's clear combination; it SHALL NOT count as a mode request.

#### Scenario: Emergency stop during calibration
- **WHEN** the e-stop button is pressed while the portal is calibrating a joint
- **THEN** servo output switches off as in normal operation, and the portal shows the emergency stop

#### Scenario: Clear after an emergency stop
- **WHEN** the e-stop is released and the operator presses Clear faults on the portal
- **THEN** the next command frames carry a new fault-clear counter value, and the robot stays asleep until the next mode request

### Requirement: Setup screen
In setup mode the bridge screen SHALL show that it is in setup mode, the network name, the password, the portal address, a QR code for joining the network, whether a portal client is connected, and the robot link state.

#### Scenario: Setup screen content
- **WHEN** the bridge enters setup mode
- **THEN** the screen shows the network name, password, address and a joining QR code

### Requirement: Portal calibration editor
The portal SHALL offer, for each of the 18 joints, its offset, its pulses for 0° and 180°, and its direction, with fine and coarse nudge controls, following the joint-calibration rules for confirmation and display. It SHALL offer the export of the reported table as text to copy. The portal page SHALL be a single file with no external resources and no JavaScript framework.

#### Scenario: Nudge a joint
- **WHEN** the operator nudges leg 2's femur offset by +1° in Calibrate
- **THEN** the page shows the new offset once the robot's report confirms it

#### Scenario: Copy the export
- **WHEN** the operator opens Export
- **THEN** the page shows the complete `calibration_defaults.cpp` text with a control to copy it
