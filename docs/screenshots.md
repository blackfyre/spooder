# Screenshots

## The desktop calibrator

All taken against the virtual robot, in the calibrator's dark or light look (following your system's).

<picture>
  <source media="(prefers-color-scheme: light)" srcset="images/calibrator-light.png">
  <img alt="The desktop calibrator in Calibrate: the joint table, with coxa offsets shown from the servo centre, and the legs pane drawing leg 1 and all six legs from above" src="images/calibrator-dark.png">
</picture>

*Calibrate: the joint table, with coxa offsets shown from the servo centre, and the legs pane.*

<picture>
  <source media="(prefers-color-scheme: light)" srcset="images/walking-legs-light.png">
  <img alt="The calibrator in control, walking forward in the tri gait from the keyboard: the status strip, the key hints and the legs pane drawing leg 1 mid-stride" src="images/walking-legs-dark.png">
</picture>

*Driving from the keyboard: walking forward in the tri gait, the legs pane following the reported pose.*

<picture>
  <source media="(prefers-color-scheme: light)" srcset="images/four-views-light.png">
  <img alt="The four-view window: the robot mid-stride from the top, isometrically from the front left, from the front and from the left, with the selected leg 1 highlighted and the ground line at the feet" src="images/four-views-dark.png">
</picture>

*The four-view window (**4 views** in the top bar): top, isometric, front and left, from the same pose.*

<picture>
  <source media="(prefers-color-scheme: light)" srcset="images/charts-light.png">
  <img alt="The charts tab: supply voltage with its limits and held low, servo current with the trip and sustained limits and held peak, and the frames accepted and rejected on each link" src="images/charts-dark.png">
</picture>

*The charts tab: ten minutes of supply, current, link and robot-internal history, with the recent peak current and low voltage held for 3 s, and CSV export.*

<picture>
  <source media="(prefers-color-scheme: light)" srcset="images/estop-light.png">
  <img alt="The calibrator after an emergency stop: the red banner, the fault in the status strip, output off, and the debug log showing the fault latching and movement stopping" src="images/estop-dark.png">
</picture>

*An emergency stop: the fault latches, output turns off, and the debug log records why.*

## The bridge screen

The gamepad bridge's 240×135 screen, rendered on the host by its own drawing code (`mise run bridge-screenshots`). A short press of the left button cycles the pages; the status strip at the bottom shows the e-stop, faults and link state on every page. The voltage and current read green, amber or orange by band against the protection limits the robot reports: amber in the warning band (within the brown-out-to-undervoltage gap of the voltage limits, or from the sustained current), orange at or past a fault level (undervoltage, overvoltage, the trip current). White ticks hold the recent peak current and lowest voltage for 3 s; a value in the green band turns amber when it sets a new high or a deeper sag.

| Drive | Stand surge | Power | Link |
| --- | --- | --- | --- |
| ![The drive page: walking in the tri gait, 6.08 V and 1.84 A in green with bars, ticks holding the recent 4.3 A peak and 5.54 V low, height 60 and speed 44, dynamic stride and steer, output on, no faults](images/bridge-drive.png) | ![The drive page at a stand surge: 5.54 V and 4.30 A in amber, each a new extreme beyond its held tick](images/bridge-surge.png) | ![The power page: the last minute of supply voltage and current as traces, with the sag and surge of a stand, the lowest voltage and the highest current](images/bridge-power.png) | ![The link page: 50 frames per second accepted by the robot and 10 by the bridge, none rejected, telemetry 40 ms old, the bridge as the source, the pad at 80 %](images/bridge-link.png) |

| Fault | Emergency stop | Link lost |
| --- | --- | --- |
| ![The drive page after a brown-out: low V and brown-out in the orange status strip, output off](images/bridge-fault.png) | ![The power page with the emergency stop in the orange status strip](images/bridge-estop.png) | ![The drive page with the link lost: no robot values, LINK LOST in the strip](images/bridge-lost.png) |

| WiFi drive | WiFi drive link |
| --- | --- |
| ![The drive page in WiFi drive: WF CTRL, the handheld in control, in the header, and the stride mode without a stick mode](images/bridge-wifi-drive.png) | ![The link page in WiFi drive before a handheld joins: the rates, the network name and the password, and HANDHELD NONE in place of the pad](images/bridge-wifi-link.png) |

## The handheld

The TrimUI's 1280×720 start and drive screens, rendered on the host by the pak's own drawing code from sample states (see [Development](development.md#screenshots)). [The handheld controller](handheld.md) describes each.

| Start screen |
| --- |
| ![The start screen with the join message and the A RETRY and B QUIT keys](images/handheld-join.png) |

| Panels | Driving view |
| --- | --- |
| ![The panels page while walking](images/handheld-driving.png) | ![The driving view: the robot from a chase camera behind it, 45° down](images/handheld-driveview.png) |

| Bench simulation | Faults and the clear hold | Leaving |
| --- | --- | --- |
| ![The panels page in bench simulation, with the SIM chip](images/handheld-simulation.png) | ![The panels page with the emergency stop and undervoltage faults](images/handheld-fault.png) | ![The leave banner, waiting for Sleep](images/handheld-leaving.png) |

| Off the network |
| --- |
| ![The panels page with the OFF NETWORK chip and the notice to rejoin spooder-drive-1a2b, nothing sent](images/handheld-offnetwork.png) |

| Gesture overlay |
| --- |
| ![The gesture overlay while L2 is held: five tiles as the TrimUI's buttons, wiggle playing, dance, attack and ball not scripted yet](images/handheld-gestures.png) |
