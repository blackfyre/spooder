# The handheld controller

`Spooder.pak` is a NextUI tool pak for the TrimUI Smart Pro (Go, gabagool and SDL2, in `desktop/handheld/`). It drives the robot through the bridge's WiFi drive with the gamepad's mapping, and shows the robot's state and the WiFi link's quality on its 1280×720 screen.

## Install

1. Build the pak with `mise run handheld-package` (arm64, under rootless Podman). It writes `build/handheld/Spooder.pak.zip`.
2. Unzip it into `Tools/tg5040/` on the TrimUI's SD card, replacing any earlier `Spooder.pak`.
3. Put the card back in the TrimUI.

## Join and drive

1. Close the bridge's drive mode switch (GPIO32) and start the bridge in WiFi drive.
2. Join the bridge's `spooder-drive-XXXX` network in NextUI's WiFi settings. The password is on the bridge's link page.
3. Start **Spooder** from Tools.
4. Press `Y` to stand the robot, then `Start` to walk it.

If the TrimUI is on another network, the pak renews its address once and then names the network to join. The controls are in [Controls](controls.md#trimui-handheld).

## The drive screen

The panels page shows the robot (body mode, output, supply voltage and current, gait, gesture, the source in control and the faults by name), what the pak requests, with small keys naming the controls that change each value, and the link's quality: the signal, frames per type and second, invalid datagrams, lost and late frames, and the time since the last frame. The keycaps along the bottom press down while their control is held; the fault clear's fills as its hold runs.

![The handheld's panels page while walking: the robot in Walk with output on, 7.84 V and 3.42 A, the requested mode, gait, stride, height, speed and stick mode with their keys, and the link at 10 frames per second per type](images/handheld-driving.png)

`Select` + `X` switches to the driving view: the robot from a chase camera fixed behind it, looking forward and 45° down in perspective over a ground grid at the lowest foot, drawn from the robot's pose frames. Joints near or at their servo limits are ringed, the orange chevron marks the front, and the body mode, gait, output, supply and faults sit beside it. The camera follows the body, and with no odometry the grid does not move as the robot walks.

![The driving view: the robot mid-stride from behind and above, its legs numbered 1 to 6, two joints ringed at their limits, a ground grid under it, and the body mode, gait, output, supply and an undervoltage fault beside it](images/handheld-driveview.png)

While the robot reports bench simulation, a `SIM` chip leads the header on both pages and the output reads as simulated.

![The panels page in bench simulation: the SIM chip in the header, OUTPUT SIM on the robot panel, the modelled 6.00 V supply](images/handheld-simulation.png)

Faults are named chips, the emergency stop in orange. Holding `Select` + `Y` fills the fault clear's keycap until the clear is sent.

![The panels page with the emergency stop and undervoltage faults, the e-stop chip set in the header and the fault clear's hold part way](images/handheld-fault.png)

## Leaving

Menu, a short Power press (suspend) and a 2 s Power hold (power off) first request Sleep, and act once two newer telemetry frames in a row report Sleep or output off, or after 3 s. If the pak stops sending without that, the bridge's 150 ms lapse makes the robot sleep. WiFi power saving is turned off while the pak runs and restored however it exits.

![The leave banner: Sleep requested and the pak waiting for the robot to confirm it](images/handheld-leaving.png)

## Without the board

`build/virtual-robot/virtual-robot --wifi-bridge` runs a virtual bridge in WiFi drive on 127.0.0.1:52040, and `mise run handheld-build` gives a desktop build (`build/handheld/amd64/spooder-handheld`) that drives it with `--addr 127.0.0.1:52040`. The handheld's end-to-end tests (`mise run e2e`) drive the same pair.

Installing, joining and the on-device checks are in the [hardware check](../.claude/skills/spooder-hardware-check/SKILL.md).
