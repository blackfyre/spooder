# Design

## Context

The pak draws its drive screen itself on gabagool's SDL renderer (`cmd/spooder-handheld/screen.go`), in the Blackfyre dark app palette: the window on `bf-ink`, panels on `bf-ink-raised` with a black outline, wells on `bf-ink-code`, text in `bf-surface`, labels in uppercase tracked JetBrains Mono in `bf-ink-subtle`, Archivo for what is read, one orange. Before it drives, `main.join` shows start-up failures with gabagool's `ConfirmationMessage`, whose look is NextUI's.

## Decisions

**D1. One screen object for the whole run.** `main` makes the screen (fonts, layout, texture cache) once after `gabagool.Init` and passes it to the join loop and then to the drive loop. Without it (the embedded fonts fail to load) the pak prints the error and exits: there is no other screen to fall back to.

**D2. The start screen reuses the drive screen's parts.** The title, the panel with its kicker and the keycaps are the drive screen's own functions; the header's title is split out of `header` so both screens draw it the same. The message wraps at spaces to the panel's width (`wrap`); a word wider than the panel is shortened with an ellipsis, and lines past the panel's foot are not drawn (the join message takes three lines at 1280×720).

**D3. Keys act on press.** A and B act on the button's press event, not on its state, so a button still held from launching the pak in NextUI's menu (A) does nothing until pressed again. The controls follow the pad table's labels (the TrimUI's A is SDL's B). Menu, closing the window and Escape quit; Enter retries, for desktop Linux.

**D4. A check in progress shows.** Before each `app.Start` the screen shows the check under way (`JOIN`, checking the WiFi network on the device; `START`, opening the link on desktop Linux) without keys, since `udhcpc`'s renewal can take seconds.

## Risks

- A start-up error's text comes from Go errors and is not translated; it is set as written, not in upper case.
