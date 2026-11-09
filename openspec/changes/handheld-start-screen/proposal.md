# Proposal

## Why

Every screen of the pak must follow the Blackfyre design system. The drive screen, its driving view and the gesture overlay do: they draw with the system's dark app palette, its two typefaces and its keycaps. The start-up message does not: the join message (and any other start-up error, with retry and quit) is gabagool's `ConfirmationMessage`, drawn in NextUI's own look, and while a start check runs the screen shows nothing of the pak's.

## What Changes

- **A start screen in the Blackfyre look.** The pak draws its own start screen with the drive screen's drawing code: the `Spooder.` title, one panel with the message set in Archivo and wrapped to the panel, and the keycaps `A` RETRY and `B` QUIT along the bottom, pressing down while held. Its panel's kicker reads `JOIN` for the join message and `START` for any other start-up error. While a start check runs, the panel says so and no keys show. `Menu` (and on desktop Linux closing the window or Escape) quits, as `B` does; `Enter` on a desktop keyboard retries.
- **A requirement that every screen of the pak follows the Blackfyre design system**, naming the tokens, so no screen comes from gabagool's or NextUI's own components.
- The power button keeps acting at once before driving starts.
- No protocol, firmware or bridge change.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `handheld-controller`: a new requirement, the Blackfyre look for every screen of the pak, the start screen included. The capability is added by `add-handheld-controller`, merged but not yet archived.

## Impact

`desktop/handheld/cmd/spooder-handheld` (`main.go`: the join loop on the pak's own screen; a new `startscreen.go`; `screen.go`: the title drawn by itself), its tests and screenshots, `docs/handheld.md`, `docs/screenshots.md`, `docs/development.md`, `desktop/CLAUDE.md` and `CHANGELOG.md`.
