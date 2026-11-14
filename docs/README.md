# Spooder documentation

- [Getting started](getting-started.md): the toolchains, trying it without hardware, flashing the boards, bench simulation and wiring the link.
- [Bill of materials](bom.md): the electronics, the power train, the controllers, and the printed parts with their hardware.
- [Wiring](wiring.md): the power and signal diagrams, the bridge link, the J6 header and the servo outputs.
- [Controls](controls.md): the gamepad through the bridge, and the TrimUI handheld.
- [The handheld controller](handheld.md): installing `Spooder.pak` on the TrimUI Smart Pro, joining the bridge, its two pages and leaving.
- [Screenshots](screenshots.md): the desktop calibrator, the bridge screen and the handheld's drive screen.
- [Development](development.md): the verification gates, the repository layout and regenerating the screenshots.

The staged hardware bring-up (power, wiring, flashing, the bench checks) is the [`spooder-hardware-check`](../.claude/skills/spooder-hardware-check/SKILL.md) skill. Intended behaviour is specified in [`openspec/`](../openspec/), which is the source of truth when a page here and a specification disagree.
