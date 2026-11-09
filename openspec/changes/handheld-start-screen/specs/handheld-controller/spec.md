## ADDED Requirements

### Requirement: Blackfyre screens
Every screen the handheld shows, from start-up to exit, SHALL follow the Blackfyre design system's dark app palette and type: the window in `bf-ink`, panels in `bf-ink-raised` with a black outline, wells in `bf-ink-code`, text in `bf-surface`, labels, kickers, chips and keycaps in uppercase tracked JetBrains Mono, labels in `bf-ink-subtle`, what is read in Archivo, `bf-accent` the one accent with `bf-accent-on-dark` for orange text and `bf-on-accent` on orange fills, and status fills (`bf-ok`, `bf-warn`) always with a label. No screen SHALL be drawn by gabagool's or NextUI's own components. The start-up messages (the join message and any other start-up error) SHALL show on the pak's own start screen: the `Spooder.` title, a panel with the message and keycaps to retry (A) and quit (B); a press of A SHALL retry the start, and a press of B or Menu SHALL quit. While a start check runs, the start screen SHALL say so.

#### Scenario: Join message in the Blackfyre look
- **WHEN** the pak starts on a network that is not a bridge's WiFi drive network
- **THEN** it shows the join message on its start screen, in the Blackfyre palette, with the A RETRY and B QUIT keycaps, not on a gabagool message screen

#### Scenario: Retry from the start screen
- **WHEN** the join message shows, the user joins the bridge's network and presses A
- **THEN** the start screen shows the check under way, then the drive screen

#### Scenario: Held A from the launcher
- **WHEN** the join message shows while A is still held from launching the pak, and A is released
- **THEN** the start is not retried until A is pressed again
