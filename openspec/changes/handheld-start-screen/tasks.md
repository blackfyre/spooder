## 1. Start screen

- [x] 1.1 Draw the start screen in `cmd/spooder-handheld` with the drive screen's title, panel and keycaps (`startscreen.go`), the message wrapped to the panel; make the screen once in `main` and use it for the join loop and the drive loop; replace `gabagool.ConfirmationMessage` with the join loop on that screen (A retries, B or Menu quits, keys act on press, the check in progress shown)
- [x] 1.2 Tests: the wrap, the key handling and the join message fitting its panel at 1280×720; the start states in the drawing test
- [x] 1.3 The start screen's screenshot in `docs/`, the docs that describe the join message, `desktop/CLAUDE.md` and the changelog

## 2. Gates

- [x] 2.1 `mise run desktop-test`, `handheld-test`, `handheld-build`, `handheld-package`, `e2e`
- [ ] 2.2 On the TrimUI: the join message on the start screen in the Blackfyre look, A retries, B and Menu quit (requires the user)
