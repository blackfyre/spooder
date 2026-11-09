package main

import (
	"errors"
	"strings"

	"github.com/veandco/go-sdl2/sdl"
	"github.com/veandco/go-sdl2/ttf"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
)

// The start screen (openspec change handheld-start-screen): before the pak
// drives, the start check under way or its failure (the join message, or
// any other start-up error) with the keys to retry or quit, in the drive
// screen's Blackfyre look and drawn by its parts.

// startPage is what the start screen shows.
type startPage struct {
	kicker   string // the panel's kicker: JOIN or START
	message  string
	checking bool // a start check is under way: no keys
}

// checkingPage is the start screen while app.Start runs: on the device the
// WiFi check (whose lease renewal can take seconds), else opening the link.
func checkingPage(device bool, addr string) startPage {
	if device {
		return startPage{kicker: "JOIN", message: "Checking the WiFi network…", checking: true}
	}
	return startPage{kicker: "START", message: "Opening the link to " + addr + "…", checking: true}
}

// failedPage is the start screen for a failed start: the join message under
// JOIN, any other error under START.
func failedPage(err error) startPage {
	var nj *wlan.NotJoinedError
	if errors.As(err, &nj) {
		return startPage{kicker: "JOIN", message: err.Error()}
	}
	return startPage{kicker: "START", message: err.Error()}
}

// startKeys are the start screen's keycaps and what they do.
var startKeys = []struct {
	k    key
	does string
}{
	{key{"A", padmap.HandA}, "RETRY"},
	{key{"B", padmap.HandB}, "QUIT"},
}

// startAction is what an event asks of the start screen.
type startAction int

const (
	startNone startAction = iota
	startRetry
	startQuit
)

// startEvent reads one SDL event on the start screen (design D3): a press of
// A, or Enter on a desktop keyboard, retries; a press of B or Menu, closing
// the window or Escape quits. Only presses act, so a button still held from
// launching the pak does nothing until pressed again.
func startEvent(ev sdl.Event) startAction {
	switch e := ev.(type) {
	case *sdl.QuitEvent:
		return startQuit
	case *sdl.ControllerButtonEvent:
		if e.Type != sdl.CONTROLLERBUTTONDOWN {
			return startNone
		}
		b := sdl.GameControllerButton(e.Button)
		switch {
		case b == menuButton || handButton(b) == padmap.HandB:
			return startQuit
		case handButton(b) == padmap.HandA:
			return startRetry
		}
	case *sdl.KeyboardEvent:
		if e.Type != sdl.KEYDOWN || e.Repeat != 0 {
			return startNone
		}
		switch e.Keysym.Sym {
		case sdl.K_ESCAPE:
			return startQuit
		case sdl.K_RETURN, sdl.K_KP_ENTER:
			return startRetry
		}
	}
	return startNone
}

// handButton is the TrimUI control an SDL controller button is (pad.go's
// table), or 0.
func handButton(b sdl.GameControllerButton) padmap.Buttons {
	for _, e := range buttonTable {
		if e.sdl == b {
			return e.hand
		}
	}
	return 0
}

// wrap breaks str at spaces into lines at most width wide in f (design D2).
// A word wider than width stays on its own line, for text to shorten.
func wrap(f *ttf.Font, str string, width int32) []string {
	var lines []string
	line := ""
	for _, word := range strings.Fields(str) {
		next := word
		if line != "" {
			next = line + " " + word
		}
		if line != "" && textWidth(f, next) > width {
			lines = append(lines, line)
			line = word
			continue
		}
		line = next
	}
	if line != "" {
		lines = append(lines, line)
	}
	return lines
}

// startPanel is the start screen's panel: its position, and the message's
// inner width.
func (l layout) startPanel() (x, y, w, h, innerW int32) {
	x, y = l.left, l.bodyY
	w, h = l.w-2*l.gap(margin), l.bodyBottom-l.bodyY
	return x, y, w, h, w - 2*l.gap(panelPad)
}

// start draws the start screen: the title, one panel with the page's
// message, and, unless a check is under way, the keycaps, pressed while
// held.
func (s *screen) start(p startPage, held padmap.Buttons) {
	l := s.l
	s.fill(0, 0, l.w, l.h, 0, colWindow)
	s.title()
	x, y, w, h, iw := l.startPanel()
	ty := s.panel(x, y, w, h, "01", p.kicker, colMuted)
	ix := x + l.gap(panelPad)
	bottom := y + h - l.gap(panelPad)
	lh := int32(s.f.value.LineSkip())
	for _, line := range wrap(s.f.value, p.message, iw) {
		if ty+lh > bottom {
			break
		}
		s.text(s.f.value, line, ix, ty, iw, colText)
		ty += lh
	}
	if !p.checking {
		s.startFooter(held)
	}
	s.endFrame()
	s.win.Present()
}

// startFooter draws the start screen's keycaps and what they do, as the
// drive screen's footer does.
func (s *screen) startFooter(held padmap.Buttons) {
	l := s.l
	x, y := l.left, l.footerY
	ty := y + (l.gap(keyHeight)-l.microH)/2
	for i, sk := range startKeys {
		if i > 0 {
			x += l.gap(28)
		}
		x += s.keycap(x, y, sk.k.label, held&sk.k.b != 0) + l.gap(10)
		x += s.tracked(s.f.micro, sk.does, x, ty, colMuted, trackLabel)
	}
}
