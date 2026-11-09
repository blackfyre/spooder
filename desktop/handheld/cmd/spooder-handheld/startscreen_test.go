package main

import (
	"errors"
	"strings"
	"testing"

	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
)

// wrap keeps every word, in order, on lines no wider than the width.
func TestWrapBreaksAtSpaces(t *testing.T) {
	f := testFonts(t)
	msg := "Join the spooder-drive-XXXX network in NextUI's WiFi settings, with the password shown on the bridge's link page."
	width := textWidth(f.value, "Join the spooder-drive-XXXX network")
	lines := wrap(f.value, msg, width)
	if len(lines) < 3 {
		t.Fatalf("%d lines, want at least 3: %q", len(lines), lines)
	}
	for _, l := range lines {
		if w := textWidth(f.value, l); w > width {
			t.Errorf("line %q is %d wide, the width is %d", l, w, width)
		}
	}
	if got := strings.Join(lines, " "); got != msg {
		t.Errorf("rejoined %q, want %q", got, msg)
	}
	if got := wrap(f.value, "   ", width); len(got) != 0 {
		t.Errorf("blank text wraps to %q", got)
	}
}

// The join message, with the longest network name WiFi allows, fits the
// start screen's panel at the TrimUI's 1280×720 without dropping a line.
func TestJoinMessageFitsTheStartPanel(t *testing.T) {
	f := testFonts(t)
	l := newLayout(1280, 720, f)
	_, _, _, h, iw := l.startPanel()
	msg := (&wlan.NotJoinedError{Network: strings.Repeat("W", 32)}).Error()
	lines := wrap(f.value, msg, iw)
	avail := h - 2*l.gap(panelPad) - l.microH - l.gap(kickerGap)
	if need := int32(len(lines)) * int32(f.value.LineSkip()); need > avail {
		t.Errorf("%d lines need %d, the panel holds %d", len(lines), need, avail)
	}
	for _, line := range lines {
		if textWidth(f.value, line) > iw {
			t.Errorf("line %q wider than the panel", line)
		}
	}
}

func TestFailedPageKicker(t *testing.T) {
	if p := failedPage(&wlan.NotJoinedError{}); p.kicker != "JOIN" || p.checking {
		t.Errorf("join message: %+v", p)
	}
	if p := failedPage(errors.New("dial udp: no route")); p.kicker != "START" || p.message != "dial udp: no route" {
		t.Errorf("other error: %+v", p)
	}
	if p := checkingPage(true, ""); !p.checking || p.kicker != "JOIN" {
		t.Errorf("checking on the device: %+v", p)
	}
}

// Presses act, releases do not (design D3); the TrimUI's A is SDL's B.
func TestStartEvent(t *testing.T) {
	button := func(typ uint32, b sdl.GameControllerButton) sdl.Event {
		return &sdl.ControllerButtonEvent{Type: typ, Button: uint8(b)}
	}
	key := func(sym sdl.Keycode, repeat uint8) sdl.Event {
		return &sdl.KeyboardEvent{Type: sdl.KEYDOWN, Repeat: repeat, Keysym: sdl.Keysym{Sym: sym}}
	}
	for _, c := range []struct {
		name string
		ev   sdl.Event
		want startAction
	}{
		{"A pressed", button(sdl.CONTROLLERBUTTONDOWN, sdl.CONTROLLER_BUTTON_B), startRetry},
		{"A released", button(sdl.CONTROLLERBUTTONUP, sdl.CONTROLLER_BUTTON_B), startNone},
		{"B pressed", button(sdl.CONTROLLERBUTTONDOWN, sdl.CONTROLLER_BUTTON_A), startQuit},
		{"Menu pressed", button(sdl.CONTROLLERBUTTONDOWN, menuButton), startQuit},
		{"Y pressed", button(sdl.CONTROLLERBUTTONDOWN, sdl.CONTROLLER_BUTTON_X), startNone},
		{"Enter", key(sdl.K_RETURN, 0), startRetry},
		{"Enter repeating", key(sdl.K_RETURN, 1), startNone},
		{"Escape", key(sdl.K_ESCAPE, 0), startQuit},
		{"window closed", &sdl.QuitEvent{Type: sdl.QUIT}, startQuit},
	} {
		if got := startEvent(c.ev); got != c.want {
			t.Errorf("%s: %v, want %v", c.name, got, c.want)
		}
	}
}
