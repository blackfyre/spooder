package main

import (
	"strings"
	"testing"
	"time"

	"github.com/veandco/go-sdl2/ttf"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
)

func testFonts(t *testing.T) *fontSet {
	t.Helper()
	if err := ttf.Init(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(ttf.Quit)
	f, err := loadFonts(1)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(f.close)
	return f
}

func width(t *testing.T, f *ttf.Font, s string) int32 {
	t.Helper()
	w, _, err := f.SizeUTF8(s)
	if err != nil {
		t.Fatal(err)
	}
	return int32(w)
}

// The drive screen at the TrimUI's 1280×720: every block above the help
// line, room for a two-line leave banner, and the widest ordinary values
// and every label within their cells.
func TestLayoutFitsTheDevice(t *testing.T) {
	f := testFonts(t)
	l := newLayout(1280, 720, f)

	robot := robotRows(app.Robot{Have: true, Body: "Stand", Gait: "Tripod", Faults: []string{"overcurrent"}, Gesture: "say hi", VoltageMv: 12600, CurrentMa: 25000})
	requested := requestedRows(app.Requested{Mode: "Calibrate", Height: 255, Speed: 255, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer", EStop: true})
	link := linkRows(app.LinkView{SignalKnown: true, SignalDBm: -100, Telemetry: 100, Pose: 100, Diagnostics: 100, Invalid: 99999, Lost: 99999, Late: 99999, HaveFrame: true, SinceLast: 9999 * time.Second, Sent: 9999999, SendErrors: 99999})
	for _, c := range []struct {
		name string
		rows []row
		n    int
	}{{"robot", robot, robotRowCount}, {"requested", requested, requestedRowCount}, {"link", link, linkRowCount}} {
		if len(c.rows) != c.n {
			t.Errorf("%s: %d rows, layout has %d", c.name, len(c.rows), c.n)
		}
		for _, r := range c.rows {
			if w := width(t, f.label, r.label); w > l.valueOff-l.gap(8) {
				t.Errorf("%s label %q is %d wide, cell %d", c.name, r.label, w, l.valueOff-l.gap(8))
			}
			if r.label == "Faults" {
				continue // shortened to fit when several faults hold
			}
			if w := width(t, f.mono, r.value); w > l.colW-l.valueOff {
				t.Errorf("%s value %q is %d wide, cell %d", c.name, r.value, w, l.colW-l.valueOff)
			}
		}
	}
	t.Logf("right column ends at %d, banner at %d, help line at %d (row %d)", l.rightBottom, l.bannerY, l.footerY, l.rowH)
	if l.rightBottom > l.footerY {
		t.Errorf("right column ends at %d, help line starts at %d", l.rightBottom, l.footerY)
	}
	if l.bannerY+2*l.titleH > l.footerY {
		t.Errorf("leave banner from %d leaves no two lines above %d", l.bannerY, l.footerY)
	}
	if w := width(t, f.label, helpText); w > 2*l.colW+l.gap(columnGap) {
		t.Errorf("help line is %d wide, screen %d", w, 2*l.colW+l.gap(columnGap))
	}
}

func TestFitShortensWithEllipsis(t *testing.T) {
	f := testFonts(t)
	long := strings.Repeat("overcurrent, ", 10)
	got := fit(f.mono, long, 200)
	if !strings.HasSuffix(got, "…") || width(t, f.mono, got) > 200 {
		t.Errorf("fit(%q) = %q, %d wide", long, got, width(t, f.mono, got))
	}
	if got := fit(f.mono, "Stand", 200); got != "Stand" {
		t.Errorf("fit(Stand) = %q", got)
	}
}
