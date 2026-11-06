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

// The drive screen at the TrimUI's 1280×720: both right-hand panels hold
// their three rows of cells, the widest ordinary values and every label fit
// their cells, and the keycaps fit across the screen.
func TestLayoutFitsTheDevice(t *testing.T) {
	f := testFonts(t)
	l := newLayout(1280, 720, f)
	s := &screen{f: f, l: l}

	inner := l.colW - 2*l.gap(panelPad)
	requested := requestedCells(app.Requested{Mode: "Calibrate", Height: 255, Speed: 255, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer", EStop: true})
	link := linkCells(app.LinkView{SignalKnown: true, SignalDBm: -100, Telemetry: 100, Pose: 100, Diagnostics: 100, Invalid: 99999, Lost: 9999, Late: 9999, HaveFrame: true, SinceLast: 9999 * time.Second, Sent: 9999999, SendErrors: 999})
	for _, c := range []struct {
		name       string
		cells      []cell
		rows, cols int
		height     int32
	}{{"requested", requested, requestedRows, requestedCols, l.requested}, {"link", link, linkRows, linkCols, l.link}} {
		cellW := l.cellW(inner, c.cols)
		if len(c.cells) != c.rows*c.cols {
			t.Errorf("%s: %d cells, the layout has %d", c.name, len(c.cells), c.rows*c.cols)
		}
		if need := l.gridH(c.rows); need > c.height {
			t.Errorf("%s: panel %d high, its rows need %d", c.name, c.height, need)
		}
		for _, cl := range c.cells {
			w := trackedWidth(f.micro, cl.label, trackMicro)
			if len(cl.keys) > 0 {
				w += l.gap(10)
				for _, k := range cl.keys {
					w += s.hintKeyWidth(k) + l.gap(4)
				}
			}
			if w > cellW {
				t.Errorf("%s label %q is %d wide, cell %d", c.name, cl.label, w, cellW)
			}
			vf := f.value
			if cl.mono {
				vf = f.mono
			}
			if w := textWidth(vf, cl.value); w > cellW {
				t.Errorf("%s value %q is %d wide, cell %d", c.name, cl.value, w, cellW)
			}
		}
	}

	// The robot panel's gait, gesture and source.
	rcW := l.cellW(inner, robotCols)
	for _, cl := range robotCells(app.Robot{Have: true, Gait: "Tripod", Gesture: "say hi", Source: "conflict"}, colText) {
		if w := textWidth(f.value, cl.value); w > rcW {
			t.Errorf("robot value %q is %d wide, cell %d", cl.value, w, rcW)
		}
	}

	// The robot's panel: the hero with its chip, the well, gait and
	// gesture, and a row of fault chips with a notice still fit.
	heroW := textWidth(f.hero, "Calibrate.") + s.chipWidth("OUTPUT OFF", true) + l.gap(16)
	if heroW > inner {
		t.Errorf("the body mode and its chip need %d, the panel has %d", heroW, inner)
	}
	used := l.bodyY + l.gap(panelPad) + l.microH + l.gap(kickerGap) - l.gap(8) + int32(f.hero.Height()) + l.gap(10) +
		l.gap(18) + l.microH + l.gap(labelGap) + int32(f.big.Height()) + l.gap(18) + l.gap(22) +
		l.cellH() + l.gap(cellGap) + l.microH + l.gap(labelGap) + l.gap(2) + l.gap(chipHeight)
	if bottom := l.bodyBottom - l.gap(panelPad) - l.microH - l.gap(12); used > bottom {
		t.Errorf("the robot panel's content ends at %d, the notice line starts at %d", used, bottom)
	}

	// The keycaps along the bottom.
	x := l.left
	for i, h := range helpKeys {
		if i > 0 {
			x += l.gap(28)
		}
		for j, k := range h.keys {
			if j > 0 {
				x += 2*l.gap(6) + trackedWidth(f.micro, "+", trackLabel)
			}
			x += s.keycapWidth(k.label)
		}
		x += l.gap(10) + max(trackedWidth(f.micro, h.does, trackLabel), trackedWidth(f.micro, h.onView, trackLabel))
	}
	if x > l.w-l.gap(margin) {
		t.Errorf("the controls end at %d, the screen's margin at %d", x, l.w-l.gap(margin))
	}

	// The top-view page: the well and the cells beside it share the panel,
	// and the cells fit their values.
	wellH := l.bodyBottom - l.gap(panelPad) - (l.bodyY + l.gap(panelPad) + l.microH + l.gap(kickerGap))
	wellW := l.topViewWellW(wellH)
	sideW := l.w - 2*l.gap(margin) - 2*l.gap(panelPad) - wellW - l.gap(28)
	if wellW < wellH/2 || sideW < l.colW/2 {
		t.Errorf("top view well %d×%d, cells %d wide", wellW, wellH, sideW)
	}
	vcW := l.cellW(sideW, viewCols)
	for _, cl := range topViewCells(app.Robot{Have: true, Body: "Calibrate", Gait: "Tripod", VoltageMv: 65535}, colText) {
		vf := f.value
		if cl.mono {
			vf = f.mono
		}
		if w := textWidth(vf, cl.value); w > vcW {
			t.Errorf("top view value %q is %d wide, cell %d", cl.value, w, vcW)
		}
	}

	// The header's chips and the title share the width.
	chips := s.chipWidth("LINK STALE", true) + s.chipWidth("-100 DBM", false) + s.chipWidth("E-STOP OFF", false) + 2*l.gap(10)
	if title := textWidth(f.title, "Spooder."); l.left+title+l.gap(20) > l.w-l.gap(margin)-chips {
		t.Errorf("the title (%d) and the chips (%d) overlap", title, chips)
	}
}

func TestFitShortensWithEllipsis(t *testing.T) {
	f := testFonts(t)
	long := strings.Repeat("overcurrent, ", 10)
	got := fit(f.mono, long, 200)
	if !strings.HasSuffix(got, "…") || textWidth(f.mono, got) > 200 {
		t.Errorf("fit(%q) = %q, %d wide", long, got, textWidth(f.mono, got))
	}
	if got := fit(f.mono, "Stand", 200); got != "Stand" {
		t.Errorf("fit(Stand) = %q", got)
	}
}

// Tracking widens text by the tracking between each pair of characters.
func TestTrackedWidthAddsTracking(t *testing.T) {
	f := testFonts(t)
	plain := trackedWidth(f.micro, "LINK", 0)
	if got, want := trackedWidth(f.micro, "LINK", trackMicro), plain+3*tracking(f.micro, trackMicro); got != want {
		t.Errorf("tracked LINK = %d, want %d", got, want)
	}
}
