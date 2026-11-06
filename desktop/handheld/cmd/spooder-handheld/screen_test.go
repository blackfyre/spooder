package main

import (
	"math"
	"slices"
	"strings"
	"testing"
	"time"

	"github.com/veandco/go-sdl2/sdl"
	"github.com/veandco/go-sdl2/ttf"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/internal/legview"
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
					w += s.hintKeyWidth(k.label) + l.gap(4)
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
	heroW := textWidth(f.hero, "Calibrate.") + s.chipWidth("OUTPUT SIM", true) + l.gap(16)
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

	// The driving-view page: the well and the cells beside it share the panel,
	// and the cells fit their values.
	wellH := l.bodyBottom - l.gap(panelPad) - (l.bodyY + l.gap(panelPad) + l.microH + l.gap(kickerGap))
	wellW := l.driveViewWellW(wellH)
	sideW := l.w - 2*l.gap(margin) - 2*l.gap(panelPad) - wellW - l.gap(28)
	if wellW < wellH/2 || sideW < l.colW/2 {
		t.Errorf("driving view well %d×%d, cells %d wide", wellW, wellH, sideW)
	}
	vcW := l.cellW(sideW, viewCols)
	vcs := driveViewCells(app.Robot{Have: true, Body: "Calibrate", Gait: "Tripod", Gesture: "say hi", OutputsOn: true, Simulated: true, VoltageMv: 65535}, colText)
	// The cells' rows and a row of fault chips fit the panel.
	if bottom, need := l.bodyBottom-l.gap(panelPad), l.bodyY+l.gap(panelPad)+l.microH+l.gap(kickerGap)+
		int32((len(vcs)+viewCols-1)/viewCols)*(l.cellH()+l.gap(cellGap))+l.microH+l.gap(labelGap)+l.gap(2)+l.gap(chipHeight); need > bottom {
		t.Errorf("the driving view's cells and faults end at %d, the panel at %d", need, bottom)
	}
	for _, cl := range vcs {
		vf := f.value
		if cl.mono {
			vf = f.mono
		}
		if w := textWidth(vf, cl.value); w > vcW {
			t.Errorf("driving view value %q is %d wide, cell %d", cl.value, w, vcW)
		}
	}

	// The header's chips and the title share the width.
	chips := s.chipWidth("SIM", false) + s.chipWidth("LINK STALE", true) + s.chipWidth("-100 DBM", true) + s.chipWidth("E-STOP OFF", false) + 3*l.gap(10)
	if title := textWidth(f.title, "Spooder."); l.left+title+l.gap(20) > l.w-l.gap(margin)-chips {
		t.Errorf("the title (%d) and the chips (%d) overlap", title, chips)
	}
}

func TestSignalTone(t *testing.T) {
	for _, c := range []struct {
		known bool
		dbm   int
		want  sdl.Color
	}{
		{false, -40, colMuted},
		{true, -40, colOk},
		{true, signalGood, colOk},
		{true, signalGood - 1, colWarn},
		{true, signalWeak, colWarn},
		{true, signalWeak - 1, colAccent},
	} {
		if got := signalTone(c.known, c.dbm); got != c.want {
			t.Errorf("signalTone(%v, %d) = %v, want %v", c.known, c.dbm, got, c.want)
		}
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

// The chase camera looks at its target from behind and above: the target
// draws at the centre, the front higher than the rear, the robot's right on
// the right, the rear (nearer) larger than the front, and every point within
// the legs' reach in front of the camera.
func TestChaseCamera(t *testing.T) {
	if q, d := chase.project(chaseTarget); math.Abs(q.X) > 1e-9 || math.Abs(q.Y) > 1e-9 || math.Abs(d-chaseDistance) > 1e-9 {
		t.Errorf("target projects to %v at depth %.1f", q, d)
	}
	front, df := chase.project(legview.Point3{Y: 200})
	rear, dr := chase.project(legview.Point3{Y: -200})
	if front.Y <= rear.Y || df <= dr {
		t.Errorf("front %v at %.0f, rear %v at %.0f: want the front higher and farther", front, df, rear, dr)
	}
	if right, _ := chase.project(legview.Point3{X: 200}); right.X <= 0 {
		t.Errorf("the robot's right projects to %v", right)
	}
	lo, hi := reachBox()
	for _, x := range []float64{lo.X, hi.X} {
		for _, y := range []float64{lo.Y, hi.Y} {
			for _, z := range []float64{lo.Z, hi.Z} {
				if _, d := chase.project(legview.Point3{X: x, Y: y, Z: z}); d <= 0 {
					t.Errorf("corner (%.0f, %.0f, %.0f) behind the camera", x, y, z)
				}
			}
		}
	}
}

// The driving view names the gesture the robot plays beside its body mode
// (design D1 of handheld-observations): from fresh telemetry, None when
// none plays, a dash without telemetry.
func TestDriveViewNamesGesture(t *testing.T) {
	value := func(cs []cell, label string) (string, bool) {
		for _, c := range cs {
			if c.label == label {
				return c.value, true
			}
		}
		return "", false
	}
	for _, c := range []struct {
		rb   app.Robot
		want string
	}{
		{app.Robot{Have: true, Body: "Perform", Gesture: "say hi", GestureID: 1}, "say hi"},
		{app.Robot{Have: true, Body: "Stand"}, "None"},
		{app.Robot{}, "—"},
	} {
		cs := driveViewCells(c.rb, colText)
		got, ok := value(cs, "GESTURE")
		if !ok || got != c.want {
			t.Errorf("driving view with %+v: gesture %q (shown %v), want %q", c.rb, got, ok, c.want)
		}
		if i := slices.IndexFunc(cs, func(c cell) bool { return c.label == "GESTURE" }); i != 1 {
			t.Errorf("the gesture is cell %d, want 1, beside the body mode", i)
		}
	}
}

// A requested value's hint key presses while its control is held, as the
// footer's keycaps do, except while Select or L2 turns the control to
// another function (design D2 of handheld-observations).
func TestHintKeysPressWhileHeld(t *testing.T) {
	for _, c := range []struct {
		b, held padmap.Buttons
		want    bool
	}{
		{padmap.HandUp, padmap.HandUp, true},
		{padmap.HandUp, padmap.HandDown, false},
		{padmap.HandUp, 0, false},
		{padmap.HandL1, padmap.HandL1 | padmap.HandR1, true},
		{padmap.HandY, padmap.HandY, true},
		{padmap.HandY, padmap.HandSelect | padmap.HandY, false},
		{padmap.HandStart, padmap.HandSelect | padmap.HandStart, false},
		{padmap.HandX, padmap.HandSelect | padmap.HandX, false},
		{padmap.HandB, padmap.HandSelect | padmap.HandB, true},
		{padmap.HandUp, padmap.HandSelect | padmap.HandUp, true},
		{padmap.HandB, padmap.HandL2 | padmap.HandB, false},
		{padmap.HandX, padmap.HandL2 | padmap.HandX, false},
		{padmap.HandStart, padmap.HandL2 | padmap.HandStart, true},
		{0, padmap.HandA, false},
	} {
		if got := hintPressed(c.b, c.held); got != c.want {
			t.Errorf("hintPressed(%b, held %b) = %v, want %v", c.b, c.held, got, c.want)
		}
	}
	// Every requested value's keys name a control, so each can press.
	for _, cl := range requestedCells(app.Requested{}) {
		for _, k := range cl.keys {
			if k.b == 0 {
				t.Errorf("%s key %q has no control", cl.label, k.label)
			}
		}
	}
}
