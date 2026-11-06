package main

import (
	"math"
	"strings"
	"testing"
	"time"
	"unsafe"

	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/gestures"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
)

// Each tile's button, pressed with L2 beyond half travel, requests the
// tile's gesture through the mapping (design D3): the overlay cannot drift
// from padmap without this failing.
func TestTileButtonsRequestTheirGesture(t *testing.T) {
	l2 := padmap.Controls{TriggerL: padmap.TriggerThreshold + 1}
	for _, tile := range gestureTiles {
		h := padmap.NewHandheld()
		ms := uint32(0)
		h.Begin(ms)
		step := func(c padmap.Controls) padmap.Desired {
			ms += 20
			return h.Step(c, ms)
		}
		step(padmap.Controls{})
		before := step(l2)
		c := l2
		if tile.b == padmap.HandR2 {
			c.TriggerR = padmap.TriggerThreshold + 1
		} else {
			c.Buttons = tile.b
		}
		d := step(c)
		if d.Gesture != tile.gesture || d.GestureRequestCounter != before.GestureRequestCounter+1 {
			t.Errorf("L2 + %s: gesture %d #%d, want %d #%d", tile.key, d.Gesture, d.GestureRequestCounter, tile.gesture, before.GestureRequestCounter+1)
		}
	}
}

// Every gesture of the L2 layer has one tile, and every tile a preview.
func TestTilesCoverTheTable(t *testing.T) {
	if len(gestureTiles) != len(gestures.Table) {
		t.Fatalf("%d tiles, %d gestures", len(gestureTiles), len(gestures.Table))
	}
	seen := map[uint8]bool{}
	for _, tile := range gestureTiles {
		if _, ok := preview(tile.gesture); !ok || seen[tile.gesture] {
			t.Errorf("tile %s: gesture %d missing or repeated", tile.key, tile.gesture)
		}
		seen[tile.gesture] = true
	}
	// Every script starts from the standing robot the unscripted tiles show.
	for _, g := range gestures.Table {
		if g.Scripted && g.Frames[0] != standingFrame() {
			t.Errorf("gesture %s starts from %v, not the standing frame", g.Name, g.Frames[0])
		}
	}
}

// With a fixed clock the shown frame steps every StepMs, holds the last
// frame for loopHold and starts again.
func TestFrameAtLoops(t *testing.T) {
	g, _ := preview(2) // wiggle
	n := len(g.Frames)
	step := time.Duration(g.StepMs) * time.Millisecond
	run := time.Duration(n-1) * step
	for _, c := range []struct {
		at   time.Duration
		want int
	}{
		{0, 0}, {step - time.Millisecond, 0}, {step, 1}, {3*step + step/2, 3},
		{run - time.Millisecond, n - 2}, {run, n - 1}, {run + loopHold - time.Millisecond, n - 1},
		{run + loopHold, 0}, {run + loopHold + step, 1}, {2*(run+loopHold) + 2*step, 2},
	} {
		if got := frameAt(g, c.at); got != c.want {
			t.Errorf("frame at %v = %d, want %d", c.at, got, c.want)
		}
	}
	if got := frameAt(gestures.Gesture{StepMs: 50}, time.Second); got != 0 {
		t.Errorf("no frames: frame %d", got)
	}
}

// The overlay opens with L2 held and its animation runs from then; a
// request counter change lights its tile for requestLit, the first frame
// lights none; the playing tile is lit only from fresh telemetry.
func TestOverlayState(t *testing.T) {
	t0 := time.Unix(1000, 0)
	var o overlayState
	sc := app.Screen{Requested: app.Requested{Gesture: 1, GestureSeq: 7}}
	o.update(sc, t0)
	if o.open {
		t.Fatal("open without L2")
	}
	for i, st := range o.tileStates(sc, t0) {
		if st.lit() {
			t.Errorf("tile %s lit by the first frame's counter", gestureTiles[i].key)
		}
	}

	sc.Held = padmap.HandL2
	open := t0.Add(time.Second)
	o.update(sc, open)
	if !o.open || !o.openedAt.Equal(open) {
		t.Fatalf("open %v since %v, want since %v", o.open, o.openedAt, open)
	}
	o.update(sc, open.Add(time.Second))
	if !o.openedAt.Equal(open) {
		t.Fatal("the animation restarted while L2 stayed held")
	}
	wiggle, _ := preview(2)
	at := open.Add(time.Duration(3*wiggle.StepMs) * time.Millisecond)
	if st := o.tileStates(sc, at)[1]; st.frame != wiggle.Frames[3] || !st.scripted {
		t.Errorf("wiggle tile at 3 steps: frame %v, want %v", st.frame, wiggle.Frames[3])
	}
	dance, _ := preview(3)
	if st := o.tileStates(sc, at)[2]; !st.scripted || st.frame != dance.Frames[3] {
		t.Errorf("dance tile: scripted %v, frame %v, want %v", st.scripted, st.frame, dance.Frames[3])
	}

	// Releasing L2 closes it; pressing it again restarts the animations.
	sc.Held = 0
	o.update(sc, at)
	if o.open {
		t.Fatal("still open after L2 was released")
	}
	sc.Held = padmap.HandL2
	reopen := at.Add(time.Second)
	o.update(sc, reopen)
	if !o.open || !o.openedAt.Equal(reopen) {
		t.Fatalf("reopened %v since %v, want since %v", o.open, o.openedAt, reopen)
	}
	if st := o.tileStates(sc, reopen)[1]; st.frame != wiggle.Frames[0] {
		t.Errorf("wiggle tile on reopening: frame %v, want frame 0", st.frame)
	}

	// A press of A: the counter advances with say hi.
	req := open.Add(2 * time.Second)
	sc.Requested = app.Requested{Gesture: 1, GestureSeq: 8}
	o.update(sc, req)
	lit := func(at time.Time) []bool {
		var out []bool
		for _, st := range o.tileStates(sc, at) {
			out = append(out, st.requested)
		}
		return out
	}
	if got := lit(req.Add(requestLit - time.Millisecond)); !got[0] || got[1] || got[2] || got[3] || got[4] {
		t.Errorf("lit %v just before %v, want say hi alone", got, requestLit)
	}
	if got := lit(req.Add(requestLit)); got[0] {
		t.Errorf("say hi still lit at %v", requestLit)
	}

	// Wiggle playing: lit from fresh telemetry only.
	for _, c := range []struct {
		rb   app.Robot
		want bool
	}{
		{app.Robot{Have: true, Body: "Perform", GestureID: 2}, true},
		{app.Robot{Have: true, Stale: true, Body: "Perform", GestureID: 2}, false},
		{app.Robot{Have: false, Stale: true}, false},
		{app.Robot{Have: true, Body: "Stand", GestureID: 0}, false},
	} {
		sc.Robot = c.rb
		st := o.tileStates(sc, req.Add(time.Second))
		if st[1].playing != c.want || st[1].lit() != c.want || st[0].playing {
			t.Errorf("robot %+v: wiggle playing %v, want %v", c.rb, st[1].playing, c.want)
		}
		if got := tileChips(st[1]); c.want != (len(got) == 1 && got[0] == labelPlaying) {
			t.Errorf("robot %+v: wiggle chips %v", c.rb, got)
		}
	}
}

func TestStandFirst(t *testing.T) {
	for _, c := range []struct {
		rb   app.Robot
		want bool
	}{
		{app.Robot{Stale: true}, true},
		{app.Robot{Have: true, Stale: true, Body: "Stand"}, true},
		{app.Robot{Have: true, Body: "Sleep"}, true},
		{app.Robot{Have: true, Body: "Walk"}, true},
		{app.Robot{Have: true, Body: "Stand"}, false},
		{app.Robot{Have: true, Body: "Perform"}, false},
	} {
		if got := standFirst(c.rb); got != c.want {
			t.Errorf("standFirst(%+v) = %v, want %v", c.rb, got, c.want)
		}
	}
}

// The overlay fits the device: its panel, tiles, names, keycaps, status
// chips, title and STAND FIRST inside 1280×720 without overlapping, and
// clear of the header and the footer.
func TestOverlayLayoutFits(t *testing.T) {
	f := testFonts(t)
	l := newLayout(1280, 720, f)
	s := &screen{f: f, l: l}
	g := s.overlayGeom()
	screenR := rect{0, 0, 1280, 720}
	if !g.panel.inside(screenR) || g.panel.y < l.headerY+l.gap(headerH) || g.panel.y+g.panel.h > l.footerY {
		t.Errorf("panel %+v: off the screen or over the header or footer", g.panel)
	}
	type named struct {
		what string
		r    rect
	}
	var all []named
	for i, tg := range g.tiles {
		k := gestureTiles[i].key
		if !tg.face.inside(g.panel) {
			t.Errorf("tile %s %+v outside the panel", k, tg.face)
		}
		inner := rect{tg.face.x + l.gap(tilePad), tg.face.y + l.gap(tilePad), tg.face.w - 2*l.gap(tilePad), tg.face.h - 2*l.gap(tilePad)}
		for _, part := range []named{{"key", tg.key}, {"name", tg.name}, {"picture", tg.picture}, {"status", tg.status}} {
			if !part.r.inside(inner) {
				t.Errorf("tile %s: %s %+v outside %+v", k, part.what, part.r, inner)
			}
		}
		// The widest chips a tile can show: REQUESTED or PLAYING, and NOT YET.
		lit := max(s.tileChipWidth(labelRequested), s.tileChipWidth(labelPlaying))
		if w := lit + l.gap(8) + s.tileChipWidth(labelNotYet); w > tg.status.w {
			t.Errorf("tile %s: chips need %d, the row has %d", k, w, tg.status.w)
		}
		if tg.picture.h < tg.face.h/2 {
			t.Errorf("tile %s: picture %d high in a tile %d high", k, tg.picture.h, tg.face.h)
		}
		all = append(all, named{"tile " + k, tg.face}, named{k + " key", tg.key}, named{k + " name", tg.name},
			named{k + " picture", tg.picture}, named{k + " status", tg.status})
	}
	for _, r := range []named{{"title", g.title}, {"stand first", g.standFirst}} {
		if !r.r.inside(g.panel) {
			t.Errorf("%s %+v outside the panel", r.what, r.r)
		}
		all = append(all, r)
	}
	for i := range all {
		for j := i + 1; j < len(all); j++ {
			a, b := all[i], all[j]
			// A tile's own parts lie on its face.
			if strings.HasPrefix(a.what, "tile ") && strings.HasPrefix(b.what, strings.TrimPrefix(a.what, "tile ")+" ") {
				continue
			}
			if a.r.overlaps(b.r) {
				t.Errorf("%s %+v overlaps %s %+v", a.what, a.r, b.what, b.r)
			}
		}
	}
}

// The standing robot, and every preview frame, fit a tile's picture with
// both cameras, and the tiles' camera is steeper than the chase camera.
func TestTileCamera(t *testing.T) {
	f := testFonts(t)
	l := newLayout(1280, 720, f)
	s := &screen{f: f, l: l}
	pic := s.overlayGeom().tiles[0].picture
	stand := framePose(standingFrame())
	for _, c := range []struct {
		name string
		cam  camera
	}{{"chase", chase}, {"tile", tileCam}} {
		vp := fitViewport(c.cam, previewBounds(c.cam), pic.x, pic.y, pic.w, pic.h)
		poses := []app.Pose{stand}
		for _, g := range gestures.Table {
			for _, fr := range g.Frames {
				poses = append(poses, framePose(fr))
			}
		}
		for _, p := range poses {
			for leg, lg := range p.Legs {
				for _, q := range lg.Points {
					x, y, _ := vp.pt(q)
					if x < pic.x || x > pic.x+pic.w || y < pic.y || y > pic.y+pic.h {
						t.Fatalf("%s camera: leg %d point %v at (%d, %d), outside the picture %+v", c.name, leg, q, x, y, pic)
					}
				}
			}
		}
	}
	if tileCam.fwd.Z >= chase.fwd.Z || math.Abs(tileCam.fwd.X) > 1e-9 {
		t.Errorf("tile camera looks along %v, the chase camera %v: want it steeper, straight ahead", tileCam.fwd, chase.fwd)
	}
	// The standing robot fills a good part of the picture with the tiles'
	// camera.
	vp := fitViewport(tileCam, tileBounds, pic.x, pic.y, pic.w, pic.h)
	minX, maxX := int32(math.MaxInt32), int32(math.MinInt32)
	for _, lg := range stand.Legs {
		for _, q := range lg.Points {
			x, _, _ := vp.pt(q)
			minX, maxX = min(minX, x), max(maxX, x)
		}
	}
	if maxX-minX < pic.w/2 && maxX-minX < pic.h/2 {
		t.Errorf("standing robot %d wide in a picture %d×%d", maxX-minX, pic.w, pic.h)
	}
}

func TestDrawTiming(t *testing.T) {
	if newDrawTiming(func(string) string { return "" }, nil) != nil {
		t.Fatal("timing without SPOODER_DRAW_TIMING")
	}
	var nilTiming *drawTiming
	nilTiming.controlsSet(time.Now()) // must not panic
	nilTiming.drew(time.Millisecond)

	var out strings.Builder
	tm := newDrawTiming(func(k string) string {
		if k == drawTimingEnv {
			return "1"
		}
		return ""
	}, &out)
	t0 := time.Unix(10, 0)
	tm.controlsSet(t0)
	tm.controlsSet(t0.Add(5 * time.Millisecond))
	tm.drew(12 * time.Millisecond)
	tm.controlsSet(t0.Add(22 * time.Millisecond)) // a 17 ms gap
	tm.drew(3 * time.Millisecond)
	tm.controlsSet(t0.Add(27 * time.Millisecond))
	if out.Len() != 0 {
		t.Fatalf("reported before the second ended: %q", out.String())
	}
	tm.controlsSet(t0.Add(time.Second))
	want := "spooder-handheld: draw timing: 2 draws, longest 12ms; 4 samplings, longest gap 973ms\n"
	if out.String() != want {
		t.Fatalf("report %q, want %q", out.String(), want)
	}
}

// The overlay is drawn over either page while L2 is held and not once it is
// released; the tiles of the gestures without a script, and only those, show
// NOT YET (read from the rendered pixels: the chip's face at the start of the
// status row, where a scripted tile shows its bare well).
func TestOverlayDrawn(t *testing.T) {
	f := testFonts(t)
	surf, err := sdl.CreateRGBSurfaceWithFormat(0, 1280, 720, 32, sdl.PIXELFORMAT_ARGB8888)
	if err != nil {
		t.Fatal(err)
	}
	defer surf.Free()
	r, err := sdl.CreateSoftwareRenderer(surf)
	if err != nil {
		t.Fatal(err)
	}
	defer func() { _ = r.Destroy() }()
	now := time.Unix(1000, 0)
	s := &screen{win: surfaceWindow{1280, 720}, r: r, f: f, l: newLayout(1280, 720, f), now: func() time.Time { return now }}
	pixel := func(x, y int32) uint32 {
		_ = r.Flush()
		return *(*uint32)(unsafe.Pointer(&surf.Pixels()[y*surf.Pitch+x*4]))
	}
	rgb := func(c sdl.Color) uint32 { return 0xff000000 | uint32(c.R)<<16 | uint32(c.G)<<8 | uint32(c.B) }
	g := s.overlayGeom()
	at := func(tg tileGeom) (int32, int32) { return tg.status.x + 3, tg.status.y + tg.status.h/2 }
	for _, driveView := range []bool{false, true} {
		sc := app.Screen{
			Robot:     app.Robot{Have: true, Body: "Stand", OutputsOn: true},
			Requested: app.Requested{Mode: "Stand"},
			State:     app.Driving,
			DriveView: driveView,
		}
		// The page alone, then with L2 held, then released again.
		s.draw(sc)
		page := make([]uint32, len(g.tiles))
		for i, tg := range g.tiles {
			page[i] = pixel(at(tg))
		}
		sc.Held = padmap.HandL2
		s.draw(sc)
		for i, tg := range g.tiles {
			tile := gestureTiles[i]
			g, _ := preview(tile.gesture)
			want := rgb(colWell)
			if !g.Scripted {
				want = rgb(colChip)
			}
			if got := pixel(at(tg)); got != want {
				t.Errorf("drive view %v, L2 held: tile %s status row %08x, want %08x (scripted %v)", driveView, tile.key, got, want, g.Scripted)
			}
		}
		sc.Held = 0
		now = now.Add(time.Second)
		s.draw(sc)
		for i, tg := range g.tiles {
			if got := pixel(at(tg)); got != page[i] {
				t.Errorf("drive view %v, L2 released: tile %s's place %08x, the page alone %08x", driveView, gestureTiles[i].key, got, page[i])
			}
		}
	}
}
