package main

import (
	"math"
	"time"

	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/gestures"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/internal/legview"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// The gesture overlay (openspec change add-handheld-gesture-overlay, designs
// D3 to D7): while L2 is held beyond half travel, a panel over the page with
// one tile per gesture of the L2 layer, placed as the TrimUI's buttons are
// (A, B, X and Y as a diamond, R2 beside it). A scripted gesture's tile
// loops the robot playing it, from the robot's own script's joint angles
// (internal/gestures); one without a script shows the standing robot muted,
// marked NOT YET. A request lights its tile for a moment; the gesture fresh
// telemetry reports playing stays lit. STAND FIRST shows while the robot
// cannot start a gesture. The overlay only reads the screen model: it
// changes nothing that is sent.
const (
	// requestLit is how long a gesture request lights its tile.
	requestLit = 400 * time.Millisecond
	// loopHold is how long a preview holds its last frame before it starts
	// again.
	loopHold = 600 * time.Millisecond

	tileGap    = 16 // between tiles
	tileSplit  = 32 // between the diamond and R2
	tilePad    = 12 // inside a tile
	tileRowGap = 6  // between a tile's rows and its picture
	tileChipH  = 30 // a tile's status chips
	tileChipX  = 10 // their side padding
	tileRing   = 3  // a lit tile's outline

	tilePitchDeg = 60.0 // the tiles' camera: higher and steeper than the chase camera
	tileLines    = 0.7  // the tiles' line widths, of the page's
)

// tileCam is the tiles' camera: behind the robot as the chase camera is, but
// steeper, so that the six legs stay apart at thumbnail size (design D6).
var tileCam = newChase(tilePitchDeg, chaseDistance)

// gestureTile is one tile: the button that requests its gesture with L2
// held, its keycap, its name and its place in the overlay's grid (columns
// 0 to 3, rows 0 and 1 in halves: the diamond's middle row sits between
// them).
type gestureTile struct {
	b        padmap.Buttons
	gesture  uint8
	key      string
	name     string
	col      int
	halfRows int // the tile's top, in half rows
}

// gestureTiles follow the TrimUI Smart Pro's face buttons (Nintendo layout:
// X top, Y left, A right, B bottom), R2 to the right of the diamond, level
// with X (design D3).
var gestureTiles = []gestureTile{
	{padmap.HandA, protocol.GestureSayHi, "A", "SAY HI", 2, 1},
	{padmap.HandB, protocol.GestureWiggle, "B", "WIGGLE", 1, 2},
	{padmap.HandX, protocol.GestureDance, "X", "DANCE", 1, 0},
	{padmap.HandY, protocol.GestureAttack, "Y", "ATTACK", 0, 1},
	{padmap.HandR2, protocol.GestureBall, "R2", "BALL", 3, 0},
}

// preview is a gesture id's preview from the table, if there is one.
func preview(id uint8) (gestures.Gesture, bool) {
	for _, g := range gestures.Table {
		if g.ID == id {
			return g, true
		}
	}
	return gestures.Gesture{}, false
}

// standingFrame is the standing robot: every preview starts from it (the
// player starts at the standing point), so it is any script's first frame.
func standingFrame() [gestures.Joints]float32 {
	for _, g := range gestures.Table {
		if g.Scripted && len(g.Frames) > 0 {
			return g.Frames[0]
		}
	}
	// No script yet: the standing point's solution (stand_height_0 in
	// test/golden/leg_pose.txt).
	var f [gestures.Joints]float32
	for leg := 0; leg < legview.Legs; leg++ {
		f[leg*3+1], f[leg*3+2] = 59.16, 119.87
	}
	return f
}

// frameAt is the frame of g shown elapsed after the overlay opened (design
// D5): frame ⌊elapsed / StepMs⌋, then the last frame held for loopHold, then
// again from frame 0.
func frameAt(g gestures.Gesture, elapsed time.Duration) int {
	n := len(g.Frames)
	if n <= 1 || elapsed < 0 {
		return 0
	}
	step := time.Duration(g.StepMs) * time.Millisecond
	run := time.Duration(n-1) * step
	t := elapsed % (run + loopHold)
	if t >= run {
		return n - 1
	}
	return int(t / step)
}

// framePose is the robot in a preview frame, every leg set, output on.
func framePose(f [gestures.Joints]float32) app.Pose {
	p := legview.Pose{OutputsOn: true}
	for leg := range p.Legs {
		j := leg * legview.JointsPerLeg
		p.Legs[leg] = legview.Leg{Set: true, Angles: legview.Angles{
			Coxa: float64(f[j+legview.Coxa]), Femur: float64(f[j+legview.Femur]), Tibia: float64(f[j+legview.Tibia])}}
	}
	return app.Pose{Have: true, Pose: p, Legs: legview.Body3(p)}
}

// previewBounds are c's screen-plane extents of every preview frame (and
// the standing robot), the plate included, so that a tile's drawing neither
// jumps nor leaves its picture while it loops.
func previewBounds(c camera) [4]float64 {
	b := [4]float64{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	add := func(p legview.Point3) {
		q, _ := c.project(p)
		b[0], b[1] = math.Min(b[0], q.X), math.Min(b[1], q.Y)
		b[2], b[3] = math.Max(b[2], q.X), math.Max(b[3], q.Y)
	}
	top, bottom := legview.Plate()
	for _, p := range append(top, bottom...) {
		add(p)
	}
	addFrame := func(f [gestures.Joints]float32) {
		for _, leg := range framePose(f).Legs {
			for _, p := range leg.Points {
				add(p)
			}
		}
	}
	addFrame(standingFrame())
	for _, g := range gestures.Table {
		for _, f := range g.Frames {
			addFrame(f)
		}
	}
	// A margin for the joints' discs and the line widths.
	m := 0.04 * math.Max(b[2]-b[0], b[3]-b[1])
	return [4]float64{b[0] - m, b[1] - m, b[2] + m, b[3] + m}
}

// tileBounds frame every tile's picture.
var tileBounds = previewBounds(tileCam)

// tileStyle is a tile's drawing: the tiles' camera, thicker relative lines,
// no grid, leg numbers or limit rings.
func tileStyle(p palette) chaseStyle {
	return chaseStyle{cam: tileCam, bounds: tileBounds, lines: tileLines, palette: p}
}

// overlayState is what the overlay keeps between frames: whether it is open
// and since when (the animations run from then), the last request counter
// seen, and the tile a request lit.
type overlayState struct {
	open     bool
	openedAt time.Time
	haveSeq  bool
	seq      uint8
	litID    uint8
	litUntil time.Time
}

// update takes one frame's screen model at now: L2 held opens the overlay,
// and a change of the request counter lights the requested gesture's tile
// for requestLit (the first frame only records the counter).
func (o *overlayState) update(sc app.Screen, now time.Time) {
	switch {
	case !o.haveSeq:
		o.haveSeq, o.seq = true, sc.Requested.GestureSeq
	case sc.Requested.GestureSeq != o.seq:
		o.seq = sc.Requested.GestureSeq
		o.litID, o.litUntil = sc.Requested.Gesture, now.Add(requestLit)
	}
	open := sc.Held&padmap.HandL2 != 0
	if open && !o.open {
		o.openedAt = now
	}
	o.open = open
}

// tileState is one tile's state in a frame.
type tileState struct {
	frame     [gestures.Joints]float32
	scripted  bool
	requested bool // lit by a request
	playing   bool // lit: fresh telemetry reports it playing
}

func (t tileState) lit() bool { return t.requested || t.playing }

// tileStates are the tiles' states at now, in gestureTiles' order.
func (o *overlayState) tileStates(sc app.Screen, now time.Time) []tileState {
	out := make([]tileState, len(gestureTiles))
	fresh := sc.Robot.Have && !sc.Robot.Stale
	for i, t := range gestureTiles {
		st := &out[i]
		st.frame = standingFrame()
		if g, ok := preview(t.gesture); ok && g.Scripted && len(g.Frames) > 0 {
			st.scripted = true
			st.frame = g.Frames[frameAt(g, now.Sub(o.openedAt))]
		}
		st.requested = o.litID == t.gesture && now.Before(o.litUntil)
		st.playing = fresh && sc.Robot.GestureID == t.gesture
	}
	return out
}

// standFirst tells whether the robot cannot start a gesture: no fresh
// telemetry, or a body mode other than Stand or Perform (in Perform it is
// playing one).
func standFirst(rb app.Robot) bool {
	return !rb.Have || rb.Stale || rb.Body != "Stand" && rb.Body != "Perform"
}

// rect is a rectangle in pixels.
type rect struct{ x, y, w, h int32 }

func (r rect) inside(o rect) bool {
	return r.x >= o.x && r.y >= o.y && r.x+r.w <= o.x+o.w && r.y+r.h <= o.y+o.h
}

func (r rect) overlaps(o rect) bool {
	return r.x < o.x+o.w && o.x < r.x+r.w && r.y < o.y+o.h && o.y < r.y+r.h
}

// tileGeom is one tile's layout: its face, the keycap and the name in its
// top row, the picture, and the status chips' row.
type tileGeom struct {
	face, key, name, picture, status rect
}

// overlayGeom is the overlay's layout: the panel over the page's body, the
// tiles, and in the free place below R2 the title and the STAND FIRST chip.
type overlayGeom struct {
	panel      rect
	tiles      []tileGeom
	title      rect
	standFirst rect
}

const (
	labelStandFirst = "STAND FIRST"
	labelNotYet     = "NOT YET"
	labelPlaying    = "PLAYING"
	labelRequested  = "REQUESTED"
	overlayTitle    = "GESTURES"
	overlayKicker   = "L2"
)

// tileChipWidth is a tile's status chip's width.
func (s *screen) tileChipWidth(text string) int32 {
	return 2*s.l.gap(tileChipX) + trackedWidth(s.f.micro, text, trackLabel)
}

// overlayGeom lays the overlay out on the screen's layout.
func (s *screen) overlayGeom() overlayGeom {
	l := s.l
	g := overlayGeom{panel: rect{l.left, l.bodyY, l.w - 2*l.gap(margin), l.bodyBottom - l.bodyY}}
	ix, iy := g.panel.x+l.gap(panelPad), g.panel.y+l.gap(panelPad)
	iw, ih := g.panel.w-2*l.gap(panelPad), g.panel.h-2*l.gap(panelPad)
	tw := (iw - 2*l.gap(tileGap) - l.gap(tileSplit)) / 4
	th := (ih - l.gap(tileGap)) / 2
	colX := func(c int) int32 {
		x := ix + int32(c)*(tw+l.gap(tileGap))
		if c == 3 {
			x += l.gap(tileSplit) - l.gap(tileGap)
		}
		return x
	}
	keyH := l.gap(keyHeight) + l.gap(keyEdge)
	for _, t := range gestureTiles {
		f := rect{colX(t.col), iy + int32(t.halfRows)*(th+l.gap(tileGap))/2, tw, th}
		px, pw := f.x+l.gap(tilePad), f.w-2*l.gap(tilePad)
		key := rect{px, f.y + l.gap(tilePad), s.keycapWidth(t.key), keyH}
		nx := key.x + key.w + l.gap(12)
		name := rect{nx, key.y + (l.gap(keyHeight)-l.valueH)/2, textWidth(s.f.value, t.name), l.valueH}
		status := rect{px, f.y + f.h - l.gap(tilePad) - l.gap(tileChipH), pw, l.gap(tileChipH)}
		top := key.y + key.h + l.gap(tileRowGap)
		pic := rect{px, top, pw, status.y - l.gap(tileRowGap) - top}
		g.tiles = append(g.tiles, tileGeom{face: f, key: key, name: name, picture: pic, status: status})
	}
	// The free place below R2: the title, and the STAND FIRST chip under it.
	free := rect{colX(3), iy + th + l.gap(tileGap), tw, th}
	kw := trackedWidth(s.f.micro, overlayKicker+" · "+overlayTitle, trackKicker)
	g.title = rect{free.x + l.gap(tilePad), free.y + l.gap(tilePad), kw, l.microH}
	g.standFirst = rect{g.title.x, g.title.y + g.title.h + l.gap(kickerGap), s.chipWidth(labelStandFirst, false), l.gap(chipHeight)}
	return g
}

// tileChips are a tile's status chips, left to right: PLAYING or
// REQUESTED while lit, NOT YET without a script.
func tileChips(st tileState) []string {
	var out []string
	switch {
	case st.playing:
		out = append(out, labelPlaying)
	case st.requested:
		out = append(out, labelRequested)
	}
	if !st.scripted {
		out = append(out, labelNotYet)
	}
	return out
}

// scrim dims what is drawn so far.
func (s *screen) scrim() {
	var mode sdl.BlendMode = sdl.BLENDMODE_NONE
	_ = s.r.GetDrawBlendMode(&mode)
	_ = s.r.SetDrawBlendMode(sdl.BLENDMODE_BLEND)
	_ = s.r.SetDrawColor(0, 0, 0, 0x9a)
	_ = s.r.FillRect(&sdl.Rect{X: 0, Y: 0, W: s.l.w, H: s.l.h})
	_ = s.r.SetDrawBlendMode(mode)
}

// overlay draws the open overlay over the page at now.
func (s *screen) overlay(sc app.Screen, now time.Time) {
	l := s.l
	g := s.overlayGeom()
	s.scrim()
	p := g.panel
	s.fill(p.x-1, p.y-1, p.w+2, p.h+2, l.gap(panelRadius)+1, colOutline)
	s.fill(p.x, p.y, p.w, p.h, l.gap(panelRadius), colPanel)

	for i, st := range s.ov.tileStates(sc, now) {
		t, tg := gestureTiles[i], g.tiles[i]
		f := tg.face
		pal := paletteNormal
		switch {
		case st.lit():
			pal = paletteLit
			s.fill(f.x-l.gap(tileRing), f.y-l.gap(tileRing), f.w+2*l.gap(tileRing), f.h+2*l.gap(tileRing), l.gap(wellRadius)+l.gap(tileRing), colAccent)
		case !st.scripted:
			pal = paletteMuted
		}
		s.fill(f.x, f.y, f.w, f.h, l.gap(wellRadius), colWell)
		s.keycap(tg.key.x, tg.key.y, t.key, sc.Held&t.b != 0)
		nameCol := colText
		if !st.scripted {
			nameCol = colMuted
		}
		s.text(s.f.value, t.name, tg.name.x, tg.name.y, f.x+f.w-l.gap(tilePad)-tg.name.x, nameCol)
		pic := tg.picture
		_ = s.r.SetClipRect(&sdl.Rect{X: f.x, Y: pic.y, W: f.w, H: pic.h})
		s.drawChase(pic.x, pic.y, pic.w, pic.h, framePose(st.frame), tileStyle(pal))
		_ = s.r.SetClipRect(nil)
		cx := tg.status.x
		for _, c := range tileChips(st) {
			face, fg := colChip, colMuted
			if c != labelNotYet {
				face, fg = colAccent, colOnAccent
			}
			w := s.tileChipWidth(c)
			s.fill(cx, tg.status.y, w, tg.status.h, tg.status.h/2, face)
			s.tracked(s.f.micro, c, cx+l.gap(tileChipX), tg.status.y+(tg.status.h-l.microH)/2, fg, trackLabel)
			cx += w + l.gap(8)
		}
	}

	kx := g.title.x
	kx += s.tracked(s.f.micro, overlayKicker, kx, g.title.y, colAccentTxt, trackKicker)
	kx += s.tracked(s.f.micro, " · ", kx, g.title.y, colMuted, trackKicker)
	s.tracked(s.f.micro, overlayTitle, kx, g.title.y, colMuted, trackKicker)
	if standFirst(sc.Robot) {
		s.chip(g.standFirst.x, g.standFirst.y, labelStandFirst, colWarn, colOnStatus, nil)
	}
}
