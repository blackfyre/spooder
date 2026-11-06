package main

import (
	"fmt"
	"math"
	"sort"
	"strconv"

	"github.com/veandco/go-sdl2/gfx"
	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/legview"
)

// The driving-view page (Select + X): one panel across the body area, the
// robot seen from a chase camera behind it, looking forward and 45° down, in
// perspective, in a well on its left: a ground grid at the feet, the
// schematic plate, the legs (the coxa thick, femur and tibia thinner, joints
// as discs, all thinner with distance), joints near or at their servo limits
// ringed, the front marker and the leg numbers, as the calibrator's views
// draw them (internal/ui/views.go). The camera is fixed to the body: with no
// odometry, the grid does not move as the robot walks. Beside it the robot's
// body mode, the gesture it plays, gait, output and supply, and its faults.
const (
	viewPad  = 18 // the drawing's margin inside its well
	viewCols = 2  // the cells beside the view

	chasePitchDeg = 45.0  // how far below the horizon the camera looks
	chaseDistance = 800.0 // mm from the camera to its target
	gridStep      = 100.0 // mm between the ground grid's lines
)

// chaseTarget is the point the camera looks at: the body's centre, a little
// below the pivots, near a standing robot's middle.
var chaseTarget = legview.Point3{Z: -80}

// camera is a perspective camera: its position, its orthonormal axes and its
// distance from its target, where the screen plane is.
type camera struct {
	pos, right, up, fwd legview.Point3
	dist                float64
}

// chase is the driving view's camera: behind the robot, looking forward and
// down at chasePitchDeg, chaseDistance from chaseTarget.
var chase = newChase(chasePitchDeg, chaseDistance)

func newChase(pitchDeg, dist float64) camera {
	a := pitchDeg * math.Pi / 180
	fwd := legview.Point3{Y: math.Cos(a), Z: -math.Sin(a)}
	return camera{
		pos:   legview.Point3{X: chaseTarget.X - fwd.X*dist, Y: chaseTarget.Y - fwd.Y*dist, Z: chaseTarget.Z - fwd.Z*dist},
		right: legview.Point3{X: 1},
		up:    legview.Point3{Y: math.Sin(a), Z: math.Cos(a)},
		fwd:   fwd,
		dist:  dist,
	}
}

func dot3(a, b legview.Point3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// project maps a body point to the screen plane (millimetres at the target's
// distance, x right, y up) and its depth along the view (larger is farther).
// Every point within the legs' reach is in front of the camera.
func (c camera) project(p legview.Point3) (legview.Vec2, float64) {
	v := legview.Point3{X: p.X - c.pos.X, Y: p.Y - c.pos.Y, Z: p.Z - c.pos.Z}
	d := dot3(v, c.fwd)
	k := c.dist / d
	return legview.Vec2{X: dot3(v, c.right) * k, Y: dot3(v, c.up) * k}, d
}

// box is the box spanned by the mounts with the legs reaching out horizon
// across and from z0 to z1.
func box(out, z0, z1 float64) (lo, hi legview.Point3) {
	lo = legview.Point3{X: math.Inf(1), Y: math.Inf(1), Z: z0}
	hi = legview.Point3{X: math.Inf(-1), Y: math.Inf(-1), Z: z1}
	for _, m := range legview.Mounts {
		lo.X, hi.X = math.Min(lo.X, m.Pivot.X-out), math.Max(hi.X, m.Pivot.X+out)
		lo.Y, hi.Y = math.Min(lo.Y, m.Pivot.Y-out), math.Max(hi.Y, m.Pivot.Y+out)
	}
	return lo, hi
}

// reachBox is the box every drawn point stays in: the legs' full reach
// across, from a foot at full reach below the pivots to a femur pointing
// straight up. The ground grid spans it, and every point in it is in front
// of the camera.
func reachBox() (lo, hi legview.Point3) {
	reach := legview.CoxaLength + legview.FemurLength + legview.TibiaLength
	return box(reach, -reach, legview.CoxaLength+legview.FemurLength)
}

// frameBox is what the view is framed on: the walking envelope, the feet at
// most half the tibia beyond the knee's full reach and from the tibia and
// half the femur below the pivots to the femur's length above. A pose
// beyond it is clipped by the well.
func frameBox() (lo, hi legview.Point3) {
	return box(legview.CoxaLength+legview.FemurLength+legview.TibiaLength/2,
		-(legview.TibiaLength + legview.FemurLength/2), legview.FemurLength)
}

// boxBounds are a box's screen-plane extents through c (minX, minY, maxX,
// maxY): its corners projected. A box's projection lies within its corners'
// hull.
func boxBounds(c camera, lo, hi legview.Point3) [4]float64 {
	b := [4]float64{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	for _, x := range []float64{lo.X, hi.X} {
		for _, y := range []float64{lo.Y, hi.Y} {
			for _, z := range []float64{lo.Z, hi.Z} {
				q, _ := c.project(legview.Point3{X: x, Y: y, Z: z})
				b[0], b[1] = math.Min(b[0], q.X), math.Min(b[1], q.Y)
				b[2], b[3] = math.Max(b[2], q.X), math.Max(b[3], q.Y)
			}
		}
	}
	return b
}

// chaseBounds are the driving view's fixed screen-plane extents: the frame
// box through the chase camera, so the drawing does not jump as the pose
// changes.
func chaseBounds() [4]float64 {
	lo, hi := frameBox()
	return boxBounds(chase, lo, hi)
}

// viewport maps a camera's screen plane to pixels.
type viewport struct {
	cam        camera
	scale      float64
	minX, maxY float64
	x, y       float64 // the drawing's top left
}

// fitViewport fits a camera's screen-plane bounds b into w × h at x, y,
// centred.
func fitViewport(cam camera, b [4]float64, x, y, w, h int32) viewport {
	bw, bh := b[2]-b[0], b[3]-b[1]
	s := math.Min(float64(w)/bw, float64(h)/bh)
	return viewport{cam: cam, scale: s, minX: b[0], maxY: b[3],
		x: float64(x) + (float64(w)-bw*s)/2, y: float64(y) + (float64(h)-bh*s)/2}
}

// pt is a body point's pixel, and its perspective factor: how much larger
// than at the target's distance it draws.
func (v viewport) pt(p legview.Point3) (int32, int32, float64) {
	q, d := v.cam.project(p)
	return int32(v.x + (q.X-v.minX)*v.scale + 0.5), int32(v.y + (v.maxY-q.Y)*v.scale + 0.5), v.cam.dist / d
}

// depth is the depth of the middle of points.
func (v viewport) depth(pts ...legview.Point3) float64 {
	var m legview.Point3
	for _, p := range pts {
		m.X, m.Y, m.Z = m.X+p.X, m.Y+p.Y, m.Z+p.Z
	}
	n := float64(len(pts))
	_, d := v.cam.project(legview.Point3{X: m.X / n, Y: m.Y / n, Z: m.Z / n})
	return d
}

// palette is the tone the legs draw in.
type palette int

const (
	paletteNormal palette = iota // the text tone; muted while output is off or the pose stale
	paletteLit                   // the orange
	paletteMuted                 // muted
)

// chaseStyle is how drawChase draws: through which camera, framed on which
// bounds, its line widths' scale, which details, and in which tone.
type chaseStyle struct {
	cam     camera
	bounds  [4]float64 // the screen-plane extents fitted into the rectangle
	lines   float64    // line widths' scale
	grid    bool       // the ground grid
	numbers bool       // the leg numbers
	limits  bool       // the limit rings
	palette palette
}

// pageStyle is the driving view's: the chase camera on the frame box, every
// detail.
func pageStyle() chaseStyle {
	return chaseStyle{cam: chase, bounds: chaseBounds(), lines: 1, grid: true, numbers: true, limits: true}
}

// driveViewWellW is the well's width for a drawing height h: the view's
// aspect, plus its margins.
func (l layout) driveViewWellW(h int32) int32 {
	b := chaseBounds()
	inner := h - 2*l.gap(viewPad)
	return int32(float64(inner)*(b[2]-b[0])/(b[3]-b[1])+0.5) + 2*l.gap(viewPad)
}

func (s *screen) driveView(sc app.Screen) {
	l := s.l
	x, w := l.left, l.w-2*l.gap(margin)
	h := l.bodyBottom - l.bodyY
	title, tc := "DRIVING VIEW", colMuted
	switch {
	case !sc.Pose.Have:
		title = "DRIVING VIEW · NO POSE"
	case sc.Pose.Stale:
		title, tc = "DRIVING VIEW · STALE", colWarn
	}
	y := s.panel(x, l.bodyY, w, h, "01", title, tc)
	ix := x + l.gap(panelPad)
	wellH := l.bodyBottom - l.gap(panelPad) - y
	wellW := l.driveViewWellW(wellH)
	s.fill(ix, y, wellW, wellH, l.gap(wellRadius), colWell)
	if sc.Pose.Have {
		_ = s.r.SetClipRect(&sdl.Rect{X: ix, Y: y, W: wellW, H: wellH})
		s.drawChase(ix+l.gap(viewPad), y+l.gap(viewPad), wellW-2*l.gap(viewPad), wellH-2*l.gap(viewPad), sc.Pose, pageStyle())
		_ = s.r.SetClipRect(nil)
	} else {
		msg := "No pose"
		tw := textWidth(s.f.value, msg)
		s.text(s.f.value, msg, ix+(wellW-tw)/2, y+(wellH-l.valueH)/2, wellW, colMuted)
	}
	s.tracked(s.f.hint, "CHASE CAMERA · 45° DOWN · SCHEMATIC BODY", ix+l.gap(12), y+wellH-l.gap(10)-int32(s.f.hint.Height()), colMuted, trackMicro)

	// Beside the view: the robot's state, then its faults.
	cx := ix + wellW + l.gap(28)
	cw := x + w - l.gap(panelPad) - cx
	rb := sc.Robot
	val := colText
	if rb.Stale {
		val = colMuted
	}
	cs := driveViewCells(rb, val)
	s.cells(cs, viewCols, cx, y, cw, 0)
	fy := y + int32((len(cs)+viewCols-1)/viewCols)*(l.cellH()+l.gap(cellGap))
	s.tracked(s.f.micro, "FAULTS", cx, fy, colMuted, trackMicro)
	fy += l.microH + l.gap(labelGap) + l.gap(2)
	s.faultChips(rb, cx, fy, cw, l.bodyBottom-l.gap(panelPad), val)
}

// driveViewCells is the robot's state beside the driving view: its body
// mode and the gesture it plays (design D1 of handheld-observations), gait,
// output and supply.
func driveViewCells(rb app.Robot, val sdl.Color) []cell {
	body, gesture, gait, out, supply := "No telemetry", "—", "—", "—", "—"
	if rb.Have {
		body, gesture, gait, out = rb.Body, "None", rb.Gait, "Off"
		if rb.Gesture != "" {
			gesture = rb.Gesture
		}
		if rb.OutputsOn {
			out = "On"
			if rb.Simulated {
				out = "On (sim)"
			}
		}
		supply = fmt.Sprintf("%.2f V", float64(rb.VoltageMv)/1000)
	}
	return []cell{
		{"BODY", body, val, false, nil},
		{"GESTURE", gesture, val, false, nil},
		{"GAIT", gait, val, false, nil},
		{"OUTPUT", out, val, false, nil},
		{"SUPPLY", supply, val, true, nil},
	}
}

// piece is one thing the driving view draws, at its depth: the painter's
// algorithm draws the farthest first.
type piece struct {
	depth float64
	draw  func()
}

// drawChase draws the robot into w × h at x, y as st says: the ground grid,
// then the plate's faces and the legs' segments far to near, then the limit
// rings and the leg numbers over them. In the normal palette the legs are in
// the text tone while output is on and the pose fresh, muted otherwise.
func (s *screen) drawChase(x, y, w, h int32, p app.Pose, st chaseStyle) {
	l := s.l
	vp := fitViewport(st.cam, st.bounds, x, y, w, h)
	c := colText
	switch {
	case st.palette == paletteLit:
		c = colAccentTxt
	case st.palette == paletteMuted, !p.Pose.OutputsOn || p.Stale:
		c = colMuted
	}
	// A width in pixels at the target's distance, scaled by perspective and
	// the style.
	width := func(w int, k float64) int32 { return max(1, int32(float64(l.gap(w))*k*st.lines+0.5)) }
	poly := func(pts []legview.Point3, fill, edge sdl.Color) {
		vx, vy := make([]int16, len(pts)), make([]int16, len(pts))
		for i, q := range pts {
			x, y, _ := vp.pt(q)
			vx[i], vy[i] = int16(x), int16(y)
		}
		gfx.FilledPolygonColor(s.r, vx, vy, fill)
		gfx.AAPolygonColor(s.r, vx, vy, edge)
	}

	if ground, ok := legview.Ground(p.Legs); ok && st.grid {
		s.drawGrid(vp, ground)
	}

	var pieces []piece
	top, bottom := legview.Plate()
	for i := range top {
		j := (i + 1) % len(top)
		side := []legview.Point3{top[i], top[j], bottom[j], bottom[i]}
		pieces = append(pieces, piece{vp.depth(side...), func() { poly(side, colChip, colKey) }})
	}
	pieces = append(pieces, piece{vp.depth(top...), func() {
		poly(top, colKey, colMuted)
		// The front marker, a chevron on the plate.
		z := legview.PlateThickness / 2
		ax, ay, k := vp.pt(legview.Point3{X: -22, Y: legview.FrontMarkTip - 26, Z: z})
		tx, ty, _ := vp.pt(legview.Point3{Y: legview.FrontMarkTip, Z: z})
		bx, by, _ := vp.pt(legview.Point3{X: 22, Y: legview.FrontMarkTip - 26, Z: z})
		gfx.ThickLineColor(s.r, ax, ay, tx, ty, width(3, k), colAccentTxt)
		gfx.ThickLineColor(s.r, tx, ty, bx, by, width(3, k), colAccentTxt)
	}})
	for _, leg := range p.Legs {
		if !leg.Set {
			q := leg.Points[0]
			pieces = append(pieces, piece{vp.depth(q), func() {
				x, y, k := vp.pt(q)
				s.disc(x, y, width(4, k), colMuted)
			}})
			continue
		}
		for i, segW := range [3]int{7, 4, 4} {
			a, b, foot := leg.Points[i], leg.Points[i+1], i == 2
			pieces = append(pieces, piece{vp.depth(a, b), func() {
				ax, ay, ka := vp.pt(a)
				bx, by, kb := vp.pt(b)
				gfx.ThickLineColor(s.r, ax, ay, bx, by, width(segW, (ka+kb)/2), c)
				s.disc(ax, ay, width(4, ka), c)
				r := 4
				if foot {
					r = 7
				}
				s.disc(bx, by, width(r, kb), c)
			}})
		}
	}
	sort.SliceStable(pieces, func(i, j int) bool { return pieces[i].depth > pieces[j].depth })
	for _, pc := range pieces {
		pc.draw()
	}

	for leg, lg := range p.Legs {
		if !lg.Set || !st.limits {
			continue
		}
		for j := 0; j < 3; j++ {
			x, y, _ := vp.pt(lg.Points[j])
			s.limitRing(x, y, p.Pose.Legs[leg].Joints[j].Limit)
		}
	}

	// Each leg's number beyond its foot, outward from its pivot on the
	// screen (beside the pivot when never set, outward from the body's
	// centre), on a backing so the drawing under it does not hide it.
	if !st.numbers {
		return
	}
	ox, oy, _ := vp.pt(legview.Point3{})
	for leg, lg := range p.Legs {
		px, py, _ := vp.pt(lg.Points[0])
		x, y := px, py
		dx, dy := float64(x-ox), float64(y-oy)
		if lg.Set {
			x, y, _ = vp.pt(lg.Points[3])
			dx, dy = float64(x-px), float64(y-py)
		}
		if d := math.Hypot(dx, dy); d > 0 {
			x += int32(dx / d * float64(l.gap(26)))
			y += int32(dy / d * float64(l.gap(26)))
		}
		n := strconv.Itoa(calib.Leg(leg * legview.JointsPerLeg))
		tw, th := textWidth(s.f.micro, n), int32(s.f.micro.Height())
		s.fill(x-tw/2-l.gap(5), y-th/2, tw+2*l.gap(5), th, l.gap(4), colWell)
		s.text(s.f.micro, n, x-tw/2, y-th/2, tw+1, colText)
	}
}

// drawGrid draws the ground under the robot at height z: lines gridStep
// apart across the reach box, converging with distance.
func (s *screen) drawGrid(vp viewport, z float64) {
	lo, hi := reachBox()
	line := func(a, b legview.Point3) {
		ax, ay, _ := vp.pt(a)
		bx, by, _ := vp.pt(b)
		gfx.AALineColor(s.r, ax, ay, bx, by, colPanel)
	}
	for x := math.Ceil(lo.X/gridStep) * gridStep; x <= hi.X; x += gridStep {
		line(legview.Point3{X: x, Y: lo.Y, Z: z}, legview.Point3{X: x, Y: hi.Y, Z: z})
	}
	for y := math.Ceil(lo.Y/gridStep) * gridStep; y <= hi.Y; y += gridStep {
		line(legview.Point3{X: lo.X, Y: y, Z: z}, legview.Point3{X: hi.X, Y: y, Z: z})
	}
}

// disc is a filled, anti-aliased circle.
func (s *screen) disc(x, y, r int32, c sdl.Color) {
	gfx.FilledCircleColor(s.r, x, y, r, c)
	gfx.AACircleColor(s.r, x, y, r, c)
}

// ring is a circle stroked width wide inside radius r.
func (s *screen) ring(x, y, r, width int32, c sdl.Color) {
	for i := int32(0); i < width; i++ {
		gfx.AACircleColor(s.r, x, y, r-i, c)
	}
}

// limitRing rings a joint near its servo limit in the warning tone, and one
// at it (its pulse clamped) thicker, with an orange outer ring, as the
// calibrator does.
func (s *screen) limitRing(x, y int32, lim legview.Limit) {
	l := s.l
	switch lim {
	case legview.LimitNear:
		s.ring(x, y, l.gap(13), l.gap(3), colWarn)
	case legview.LimitAt:
		s.ring(x, y, l.gap(13), l.gap(4), colWarn)
		s.ring(x, y, l.gap(17), l.gap(2), colAccentTxt)
	}
}
