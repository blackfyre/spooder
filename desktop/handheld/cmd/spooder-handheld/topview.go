package main

import (
	"fmt"
	"math"
	"strconv"

	"github.com/veandco/go-sdl2/gfx"
	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/legview"
)

// The top-view page (Select + X): one panel across the body area, the robot
// from above in a well on its left, front up, drawn as the calibrator's top
// view draws it (internal/ui/views.go): the schematic plate under the legs,
// the coxa thick, femur and tibia thinner, joints as discs, joints near or
// at their servo limits ringed, the front marker and the leg numbers. Beside
// it the robot's body mode, gait, output and supply, and its faults.
const (
	viewPad  = 18 // the drawing's margin inside its well
	viewCols = 2  // the cells beside the view
)

// viewport maps the top view's screen plane (millimetres, y up) to pixels.
type viewport struct {
	scale      float64
	minX, maxY float64
	x, y       float64 // the drawing's top left
}

// topViewport fits the top view's fixed bounds into w × h at x, y, centred.
func topViewport(x, y, w, h int32) viewport {
	b := legview.ViewBounds(legview.ViewTop)
	bw, bh := b[2]-b[0], b[3]-b[1]
	s := math.Min(float64(w)/bw, float64(h)/bh)
	return viewport{scale: s, minX: b[0], maxY: b[3],
		x: float64(x) + (float64(w)-bw*s)/2, y: float64(y) + (float64(h)-bh*s)/2}
}

func (v viewport) pt(p legview.Point3) (int32, int32) {
	q, _ := legview.Project(legview.ViewTop, p)
	return int32(v.x + (q.X-v.minX)*v.scale + 0.5), int32(v.y + (v.maxY-q.Y)*v.scale + 0.5)
}

// topViewWellW is the well's width for a drawing height h: the view's
// aspect, plus its margins.
func (l layout) topViewWellW(h int32) int32 {
	b := legview.ViewBounds(legview.ViewTop)
	inner := h - 2*l.gap(viewPad)
	return int32(float64(inner)*(b[2]-b[0])/(b[3]-b[1])+0.5) + 2*l.gap(viewPad)
}

func (s *screen) topView(sc app.Screen) {
	l := s.l
	x, w := l.left, l.w-2*l.gap(margin)
	h := l.bodyBottom - l.bodyY
	title, tc := "TOP VIEW", colMuted
	switch {
	case !sc.Pose.Have:
		title = "TOP VIEW · NO POSE"
	case sc.Pose.Stale:
		title, tc = "TOP VIEW · STALE", colWarn
	}
	y := s.panel(x, l.bodyY, w, h, "01", title, tc)
	ix := x + l.gap(panelPad)
	wellH := l.bodyBottom - l.gap(panelPad) - y
	wellW := l.topViewWellW(wellH)
	s.fill(ix, y, wellW, wellH, l.gap(wellRadius), colWell)
	if sc.Pose.Have {
		vp := topViewport(ix+l.gap(viewPad), y+l.gap(viewPad), wellW-2*l.gap(viewPad), wellH-2*l.gap(viewPad))
		s.drawTop(vp, sc.Pose)
	} else {
		msg := "No pose"
		tw := textWidth(s.f.value, msg)
		s.text(s.f.value, msg, ix+(wellW-tw)/2, y+(wellH-l.valueH)/2, wellW, colMuted)
	}
	s.tracked(s.f.hint, "FRONT ↑ · SCHEMATIC BODY", ix+l.gap(12), y+wellH-l.gap(10)-int32(s.f.hint.Height()), colMuted, trackMicro)

	// Beside the view: the robot's state, then its faults.
	cx := ix + wellW + l.gap(28)
	cw := x + w - l.gap(panelPad) - cx
	rb := sc.Robot
	val := colText
	if rb.Stale {
		val = colMuted
	}
	cs := topViewCells(rb, val)
	s.cells(cs, viewCols, cx, y, cw)
	fy := y + int32((len(cs)+viewCols-1)/viewCols)*(l.cellH()+l.gap(cellGap))
	s.tracked(s.f.micro, "FAULTS", cx, fy, colMuted, trackMicro)
	fy += l.microH + l.gap(labelGap) + l.gap(2)
	s.faultChips(rb, cx, fy, cw, l.bodyBottom-l.gap(panelPad), val)
}

// topViewCells is the robot's state beside the top view.
func topViewCells(rb app.Robot, val sdl.Color) []cell {
	body, gait, out, supply := "No telemetry", "—", "—", "—"
	if rb.Have {
		body, gait, out = rb.Body, rb.Gait, "Off"
		if rb.OutputsOn {
			out = "On"
		}
		supply = fmt.Sprintf("%.2f V", float64(rb.VoltageMv)/1000)
	}
	return []cell{
		{"BODY", body, val, false, nil},
		{"GAIT", gait, val, false, nil},
		{"OUTPUT", out, val, false, nil},
		{"SUPPLY", supply, val, true, nil},
	}
}

// drawTop draws the robot from above: the plate, the legs over it, the
// front marker, the limit rings and the leg numbers. Legs are in the text
// tone while output is on and the pose fresh, muted otherwise.
func (s *screen) drawTop(vp viewport, p app.Pose) {
	l := s.l
	c := colText
	if !p.Pose.OutputsOn || p.Stale {
		c = colMuted
	}

	top, _ := legview.Plate()
	vx, vy := make([]int16, len(top)), make([]int16, len(top))
	for i, q := range top {
		x, y := vp.pt(q)
		vx[i], vy[i] = int16(x), int16(y)
	}
	gfx.FilledPolygonColor(s.r, vx, vy, colKey)
	gfx.AAPolygonColor(s.r, vx, vy, colMuted)

	for _, leg := range p.Legs {
		var xs, ys [4]int32
		for i, q := range leg.Points {
			xs[i], ys[i] = vp.pt(q)
		}
		if !leg.Set {
			s.disc(xs[0], ys[0], l.gap(4), colMuted)
			continue
		}
		gfx.ThickLineColor(s.r, xs[0], ys[0], xs[1], ys[1], l.gap(7), c)
		gfx.ThickLineColor(s.r, xs[1], ys[1], xs[2], ys[2], l.gap(4), c)
		gfx.ThickLineColor(s.r, xs[2], ys[2], xs[3], ys[3], l.gap(4), c)
		for j := 0; j < 3; j++ {
			s.disc(xs[j], ys[j], l.gap(4), c)
		}
		s.disc(xs[3], ys[3], l.gap(7), c)
	}

	// The front marker, a chevron on the plate.
	z := legview.PlateThickness / 2
	ax, ay := vp.pt(legview.Point3{X: -22, Y: legview.FrontMarkTip - 26, Z: z})
	tx, ty := vp.pt(legview.Point3{Y: legview.FrontMarkTip, Z: z})
	bx, by := vp.pt(legview.Point3{X: 22, Y: legview.FrontMarkTip - 26, Z: z})
	gfx.ThickLineColor(s.r, ax, ay, tx, ty, l.gap(3), colAccentTxt)
	gfx.ThickLineColor(s.r, tx, ty, bx, by, l.gap(3), colAccentTxt)

	for leg, lg := range p.Legs {
		if !lg.Set {
			continue
		}
		for j := 0; j < 3; j++ {
			x, y := vp.pt(lg.Points[j])
			s.limitRing(x, y, p.Pose.Legs[leg].Joints[j].Limit)
		}
	}

	// Each leg's number beyond its foot, outward from its pivot (beside the
	// pivot when never set, outward from the body's centre), on a backing so
	// the drawing under it does not hide it.
	for leg, lg := range p.Legs {
		px, py := vp.pt(lg.Points[0])
		x, y := px, py
		if lg.Set {
			x, y = vp.pt(lg.Points[3])
		}
		ox, oy := vp.pt(legview.Point3{})
		dx, dy := float64(x-ox), float64(y-oy)
		if lg.Set {
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
