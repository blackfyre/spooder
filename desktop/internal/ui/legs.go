package ui

import (
	"image"
	"image/color"
	"math"
	"strconv"

	"gioui.org/f32"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/op/clip"
	"gioui.org/op/paint"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/legview"
)

// World bounds of the drawings, in millimetres: the side view (outward,
// up) and the top view (body X right, Y forward).
var (
	sideBounds = [4]float64{-70, -250, 350, 170}
	topBounds  = [4]float64{-400, -400, 400, 400}
)

// viewport maps millimetres to pixels, keeping the aspect ratio, with y up.
type viewport struct {
	scale      float64
	minX, maxY float64
	off        f32.Point
}

func fit(bounds [4]float64, size image.Point) viewport {
	w, h := bounds[2]-bounds[0], bounds[3]-bounds[1]
	s := math.Min(float64(size.X)/w, float64(size.Y)/h)
	return viewport{scale: s, minX: bounds[0], maxY: bounds[3],
		off: f32.Pt(float32((float64(size.X)-w*s)/2), float32((float64(size.Y)-h*s)/2))}
}

func (v viewport) pt(p legview.Vec2) f32.Point {
	return f32.Pt(v.off.X+float32((p.X-v.minX)*v.scale), v.off.Y+float32((v.maxY-p.Y)*v.scale))
}

// strokePath draws a polyline.
func strokePath(gtx layout.Context, pts []f32.Point, width float32, c color.NRGBA) {
	if len(pts) < 2 {
		return
	}
	var p clip.Path
	p.Begin(gtx.Ops)
	p.MoveTo(pts[0])
	for _, q := range pts[1:] {
		p.LineTo(q)
	}
	paint.FillShape(gtx.Ops, c, clip.Stroke{Path: p.End(), Width: width}.Op())
}

func disc(gtx layout.Context, at f32.Point, r float32, c color.NRGBA) {
	rect := image.Rect(int(at.X-r), int(at.Y-r), int(at.X+r+0.5), int(at.Y+r+0.5))
	paint.FillShape(gtx.Ops, c, clip.Ellipse(rect).Op(gtx.Ops))
}

func ring(gtx layout.Context, at f32.Point, r, width float32, c color.NRGBA) {
	rect := image.Rect(int(at.X-r), int(at.Y-r), int(at.X+r+0.5), int(at.Y+r+0.5))
	paint.FillShape(gtx.Ops, c, clip.Stroke{Path: clip.Ellipse(rect).Path(gtx.Ops), Width: width}.Op())
}

// at lays a widget out with its top-left corner at p (unconstrained).
func at(gtx layout.Context, p image.Point, wd layout.Widget) layout.Dimensions {
	t := op.Offset(p).Push(gtx.Ops)
	gtx.Constraints.Min = image.Point{}
	d := wd(gtx)
	t.Pop()
	return d
}

// centredAt lays a widget out centred on p, kept inside size.
func centredAt(gtx layout.Context, p f32.Point, size image.Point, wd layout.Widget) {
	m := op.Record(gtx.Ops)
	g := gtx
	g.Constraints.Min = image.Point{}
	d := wd(g)
	call := m.Stop()
	x := int(p.X) - d.Size.X/2
	y := int(p.Y) - d.Size.Y/2
	x = max(0, min(x, size.X-d.Size.X))
	y = max(0, min(y, size.Y-d.Size.Y))
	t := op.Offset(image.Pt(x, y)).Push(gtx.Ops)
	call.Add(gtx.Ops)
	t.Pop()
}

// legsPane: the selected leg from the side, then its joints beside all six
// legs from above, from the latest pose.
func (w *window) legsPane(gtx layout.Context) layout.Dimensions {
	st := legsFor(w.snap, w.sel)
	if !st.Current {
		return panel(gtx, w.pal.Surface, w.pal.Border, 16, func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints.Min = gtx.Constraints.Max
			return stacked(gtx, 8,
				tracked(w.th, sizeTitle, w.pal.Muted, "no pose"),
				wrapped(textLabel(w.th, sizeText, w.pal.Muted, st.Reason)),
			)
		})
	}
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			return layout.Inset{Bottom: 6}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
				ws := []layout.Widget{tracked(w.th, sizeLabel, w.pal.Fg, legTitle(st.Leg))}
				if st.Pose.OutputsOn {
					ws = append(ws, w.chip("output on", w.pal.Green))
				} else {
					ws = append(ws, w.chip("output off", w.pal.Yellow))
				}
				l := st.Pose.Legs[st.Leg]
				if l.Set {
					ws = append(ws, oneLine(valueLabel(w.th, sizeLabel, w.pal.Muted, footText(legview.Foot(l.Angles)))).Layout)
				}
				return sideBySide(gtx, 8, ws...)
			})
		}),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions { return w.sideView(gtx, st) }),
		layout.Rigid(layout.Spacer{Height: 8}.Layout),
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			side := min(gtx.Constraints.Max.X*48/100, gtx.Dp(260))
			return layout.Flex{}.Layout(gtx,
				layout.Flexed(1, func(gtx layout.Context) layout.Dimensions { return w.jointTable(gtx, st) }),
				layout.Rigid(layout.Spacer{Width: 8}.Layout),
				layout.Rigid(func(gtx layout.Context) layout.Dimensions {
					gtx.Constraints = layout.Exact(image.Pt(side, side))
					return w.topView(gtx, st)
				}),
			)
		}),
	)
}

// legColour: geometry in ink, or muted while output is off; the selected
// joint in the accent.
func (w *window) legColour(st legsState) color.NRGBA {
	if st.Pose.OutputsOn {
		return w.pal.Fg
	}
	return w.pal.Muted
}

// canvas lays out a drawing on a well, clipped to it.
func (w *window) canvas(gtx layout.Context, draw func(gtx layout.Context, size image.Point)) layout.Dimensions {
	size := gtx.Constraints.Max
	r := image.Rectangle{Max: size}
	fill(gtx, r, gtx.Dp(8), w.pal.Well)
	outline(gtx, r, gtx.Dp(8), gtx.Dp(1), w.pal.Border)
	cl := clip.UniformRRect(r, gtx.Dp(8)).Push(gtx.Ops)
	draw(gtx, size)
	cl.Pop()
	return layout.Dimensions{Size: size}
}

// grid draws faint lines every step millimetres, and the axes through the
// origin a little stronger.
func (w *window) grid(gtx layout.Context, v viewport, bounds [4]float64, step float64) {
	faint := faded(w.pal.Muted, 0x30)
	axis := faded(w.pal.Muted, 0x70)
	for x := math.Ceil(bounds[0]/step) * step; x <= bounds[2]; x += step {
		c := faint
		if x == 0 {
			c = axis
		}
		strokePath(gtx, []f32.Point{v.pt(legview.Vec2{X: x, Y: bounds[1]}), v.pt(legview.Vec2{X: x, Y: bounds[3]})}, 1, c)
	}
	for y := math.Ceil(bounds[1]/step) * step; y <= bounds[3]; y += step {
		c := faint
		if y == 0 {
			c = axis
		}
		strokePath(gtx, []f32.Point{v.pt(legview.Vec2{X: bounds[0], Y: y}), v.pt(legview.Vec2{X: bounds[2], Y: y})}, 1, c)
	}
}

// limitRing marks a joint near its servo limit (a yellow ring, outlined so
// it shows on paper) or at it (a heavier ring in yellow and accent).
func (w *window) limitRing(gtx layout.Context, p f32.Point, l legview.Limit) {
	switch l {
	case legview.LimitNear:
		ring(gtx, p, float32(gtx.Dp(11)), float32(gtx.Dp(3)), w.pal.Yellow)
		ring(gtx, p, float32(gtx.Dp(13)), float32(gtx.Dp(1)), w.pal.Muted)
	case legview.LimitAt:
		ring(gtx, p, float32(gtx.Dp(11)), float32(gtx.Dp(4)), w.pal.Yellow)
		ring(gtx, p, float32(gtx.Dp(14)), float32(gtx.Dp(1)), w.pal.AccentText)
	}
}

// sideView: the selected leg in its vertical plane (outward to the right,
// up), the body edge at the pivot, each joint's pulse next
// to it, and the coxa angle in an inset seen from above.
func (w *window) sideView(gtx layout.Context, st legsState) layout.Dimensions {
	return w.canvas(gtx, func(gtx layout.Context, size image.Point) {
		l := st.Pose.Legs[st.Leg]
		v := fit(sideBounds, size)
		w.grid(gtx, v, sideBounds, 50)
		// The body, to the left of the pivot.
		body := image.Rectangle{Min: v.pt(legview.Vec2{X: sideBounds[0], Y: 25}).Round(), Max: v.pt(legview.Vec2{X: -6, Y: -25}).Round()}
		fill(gtx, body, gtx.Dp(4), w.pal.Surface)
		outline(gtx, body, gtx.Dp(4), gtx.Dp(1), w.pal.Border)
		defer w.caption(gtx, "side · outward →, up ↑") // drawn last, over the geometry
		if !l.Set {
			centredAt(gtx, f32.Pt(float32(size.X)/2, float32(size.Y)/2), size,
				tracked(w.th, sizeLabel, w.pal.Muted, "not set since the robot started"))
			return
		}
		s := legview.SideView(l.Angles)
		pts := []f32.Point{v.pt(s.Pivot), v.pt(s.CoxaEnd), v.pt(s.Knee), v.pt(s.Foot)}
		base := w.legColour(st)
		thick := float32(gtx.Dp(6))
		for j := 0; j < 3; j++ {
			c := base
			if j == st.Joint {
				c = w.pal.Accent
			}
			strokePath(gtx, pts[j:j+2], thick, c)
		}
		disc(gtx, pts[3], float32(gtx.Dp(4)), base)
		for j := 0; j < 3; j++ {
			c := base
			if j == st.Joint {
				c = w.pal.Accent
			}
			disc(gtx, pts[j], float32(gtx.Dp(7)), c)
			disc(gtx, pts[j], float32(gtx.Dp(3)), w.pal.Well)
			w.limitRing(gtx, pts[j], l.Joints[j].Limit)
		}
		// Each joint's pulse in a tag, away from the joint's segments and
		// pushed further out until it clears the tags already placed and
		// the coxa inset.
		caption := image.Rect(0, 0, gtx.Dp(200), gtx.Dp(22))
		placed := []image.Rectangle{w.insetRect(gtx, size), caption}
		for _, p := range pts {
			r := gtx.Dp(9)
			placed = append(placed, image.Rect(int(p.X)-r, int(p.Y)-r, int(p.X)+r, int(p.Y)+r))
		}
		for j := 0; j < 3; j++ {
			jt := l.Joints[j]
			c := w.pal.Fg
			if j == st.Joint {
				c = w.pal.AccentText
			}
			label := legview.JointNames[j]
			if jt.Limit != legview.LimitNone {
				label += " · " + jt.Limit.String()
			}
			tagW := func(gtx layout.Context) layout.Dimensions {
				return w.tag(gtx, label, jointPulseText(jt.PulseUs), c)
			}
			m := op.Record(gtx.Ops)
			g := gtx
			g.Constraints.Min = image.Point{}
			d := tagW(g)
			call := m.Stop()
			r := placeTag(gtx, pts[j], tagDirection(pts, j), d.Size, size, placed)
			placed = append(placed, r)
			t := op.Offset(r.Min).Push(gtx.Ops)
			call.Add(gtx.Ops)
			t.Pop()
		}
		w.coxaInset(gtx, size, st)
	})
}

// placeTag finds a place for a tag of the given size near a joint: along
// the preferred direction first, then turned further from it, each at
// growing distances, kept inside the canvas and clear of the others. If
// nothing is clear it takes the preferred place.
func placeTag(gtx layout.Context, joint, dir f32.Point, tag, size image.Point, others []image.Rectangle) image.Rectangle {
	place := func(d f32.Point, dist int) image.Rectangle {
		p := joint.Add(d.Mul(float32(dist)))
		return clampRect(image.Rectangle{Max: tag}.Add(image.Pt(int(p.X)-tag.X/2, int(p.Y)-tag.Y/2)), size)
	}
	for _, turn := range []float64{0, 40, -40, 80, -80, 120, -120, 180} {
		s, c := math.Sincos(turn * math.Pi / 180)
		d := f32.Pt(dir.X*float32(c)-dir.Y*float32(s), dir.X*float32(s)+dir.Y*float32(c))
		for step := 0; step < 6; step++ {
			r := place(d, gtx.Dp(unit.Dp(26+14*step)))
			if !overlapsAny(r, others) {
				return r
			}
		}
	}
	return place(dir, gtx.Dp(26))
}

// tagDirection points away from the segments meeting at joint j (the pivot
// also has the body on its left).
func tagDirection(pts []f32.Point, j int) f32.Point {
	unit := func(p f32.Point) f32.Point {
		n := float32(math.Hypot(float64(p.X), float64(p.Y)))
		if n < 1e-3 {
			return f32.Point{}
		}
		return p.Mul(1 / n)
	}
	var sum f32.Point
	if j == 0 {
		sum = f32.Pt(-1, 0) // the body
	} else {
		sum = unit(pts[j-1].Sub(pts[j]))
	}
	sum = sum.Add(unit(pts[j+1].Sub(pts[j])))
	d := unit(sum.Mul(-1))
	if d == (f32.Point{}) {
		return f32.Pt(0, 1) // straight through: below
	}
	return d
}

// tag is a small two-line label on a surface: the joint and its values.
func (w *window) tag(gtx layout.Context, title, value string, c color.NRGBA) layout.Dimensions {
	m := op.Record(gtx.Ops)
	d := layout.Inset{Top: 3, Bottom: 3, Left: 5, Right: 5}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
		return stacked(gtx, 1, tracked(w.th, 9, c, title), valueLabel(w.th, 11, w.pal.Fg, value).Layout)
	})
	call := m.Stop()
	fill(gtx, image.Rectangle{Max: d.Size}, gtx.Dp(4), faded(w.pal.Surface, 0xE6))
	call.Add(gtx.Ops)
	return d
}

// clampRect moves r inside a canvas of the given size.
func clampRect(r image.Rectangle, size image.Point) image.Rectangle {
	dx := max(0, -r.Min.X) - max(0, r.Max.X-size.X)
	dy := max(0, -r.Min.Y) - max(0, r.Max.Y-size.Y)
	return r.Add(image.Pt(dx, dy))
}

func overlapsAny(r image.Rectangle, others []image.Rectangle) bool {
	for _, o := range others {
		if r.Overlaps(o) {
			return true
		}
	}
	return false
}

// insetRect is where the coxa inset goes: the side view's top-right corner.
func (w *window) insetRect(gtx layout.Context, size image.Point) image.Rectangle {
	side := gtx.Dp(96)
	pad := gtx.Dp(8)
	return image.Rectangle{Min: image.Pt(size.X-side-pad, pad), Max: image.Pt(size.X-pad, pad+side)}
}

// coxaInset: the coxa angle seen from above in the leg's frame (x outward
// to the right, +y up), in the side view's top-right corner.
func (w *window) coxaInset(gtx layout.Context, size image.Point, st legsState) {
	l := st.Pose.Legs[st.Leg]
	r := w.insetRect(gtx, size)
	side := r.Dy()
	fill(gtx, r, gtx.Dp(6), w.pal.Surface)
	outline(gtx, r, gtx.Dp(6), gtx.Dp(1), w.pal.Border)
	o := f32.Pt(float32(r.Min.X+gtx.Dp(14)), float32(r.Min.Y+side/2))
	length := float32(side/2 - gtx.Dp(12))
	strokePath(gtx, []f32.Point{o, o.Add(f32.Pt(length, 0))}, 1, faded(w.pal.Muted, 0x90))
	a := l.Angles.Coxa * math.Pi / 180
	tip := o.Add(f32.Pt(float32(math.Cos(a))*length, -float32(math.Sin(a))*length))
	c := w.legColour(st)
	if st.Joint == legview.Coxa {
		c = w.pal.Accent
	}
	strokePath(gtx, []f32.Point{o, tip}, float32(gtx.Dp(4)), c)
	disc(gtx, o, float32(gtx.Dp(4)), c)
	at(gtx, image.Pt(r.Min.X+gtx.Dp(6), r.Min.Y+gtx.Dp(4)), tracked(w.th, 9, w.pal.Muted, "top · coxa"))
	// The angle in the bottom-right corner, clear of the coxa's swing.
	value := valueLabel(w.th, 11, w.pal.Fg, angleText(l.Joints[legview.Coxa].Commanded))
	m := op.Record(gtx.Ops)
	g := gtx
	g.Constraints.Min = image.Point{}
	d := value.Layout(g)
	call := m.Stop()
	t := op.Offset(image.Pt(r.Max.X-gtx.Dp(6)-d.Size.X, r.Max.Y-gtx.Dp(4)-d.Size.Y)).Push(gtx.Ops)
	call.Add(gtx.Ops)
	t.Pop()
}

// jointTable: the selected leg's joints with their commanded and servo
// angles and pulses, and any limit.
func (w *window) jointTable(gtx layout.Context, st legsState) layout.Dimensions {
	l := st.Pose.Legs[st.Leg]
	cols := []unit.Dp{46, 66, 66, 62}
	row := func(ws ...layout.Widget) layout.Widget {
		return func(gtx layout.Context) layout.Dimensions {
			children := make([]layout.FlexChild, 0, len(ws))
			for i, wd := range ws {
				width := gtx.Dp(cols[i])
				children = append(children, layout.Rigid(func(gtx layout.Context) layout.Dimensions {
					d := wd(gtx)
					d.Size.X = max(d.Size.X, width)
					return d
				}))
			}
			return layout.Flex{Alignment: layout.Baseline}.Layout(gtx, children...)
		}
	}
	rows := []layout.Widget{
		tracked(w.th, sizeLabel, w.pal.Fg, "Joints"),
		row(tracked(w.th, 9, w.pal.Muted, "joint"), tracked(w.th, 9, w.pal.Muted, "command"),
			tracked(w.th, 9, w.pal.Muted, "servo"), tracked(w.th, 9, w.pal.Muted, "pulse")),
	}
	for j := 0; j < legview.JointsPerLeg; j++ {
		jt := l.Joints[j]
		name := w.pal.Fg
		if j == st.Joint {
			name = w.pal.AccentText
		}
		rows = append(rows, row(
			tracked(w.th, 10, name, legview.JointNames[j]),
			valueLabel(w.th, 12, w.pal.Fg, angleText(jt.Commanded)).Layout,
			valueLabel(w.th, 12, w.pal.Fg, angleText(jt.Servo)).Layout,
			valueLabel(w.th, 12, w.pal.Fg, jointPulseText(jt.PulseUs)).Layout,
		))
		if jt.Limit != legview.LimitNone {
			rows = append(rows, func(gtx layout.Context) layout.Dimensions {
				return layout.Inset{Left: cols[0]}.Layout(gtx, w.chip(jt.Limit.String(), w.pal.Yellow))
			})
		}
	}
	rows = append(rows, wrapped(textLabel(w.th, 12, w.pal.Muted,
		"Command is the IK angle; servo adds the offset in use. The drawing shows the commanded pose.")))
	return stacked(gtx, 5, rows...)
}

// topView: all six legs from above on the schematic body, front up, the
// selected leg in the accent and limits ringed.
func (w *window) topView(gtx layout.Context, st legsState) layout.Dimensions {
	return w.canvas(gtx, func(gtx layout.Context, size image.Point) {
		v := fit(topBounds, size)
		w.grid(gtx, v, topBounds, 100)
		defer w.caption(gtx, "top · front ↑ (assumed)")
		// Body.
		outlinePts := legview.BodyOutline()
		var p clip.Path
		p.Begin(gtx.Ops)
		for i, q := range outlinePts {
			if i == 0 {
				p.MoveTo(v.pt(q))
			} else {
				p.LineTo(v.pt(q))
			}
		}
		p.Close()
		spec := p.End()
		paint.FillShape(gtx.Ops, w.pal.Surface, clip.Outline{Path: spec}.Op())
		closed := make([]f32.Point, 0, len(outlinePts)+1)
		for _, q := range outlinePts {
			closed = append(closed, v.pt(q))
		}
		closed = append(closed, closed[0])
		strokePath(gtx, closed, float32(gtx.Dp(2)), w.pal.Border)
		// The assumed front.
		tipY := 105.0
		strokePath(gtx, []f32.Point{v.pt(legview.Vec2{X: -22, Y: tipY - 26}), v.pt(legview.Vec2{X: 0, Y: tipY}), v.pt(legview.Vec2{X: 22, Y: tipY - 26})},
			float32(gtx.Dp(2)), w.pal.AccentText)

		for leg := 0; leg < legview.Legs; leg++ {
			w.topLeg(gtx, v, size, st, leg)
		}
	})
}

func (w *window) topLeg(gtx layout.Context, v viewport, size image.Point, st legsState, leg int) {
	m := legview.Mounts[leg]
	l := st.Pose.Legs[leg]
	c := w.legColour(st)
	if leg == st.Leg {
		c = w.pal.Accent
	}
	pivot := v.pt(m.Pivot)
	// The leg number, inside the body.
	inner := legview.Vec2{X: m.Pivot.X * 0.62, Y: m.Pivot.Y * 0.62}
	numColour := w.pal.Fg
	if leg == st.Leg {
		numColour = w.pal.AccentText
	}
	centredAt(gtx, v.pt(inner), size, valueLabel(w.th, 12, numColour, strconv.Itoa(legNumber(leg))).Layout)
	if !l.Set {
		disc(gtx, pivot, float32(gtx.Dp(4)), w.pal.Muted)
		out := m.ToBody(legview.Vec3{X: 70})
		centredAt(gtx, v.pt(out), size, tracked(w.th, 9, w.pal.Muted, "not set"))
		return
	}
	s := legview.SideView(l.Angles)
	pts := []f32.Point{
		pivot,
		v.pt(m.ToBody(legview.Plane(l.Angles, s.CoxaEnd))),
		v.pt(m.ToBody(legview.Plane(l.Angles, s.Knee))),
		v.pt(m.ToBody(legview.Plane(l.Angles, s.Foot))),
	}
	strokePath(gtx, pts[:2], float32(gtx.Dp(5)), c)
	strokePath(gtx, pts[1:], float32(gtx.Dp(3)), c)
	disc(gtx, pts[3], float32(gtx.Dp(5)), c)
	for j := 0; j < 3; j++ {
		disc(gtx, pts[j], float32(gtx.Dp(3)), c)
		if l.Joints[j].Limit != legview.LimitNone {
			w.limitRing(gtx, pts[j], l.Joints[j].Limit)
		}
	}
}

// caption labels a drawing in its top-left corner, on a backing so that
// geometry under it does not hide it.
func (w *window) caption(gtx layout.Context, s string) {
	m := op.Record(gtx.Ops)
	g := gtx
	g.Constraints.Min = image.Point{}
	d := layout.Inset{Top: 3, Bottom: 3, Left: 5, Right: 5}.Layout(g, tracked(w.th, 10, w.pal.Muted, s))
	call := m.Stop()
	t := op.Offset(image.Pt(gtx.Dp(4), gtx.Dp(4))).Push(gtx.Ops)
	fill(gtx, image.Rectangle{Max: d.Size}, gtx.Dp(4), faded(w.pal.Well, 0xD8))
	call.Add(gtx.Ops)
	t.Pop()
}
