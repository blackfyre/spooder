package ui

import (
	"context"
	"fmt"
	"image"
	"image/color"
	"math"
	"sort"
	"strconv"
	"sync/atomic"
	"time"

	"gioui.org/app"
	"gioui.org/f32"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/op/clip"
	"gioui.org/op/paint"
	"gioui.org/unit"
	"gioui.org/widget"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/legview"
)

// viewsRefresh is the pose rate: the views window redraws this often.
const viewsRefresh = 100 * time.Millisecond

// viewsWindow draws the whole robot in four views (top, front, left,
// isometric) from the latest pose, in its own window.
type viewsWindow struct {
	env   Env
	d     *window // the drawing helpers: theme and palette
	looks chan appearance.Settings

	// The centre of gravity's battery: fitted or not, off when the window
	// opens.
	battery      bool
	batteryClick widget.Clickable
}

func newViewsWindow(env Env) (*viewsWindow, error) {
	d, err := newWindow(env)
	if err != nil {
		return nil, err
	}
	return &viewsWindow{env: env, d: d, looks: make(chan appearance.Settings, 4)}, nil
}

// selectedJoint is the main window's selection, or 0 without one.
func (v *viewsWindow) selectedJoint() int {
	if v.env.Selection == nil {
		return 0
	}
	return int(v.env.Selection.Load())
}

func (v *viewsWindow) frame(gtx layout.Context) {
	for {
		select {
		case s := <-v.looks:
			if s != v.d.look {
				v.d.setAppearance(s)
			}
			continue
		default:
		}
		break
	}
	d := v.d
	d.snap = v.env.Session.Snapshot()
	paint.Fill(gtx.Ops, d.pal.Bg)
	gtx.Execute(op.InvalidateCmd{At: gtx.Now.Add(viewsRefresh)})
	st := legsFor(d.snap, selection{joint: v.selectedJoint()})
	if v.batteryClick.Clicked(gtx) {
		v.battery = !v.battery
	}
	var legs [legview.Legs]legview.Leg3
	var grav gravityState
	if st.Current {
		legs = legview.Body3(st.Pose)
		grav = gravityFor(legs, v.battery)
	}
	layout.UniformInset(12).Layout(gtx, func(gtx layout.Context) layout.Dimensions {
		return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
			layout.Rigid(func(gtx layout.Context) layout.Dimensions { return v.gravityBar(gtx, grav) }),
			layout.Rigid(layout.Spacer{Height: 8}.Layout),
			layout.Flexed(1, func(gtx layout.Context) layout.Dimensions { return v.views(gtx, st, legs, grav) }),
		)
	})
}

// views draws the four views, or why there is no pose.
func (v *viewsWindow) views(gtx layout.Context, st legsState, legs [legview.Legs]legview.Leg3, grav gravityState) layout.Dimensions {
	d := v.d
	gtx.Constraints.Min = gtx.Constraints.Max
	if !st.Current {
		return panel(gtx, d.pal.Surface, d.pal.Border, radiusPanel, 16, func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints.Min = gtx.Constraints.Max
			return stacked(gtx, 8,
				tracked(d.th, sizeTitle, d.pal.Muted, "no pose"),
				wrapped(textLabel(d.th, sizeText, d.pal.Muted, st.Reason)),
			)
		})
	}
	// One scale for the three orthographic views: the smallest of
	// their fits into a quarter of the window.
	cell := image.Pt((gtx.Constraints.Max.X-gtx.Dp(8))/2, (gtx.Constraints.Max.Y-gtx.Dp(8))/2)
	scale := orthoScale(cell)
	draw := func(view legview.View) layout.Widget {
		return func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints = layout.Exact(cell)
			return d.canvas(gtx, func(gtx layout.Context, size image.Point) {
				vp := fit(legview.ViewBounds(view), size)
				if view != legview.ViewIsometric {
					vp = fitAt(legview.ViewBounds(view), size, scale)
				}
				d.robotView(gtx, view, vp, size, legs, st, grav)
			})
		}
	}
	row := func(a, b legview.View) layout.Widget {
		return func(gtx layout.Context) layout.Dimensions {
			return layout.Flex{}.Layout(gtx,
				layout.Rigid(draw(a)), layout.Rigid(layout.Spacer{Width: 8}.Layout), layout.Rigid(draw(b)))
		}
	}
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
		layout.Rigid(row(legview.ViewTop, legview.ViewIsometric)),
		layout.Rigid(layout.Spacer{Height: 8}.Layout),
		layout.Rigid(row(legview.ViewFront, legview.ViewLeft)),
	)
}

// gravityState is the estimated centre of gravity of a pose and its support.
type gravityState struct {
	ok        bool // an estimate: every leg set
	g         legview.Gravity
	hull      []legview.Vec2 // nil: fewer than three feet down
	margin    float64
	hasMargin bool
}

func gravityFor(legs [legview.Legs]legview.Leg3, battery bool) gravityState {
	g, ok := legview.CentreOfGravity(legs, battery)
	if !ok {
		return gravityState{}
	}
	s := gravityState{ok: true, g: g, hull: legview.SupportHull(legs)}
	if s.hull != nil {
		s.margin, s.hasMargin = legview.SupportMargin(s.hull, legview.Vec2{X: g.Centre.X, Y: g.Centre.Y}), true
	}
	return s
}

// Support margins: from marginFineMM the centre is well inside its support
// (the host tests' margin), from 0 close to an edge, below 0 outside it.
const marginFineMM = 20.0

// gravityColour marks the margin: fine, warning, or outside.
func (w *window) gravityColour(s gravityState) color.NRGBA {
	switch {
	case !s.hasMargin:
		return w.pal.Muted
	case s.margin >= marginFineMM:
		return w.pal.Green
	case s.margin >= 0:
		return w.pal.Yellow
	}
	return w.pal.Accent
}

// gravityText is the readout, short enough for the narrowest window: the
// estimated centre of gravity (CoG) in the body frame, the mass and
// the margin.
func gravityText(s gravityState) string {
	if !s.ok {
		return "CoG · no estimate"
	}
	c := s.g.Centre
	text := fmt.Sprintf("CoG · x %.0f  y %.0f  z %.0f mm · %.2f kg · ", c.X, c.Y, c.Z, s.g.MassG/1000)
	if !s.hasMargin {
		return text + "margin unavailable"
	}
	if s.margin < 0 {
		return text + fmt.Sprintf("%.0f mm outside its support", -s.margin)
	}
	return text + fmt.Sprintf("margin %.0f mm", s.margin)
}

// gravityBar: the battery toggle and the estimate's readout.
func (v *viewsWindow) gravityBar(gtx layout.Context, s gravityState) layout.Dimensions {
	d := v.d
	kind, label := kindQuiet, "Battery · none"
	if v.battery {
		kind, label = kindActive, "Battery · 2S 5000 mAh"
	}
	dot := func(gtx layout.Context) layout.Dimensions {
		size := gtx.Dp(10)
		if s.ok {
			disc(gtx, f32.Pt(float32(size)/2, float32(size)/2), float32(size)/2, d.gravityColour(s))
		}
		return layout.Dimensions{Size: image.Pt(size, size)}
	}
	return layout.Flex{Alignment: layout.Middle}.Layout(gtx,
		layout.Rigid(d.button(&v.batteryClick, kind, label, true).width(170).Layout),
		layout.Rigid(layout.Spacer{Width: 14}.Layout),
		layout.Rigid(dot),
		layout.Rigid(layout.Spacer{Width: 8}.Layout),
		layout.Rigid(oneLine(valueLabel(d.th, 12, d.pal.Fg, gravityText(s))).Layout),
	)
}

// orthoScale is the one scale of the top, front and left views in a cell:
// the smallest of their fits, so sizes compare across them.
func orthoScale(cell image.Point) float64 {
	scale := math.Inf(1)
	for _, view := range []legview.View{legview.ViewTop, legview.ViewFront, legview.ViewLeft} {
		scale = math.Min(scale, fit(legview.ViewBounds(view), cell).scale)
	}
	return scale
}

// fitAt is fit at a given scale, centred.
func fitAt(bounds [4]float64, size image.Point, s float64) viewport {
	w, h := bounds[2]-bounds[0], bounds[3]-bounds[1]
	return viewport{scale: s, minX: bounds[0], maxY: bounds[3],
		off: f32.Pt(float32((float64(size.X)-w*s)/2), float32((float64(size.Y)-h*s)/2))}
}

// projected is a body point on screen.
func projected(view legview.View, vp viewport, p legview.Point3) f32.Point {
	q, _ := legview.Project(view, p)
	return vp.pt(q)
}

// robotView draws one view: the ground, the body plate, the legs far to
// near, the front marker in the top view, and the caption.
func (w *window) robotView(gtx layout.Context, view legview.View, vp viewport, size image.Point, legs [legview.Legs]legview.Leg3, st legsState, grav gravityState) {
	defer w.caption(gtx, legview.ViewNames[view]+" · schematic body")
	// Drawn after the legs, under the caption: deferred calls run last first.
	defer w.gravityMark(gtx, view, vp, legs, grav)
	// The ground, where the lowest foot stands below the body (not with the
	// feet raised, as on a stand in the calibration pose).
	ground, haveGround := legview.Ground(legs)
	if haveGround && ground < -legview.PlateThickness/2 && view != legview.ViewTop {
		w.groundLine(gtx, view, vp, size, ground)
	}
	if view == legview.ViewTop {
		w.supportPolygon(gtx, vp, grav)
	}

	anchors := make([]f32.Point, legview.Legs)
	plan, order := drawPlan(view, legs, st.Leg)
	defer w.legNumbers(gtx, size, anchors, order, st)
	for _, step := range plan {
		switch step.kind {
		case stepPlate:
			w.plate(gtx, view, vp)
		case stepFront:
			tipY := 105.0
			strokePath(gtx, []f32.Point{
				projected(view, vp, legview.Point3{X: -22, Y: tipY - 26, Z: legview.PlateThickness / 2}),
				projected(view, vp, legview.Point3{X: 0, Y: tipY, Z: legview.PlateThickness / 2}),
				projected(view, vp, legview.Point3{X: 22, Y: tipY - 26, Z: legview.PlateThickness / 2}),
			}, float32(gtx.Dp(2)), w.pal.AccentText)
		case stepLeg:
			anchors[step.leg] = w.robotLeg(gtx, view, vp, legs[step.leg], step.leg, st)
		case stepSelected:
			w.selectedOverlay(gtx, view, vp, legs[step.leg])
		case stepRings:
			w.legRings(gtx, view, vp, legs[step.leg], step.leg, st)
		}
	}
}

// supportPolygon draws the feet's support polygon in the top view, under
// the body.
func (w *window) supportPolygon(gtx layout.Context, vp viewport, s gravityState) {
	if s.hull == nil {
		return
	}
	pts := make([]f32.Point, 0, len(s.hull)+1)
	for _, q := range s.hull {
		pts = append(pts, vp.pt(q))
	}
	pts = append(pts, pts[0])
	var p clip.Path
	p.Begin(gtx.Ops)
	p.MoveTo(pts[0])
	for _, q := range pts[1:] {
		p.LineTo(q)
	}
	p.Close()
	c := w.gravityColour(s)
	paint.FillShape(gtx.Ops, faded(c, 0x1C), clip.Outline{Path: p.End()}.Op())
	strokePath(gtx, pts, float32(gtx.Dp(1)), faded(c, 0xC0))
}

// gravityMark marks the centre of gravity, coloured by its margin: a ringed
// dot, and in the side views a line down to the ground where it is drawn.
func (w *window) gravityMark(gtx layout.Context, view legview.View, vp viewport, legs [legview.Legs]legview.Leg3, s gravityState) {
	if !s.ok {
		return
	}
	c := w.gravityColour(s)
	at := projected(view, vp, s.g.Centre)
	if ground, ok := legview.Ground(legs); ok && view != legview.ViewTop && ground < -legview.PlateThickness/2 {
		foot := legview.Point3{X: s.g.Centre.X, Y: s.g.Centre.Y, Z: ground}
		down := projected(view, vp, foot)
		strokePath(gtx, []f32.Point{at, down}, float32(gtx.Dp(1)), c)
		disc(gtx, down, float32(gtx.Dp(3)), c)
	}
	// Ringed in the ink as well, so the light status colours read on paper.
	disc(gtx, at, float32(gtx.Dp(6)), w.pal.Bg)
	disc(gtx, at, float32(gtx.Dp(4)), c)
	ring(gtx, at, float32(gtx.Dp(8)), float32(gtx.Dp(2)), c)
	ring(gtx, at, float32(gtx.Dp(10)), float32(gtx.Dp(1)), w.pal.Fg)
}

// What a view draws, in order.
type stepKind int

const (
	stepPlate    stepKind = iota // the body plate
	stepFront                    // the front marker (top view)
	stepLeg                      // a leg
	stepSelected                 // the selected leg again, thin, over whatever covers it
	stepRings                    // a leg's limit rings, over everything
)

type drawStep struct {
	kind stepKind
	leg  int
}

// drawPlan orders a view's drawing: legs far to near, the plate between
// the legs behind it and those in front (under every leg in the top view,
// where the legs reach out from under it), then the selected leg again,
// thin, so a nearer leg cannot hide it, and the limit rings last. It also
// returns the legs far to near, for the numbers.
func drawPlan(view legview.View, legs [legview.Legs]legview.Leg3, selected int) ([]drawStep, []int) {
	order := make([]int, legview.Legs)
	depth := make([]float64, legview.Legs)
	for leg := range order {
		order[leg] = leg
		_, depth[leg] = legview.Project(view, legs[leg].Points[0])
	}
	sort.SliceStable(order, func(a, b int) bool { return depth[order[a]] < depth[order[b]] })
	var plan []drawStep
	_, plateDepth := legview.Project(view, legview.Point3{})
	plateDrawn := false
	if view == legview.ViewTop {
		plan, plateDrawn = append(plan, drawStep{kind: stepPlate}), true
	}
	for _, leg := range order {
		if !plateDrawn && depth[leg] >= plateDepth {
			plan, plateDrawn = append(plan, drawStep{kind: stepPlate}), true
		}
		plan = append(plan, drawStep{kind: stepLeg, leg: leg})
	}
	if !plateDrawn {
		plan = append(plan, drawStep{kind: stepPlate})
	}
	if view == legview.ViewTop {
		plan = append(plan, drawStep{kind: stepFront})
	}
	if selected >= 0 && selected < legview.Legs && legs[selected].Set {
		plan = append(plan, drawStep{kind: stepSelected, leg: selected})
	}
	for _, leg := range order {
		plan = append(plan, drawStep{kind: stepRings, leg: leg})
	}
	return plan, order
}

// selectedOverlay draws the selected leg again, thin, in the accent.
func (w *window) selectedOverlay(gtx layout.Context, view legview.View, vp viewport, l legview.Leg3) {
	pts := make([]f32.Point, 4)
	for i, p := range l.Points {
		pts[i] = projected(view, vp, p)
	}
	for i := 0; i < 3; i++ {
		strokePath(gtx, pts[i:i+2], float32(gtx.Dp(2)), w.pal.Accent)
	}
	for _, p := range pts {
		disc(gtx, p, float32(gtx.Dp(2)), w.pal.Accent)
	}
}

// legRings rings a leg's joints near or at their servo limits.
func (w *window) legRings(gtx layout.Context, view legview.View, vp viewport, l legview.Leg3, leg int, st legsState) {
	if !l.Set {
		return
	}
	for j := 0; j < 3; j++ {
		if lim := st.Pose.Legs[leg].Joints[j].Limit; lim != legview.LimitNone {
			w.limitRing(gtx, projected(view, vp, l.Points[j]), lim)
		}
	}
}

// groundLine: a dashed line at the lowest foot, or in the isometric view a
// faint grid on that plane.
func (w *window) groundLine(gtx layout.Context, view legview.View, vp viewport, size image.Point, z float64) {
	c := faded(w.pal.Muted, 0xA0)
	if view != legview.ViewIsometric {
		y := projected(view, vp, legview.Point3{Z: z}).Y
		dashed(gtx, 0, float32(size.X), y, c)
		at(gtx, image.Pt(gtx.Dp(6), int(y)+gtx.Dp(2)), oneLine(valueLabel(w.th, 10, w.pal.Muted, "ground")).Layout)
		return
	}
	faint := faded(w.pal.Muted, 0x38)
	for g := -400.0; g <= 400; g += 100 {
		strokePath(gtx, []f32.Point{projected(view, vp, legview.Point3{X: g, Y: -400, Z: z}), projected(view, vp, legview.Point3{X: g, Y: 400, Z: z})}, 1, faint)
		strokePath(gtx, []f32.Point{projected(view, vp, legview.Point3{X: -400, Y: g, Z: z}), projected(view, vp, legview.Point3{X: 400, Y: g, Z: z})}, 1, faint)
	}
}

// plate draws the schematic body: its top face filled, and the prism's
// visible edges.
func (w *window) plate(gtx layout.Context, view legview.View, vp viewport) {
	top, bottom := legview.Plate()
	face := func(pts []legview.Point3) []f32.Point {
		out := make([]f32.Point, 0, len(pts)+1)
		for _, p := range pts {
			out = append(out, projected(view, vp, p))
		}
		return append(out, out[0])
	}
	fillPoly := func(pts []f32.Point) {
		var p clip.Path
		p.Begin(gtx.Ops)
		p.MoveTo(pts[0])
		for _, q := range pts[1:] {
			p.LineTo(q)
		}
		p.Close()
		paint.FillShape(gtx.Ops, w.pal.Surface, clip.Outline{Path: p.End()}.Op())
	}
	edge := float32(gtx.Dp(2))
	if view == legview.ViewTop {
		t := face(top)
		fillPoly(t)
		strokePath(gtx, t, edge, w.pal.Border)
		return
	}
	// The side faces, as quadrilaterals, then the top face over them.
	for i := range top {
		j := (i + 1) % len(top)
		quad := face([]legview.Point3{top[i], top[j], bottom[j], bottom[i]})
		fillPoly(quad)
		strokePath(gtx, quad, edge, w.pal.Border)
	}
	t := face(top)
	fillPoly(t)
	strokePath(gtx, t, edge, w.pal.Border)
}

// robotLeg draws a leg as the top view of the legs pane does: the coxa
// thick, femur and tibia thinner, joints as discs, limits ringed, the
// number by the pivot, and the selected leg in the accent.
// It returns where the leg's number goes: above the knee, or the pivot of
// a leg never set.
func (w *window) robotLeg(gtx layout.Context, view legview.View, vp viewport, l legview.Leg3, leg int, st legsState) f32.Point {
	c := w.legColour(st)
	if leg == st.Leg {
		c = w.pal.Accent
	}
	pts := make([]f32.Point, 4)
	for i, p := range l.Points {
		pts[i] = projected(view, vp, p)
	}
	above := f32.Pt(0, -float32(gtx.Dp(14)))
	if !l.Set {
		disc(gtx, pts[0], float32(gtx.Dp(4)), w.pal.Muted)
		return pts[0].Add(above)
	}
	strokePath(gtx, pts[:2], float32(gtx.Dp(5)), c)
	// Femur and tibia stroked apart: the stroker leaves faint streaks on a
	// polyline with a right angle at the knee. The knee's disc joins them.
	strokePath(gtx, pts[1:3], float32(gtx.Dp(3)), c)
	strokePath(gtx, pts[2:4], float32(gtx.Dp(3)), c)
	disc(gtx, pts[3], float32(gtx.Dp(5)), c)
	for j := 0; j < 3; j++ {
		disc(gtx, pts[j], float32(gtx.Dp(3)), c) // limit rings come last (legRings)
	}
	return pts[2].Add(above)
}

// numberGroups gathers legs whose numbers would sit on top of each other
// (in symmetric poses mirrored legs project to one place), nearest first:
// each group is drawn as one label, so every leg keeps its number.
func numberGroups(anchors []f32.Point, order []int, near float32) [][]int {
	var groups [][]int
	for k := len(order) - 1; k >= 0; k-- {
		leg := order[k]
		placed := false
		for g := range groups {
			d := anchors[groups[g][0]].Sub(anchors[leg])
			if d.X*d.X+d.Y*d.Y <= near*near {
				groups[g] = append(groups[g], leg)
				placed = true
				break
			}
		}
		if !placed {
			groups = append(groups, []int{leg})
		}
	}
	return groups
}

// legNumbers draws the legs' numbers: legs at one place share a label
// ("2 · 5", each in its own colour), and a label that would still collide
// with another moves up until it is clear.
func (w *window) legNumbers(gtx layout.Context, size image.Point, anchors []f32.Point, order []int, st legsState) {
	var placed []image.Rectangle
	for _, group := range numberGroups(anchors, order, float32(gtx.Dp(10))) {
		sort.Ints(group)
		parts := make([]layout.Widget, 0, 2*len(group))
		for i, leg := range group {
			c := w.pal.Fg
			if leg == st.Leg {
				c = w.pal.AccentText
			}
			if i > 0 {
				parts = append(parts, oneLine(valueLabel(w.th, 12, w.pal.Muted, "·")).Layout)
			}
			parts = append(parts, oneLine(valueLabel(w.th, 12, c, strconv.Itoa(legNumber(leg)))).Layout)
		}
		row := sideBySideWidget(3, parts...)
		// On a backing, as the caption, so a joint under it does not hide it.
		label := func(gtx layout.Context) layout.Dimensions {
			m := op.Record(gtx.Ops)
			d := layout.Inset{Left: 2, Right: 2}.Layout(gtx, row)
			call := m.Stop()
			fill(gtx, image.Rectangle{Max: d.Size}, gtx.Dp(3), faded(w.pal.Well, 0xD8))
			call.Add(gtx.Ops)
			return d
		}
		m := op.Record(gtx.Ops)
		g := gtx
		g.Constraints.Min = image.Point{}
		d := label(g)
		m.Stop()
		at := anchors[group[0]]
		r := image.Rectangle{Max: d.Size}.Add(image.Pt(int(at.X)-d.Size.X/2, int(at.Y)-d.Size.Y/2))
		for tries := 0; tries < 4 && overlapsAny(r, placed); tries++ {
			r = r.Add(image.Pt(0, -d.Size.Y-2))
			at.Y -= float32(d.Size.Y + 2)
		}
		placed = append(placed, r)
		centredAt(gtx, at, size, label)
	}
}

// viewsOpen guards the views window: one at a time.
var viewsOpen atomic.Bool

// openViews starts the views window unless it is open: run shows it and
// returns when it closes (runViews; tests pass their own). Closing the main
// window ends the process, and the views window with it.
func openViews(env Env, run func(*viewsWindow)) {
	if !viewsOpen.CompareAndSwap(false, true) {
		return
	}
	v, err := newViewsWindow(env)
	if err != nil {
		viewsOpen.Store(false)
		return
	}
	go func() {
		defer viewsOpen.Store(false)
		run(v)
	}()
}

// offerLook hands the newest appearance to the window without ever
// blocking: when the window is not drawing (minimised) and the channel is
// full, the oldest unread setting gives way, so the watcher never hangs
// on a closed window.
func offerLook(ch chan appearance.Settings, s appearance.Settings) {
	for {
		select {
		case ch <- s:
			return
		default:
		}
		select {
		case <-ch:
		default:
		}
	}
}

// runViews is the views window's own Gio window and event loop.
func runViews(v *viewsWindow) {
	win := new(app.Window)
	win.Option(app.Title("Spooder · four views"), app.Size(unit.Dp(1100), unit.Dp(900)), app.MinSize(unit.Dp(640), unit.Dp(520)))
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	appearance.Watch(ctx, func(s appearance.Settings) {
		offerLook(v.looks, s)
		win.Invalidate()
	})
	var ops op.Ops
	for {
		switch e := win.Event().(type) {
		case app.DestroyEvent:
			return
		case app.ConfigEvent:
			if e.Config.Focused {
				offerLook(v.looks, appearance.Read())
			}
		case app.FrameEvent:
			gtx := app.NewContext(&ops, e)
			v.frame(gtx)
			e.Frame(gtx.Ops)
		}
	}
}
