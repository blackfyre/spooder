package ui

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"time"

	"gioui.org/f32"
	"gioui.org/io/event"
	"gioui.org/io/pointer"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/op/clip"
	"gioui.org/unit"
	"gioui.org/widget"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/session"
)

// Chart geometry.
const (
	chartPlotHeight unit.Dp = 78
	chartAxisWidth  unit.Dp = 44
	timelineRow     unit.Dp = 16
	chartCount              = 9 // as buildCharts returns them
)

// chartsState is the charts tab's own state in the window.
type chartsState struct {
	window  int // index into chartWindows
	windows [len(chartWindows)]widget.Clickable
	save    widget.Clickable
	list    widget.List

	// The model, rebuilt when the history changes.
	version uint64
	built   bool
	builtW  int
	builtL  int
	builtAt time.Time
	model   chartsModel

	// Saving: off the UI goroutine, so a slow disk never holds up the keys
	// and the e-stop. saver writes a history and names the file; done
	// carries the outcome back.
	saver  func(session.History) (string, error)
	saving bool
	done   chan saveResult

	// Hover: the plots' tags and horizontal extent, and the time pointed at.
	tags    [chartCount + 1]int // the timeline is the last
	plotX   [chartCount + 1][2]int
	hoverOn bool
	hoverT  float64
	hoverAt int // the plot pointed at; only its Leave ends the hover
}

// chartsRefresh is how often the charts move on with time when no frame
// arrives (a stalled stream, a disconnect), so old samples scroll away.
const chartsRefresh = 250 * time.Millisecond

// chartsModelFor returns the model for the current history, window and leg,
// rebuilt when any changes or time has moved on.
func (w *window) chartsModelFor() chartsModel {
	cs := &w.charts
	leg := w.sel.joint / session.JointKinds
	if v := w.env.Session.HistoryVersion(); !cs.built || v != cs.version || cs.builtW != cs.window || cs.builtL != leg ||
		w.snap.Now.Sub(cs.builtAt) >= chartsRefresh {
		d := chartWindows[cs.window].d
		h, v := w.env.Session.HistorySince(w.snap.Now.Add(-d))
		cs.model = buildCharts(h, w.snap.Now, d, leg)
		cs.version, cs.built, cs.builtW, cs.builtL, cs.builtAt = v, true, cs.window, leg, w.snap.Now
	}
	return cs.model
}

// handleCharts follows the window selector, the CSV button and the hover.
func (w *window) handleCharts(gtx layout.Context) {
	cs := &w.charts
	for i := range cs.windows {
		if cs.windows[i].Clicked(gtx) {
			cs.window = i
		}
	}
	if cs.save.Clicked(gtx) {
		w.saveCSV(gtx)
	}
	w.savedCSV(gtx)
	for i := range cs.tags {
		for {
			ev, ok := gtx.Event(pointer.Filter{Target: &cs.tags[i], Kinds: pointer.Move | pointer.Enter | pointer.Leave})
			if !ok {
				break
			}
			if pe, ok := ev.(pointer.Event); ok {
				w.hoverEvent(i, pe.Kind, pe.Position.X)
			}
		}
	}
}

// hoverEvent follows the pointer over plot i. Moving between plots, the new
// plot's Enter may come before the old one's Leave: only the Leave of the
// plot being hovered ends the hover.
func (w *window) hoverEvent(i int, kind pointer.Kind, x float32) {
	cs := &w.charts
	if kind == pointer.Leave {
		if cs.hoverAt == i {
			cs.hoverOn = false
		}
		return
	}
	w.hoverAt(i, x)
}

// hoverAt points at the moment under x, in plot i's own coordinates.
func (w *window) hoverAt(i int, x float32) {
	cs := &w.charts
	x0, width := cs.plotX[i][0], cs.plotX[i][1]
	if width <= 0 {
		return
	}
	f := math.Max(0, math.Min(1, (float64(x)-float64(x0))/float64(width)))
	cs.hoverOn, cs.hoverT, cs.hoverAt = true, -cs.model.window*(1-f), i
}

type saveResult struct {
	path string
	err  error
}

// saveToDownloads writes a history to the downloads directory.
func saveToDownloads(h session.History) (string, error) {
	dir, err := session.DownloadsDir()
	if err != nil {
		return "", err
	}
	return h.SaveCSV(dir)
}

// saveCSV starts writing the whole kept history in the background; the
// outcome is shown when it arrives (savedCSV).
func (w *window) saveCSV(gtx layout.Context) {
	cs := &w.charts
	if cs.saving {
		return
	}
	if cs.saver == nil {
		cs.saver = saveToDownloads
	}
	if cs.done == nil {
		cs.done = make(chan saveResult, 1)
	}
	h, _ := w.env.Session.HistorySince(w.snap.Now.Add(-session.HistoryLength))
	cs.saving = true
	save, done, redraw := cs.saver, cs.done, w.env.Redraw
	go func() {
		path, err := save(h)
		done <- saveResult{path, err}
		if redraw != nil {
			redraw.Invalidate()
		}
	}()
}

// savedCSV shows a finished save's outcome.
func (w *window) savedCSV(gtx layout.Context) {
	cs := &w.charts
	select {
	case r := <-cs.done:
		cs.saving = false
		if r.err != nil {
			w.say(gtx, fmt.Errorf("saving the history: %w", r.err))
		} else {
			w.notice, w.noticeAt = "History saved to "+r.path, gtx.Now
		}
	default:
	}
}

// chartsPane: the window selector and CSV button, then the charts, the
// timeline and the time axis, scrolling.
func (w *window) chartsPane(gtx layout.Context) layout.Dimensions {
	cs := &w.charts
	m := w.chartsModelFor()
	// Redraw as time moves on, even with no frames arriving.
	gtx.Execute(op.InvalidateCmd{At: gtx.Now.Add(chartsRefresh)})
	rows := len(m.charts) + 2 // the charts, the timeline, the time axis
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			var ws []layout.Widget
			for i, cw := range chartWindows {
				kind := kindQuiet
				if i == cs.window {
					kind = kindActive
				}
				ws = append(ws, w.button(&cs.windows[i], kind, cw.label, true).small().Layout)
			}
			ws = append(ws, layout.Spacer{Width: 8}.Layout,
				w.button(&cs.save, kindNeutral, saveLabel(cs.saving), !cs.saving && (w.snap.CanControl || w.snap.HaveTelemetry)).small().Layout)
			if cs.hoverOn {
				ws = append(ws, oneLine(valueLabel(w.th, sizeLabel, w.pal.Muted, fmt.Sprintf("at %.1f s", cs.hoverT))).Layout)
			}
			return layout.Inset{Bottom: 8}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
				return flow(gtx, 6, 6, ws...)
			})
		}),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
			return material.List(w.th, &cs.list).Layout(gtx, rows, func(gtx layout.Context, i int) layout.Dimensions {
				return layout.Inset{Bottom: 8}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
					switch {
					case i < len(m.charts):
						return w.chart(gtx, i, m.charts[i], m)
					case i == len(m.charts):
						return w.timeline(gtx, m)
					}
					return w.timeAxis(gtx, m)
				})
			})
		}),
	)
}

func saveLabel(saving bool) string {
	if saving {
		return "Saving…"
	}
	return "Save CSV"
}

// valueText formats a value with the chart's decimals.
func valueText(v float64, decimals int, unit string) string {
	if math.IsNaN(v) {
		return "–"
	}
	return fmt.Sprintf("%.*f %s", decimals, v, unit)
}

// chart draws one chart: its title and legend (with the latest or hovered
// values), then the plot with its value axis.
func (w *window) chart(gtx layout.Context, i int, c chartSpec, m chartsModel) layout.Dimensions {
	cs := &w.charts
	return panel(gtx, w.pal.Surface, w.pal.Border, radiusCard, 8, func(gtx layout.Context) layout.Dimensions {
		return stacked(gtx, 4,
			func(gtx layout.Context) layout.Dimensions {
				ws := []layout.Widget{tracked(w.th, sizeLabel, w.pal.Fg, c.title+" · "+c.unit)}
				if c.missing == "" {
					for _, s := range c.series {
						v := math.NaN()
						if n := len(s.pts); n > 0 {
							v = s.pts[n-1].v
							if cs.hoverOn {
								v = valueAt(s.pts, cs.hoverT)
							}
						}
						label := valueText(v, c.decimals, c.unit)
						if len(c.series) > 1 {
							label = s.name + " " + label
						}
						ws = append(ws, sideBySideWidget(4, dot(w.pal.Series[s.slot], 8),
							oneLine(valueLabel(w.th, sizeLabel, w.pal.Fg, label)).Layout))
					}
				}
				return flow(gtx, 10, 2, ws...)
			},
			func(gtx layout.Context) layout.Dimensions {
				h := gtx.Dp(chartPlotHeight)
				if c.missing != "" {
					h = gtx.Dp(28)
				}
				size := image.Pt(gtx.Constraints.Max.X, h)
				gtx.Constraints = layout.Exact(size)
				if c.missing != "" {
					return valueLabel(w.th, sizeLabel, w.pal.Muted, c.missing).Layout(gtx)
				}
				w.plot(gtx, i, c, m, size)
				return layout.Dimensions{Size: size}
			},
		)
	})
}

// plot draws the gridlines with their values, the reference lines, the
// series and the hover rule; it registers the plot for hover.
func (w *window) plot(gtx layout.Context, i int, c chartSpec, m chartsModel, size image.Point) {
	ax := gtx.Dp(chartAxisWidth)
	pw := size.X - ax
	if pw <= 0 {
		return
	}
	t0 := -m.window
	lo, hi, step := c.valueRange(t0)
	x := func(t float64) float32 { return float32(ax) + float32((t-t0)/(0-t0))*float32(pw) }
	y := func(v float64) float32 { return float32(size.Y-1) - float32((v-lo)/(hi-lo))*float32(size.Y-2) }

	// Gridlines, recessive, with their values on the left.
	grid := faded(w.pal.Muted, 0x30)
	for g := math.Ceil(lo/step) * step; g <= hi; g += step {
		gy := y(g)
		strokePath(gtx, []f32.Point{{X: float32(ax), Y: gy}, {X: float32(size.X), Y: gy}}, 1, grid)
		lbl := valueLabel(w.th, 10, w.pal.Muted, trimZero(g, step))
		centredAt(gtx, f32.Pt(float32(ax)/2, gy), size, oneLine(lbl).Layout)
	}

	// Reference lines: dashed and muted; named after the series are drawn.
	for _, r := range c.refs {
		dashed(gtx, float32(ax), float32(size.X), y(r.v), faded(w.pal.Muted, 0xC0))
	}

	// The series: 2 px lines, broken at gaps (NaN), decimated per column.
	cl := clip.Rect{Min: image.Pt(ax, 0), Max: size}.Push(gtx.Ops)
	for _, s := range c.series {
		pts := decimate(s.pts, t0, 0, pw)
		var run []f32.Point
		for _, p := range pts {
			if p.t < t0-1 {
				continue
			}
			if math.IsNaN(p.v) {
				strokePath(gtx, run, 2, w.pal.Series[s.slot])
				run = run[:0]
				continue
			}
			run = append(run, f32.Pt(x(p.t), y(p.v)))
		}
		strokePath(gtx, run, 2, w.pal.Series[s.slot])
	}
	cl.Pop()

	// Reference names at the right end, just above their line (below it at
	// the top), on the surface so neither a series nor the line crosses them.
	var placed []image.Rectangle
	for _, r := range c.refs {
		placed = w.refName(gtx, r.name, int(y(r.v)), size, placed)
	}

	w.hoverLayer(gtx, i, ax, pw, size, x)
}

// refName draws a reference line's name right-aligned at line y: above the
// line, else below it, else left of the names already placed there, so
// close limits stay legible. It returns the placed rectangles.
func (w *window) refName(gtx layout.Context, name string, ly int, size image.Point, placed []image.Rectangle) []image.Rectangle {
	m := op.Record(gtx.Ops)
	g := gtx
	g.Constraints.Min = image.Point{}
	d := layout.Inset{Left: 3, Right: 3}.Layout(g, oneLine(valueLabel(w.th, 10, w.pal.Muted, name)).Layout)
	call := m.Stop()
	free := func(r image.Rectangle) bool {
		if r.Min.Y < 0 || r.Max.Y > size.Y {
			return false
		}
		return !overlapsAny(r, placed)
	}
	right := size.X - 2
	var r image.Rectangle
	for {
		above := image.Rect(right-d.Size.X, ly-d.Size.Y-1, right, ly-1)
		below := image.Rect(right-d.Size.X, ly+2, right, ly+2+d.Size.Y)
		if free(above) {
			r = above
			break
		}
		if free(below) {
			r = below
			break
		}
		right -= d.Size.X + 4
		if right-d.Size.X < 0 {
			return placed // no room: the line stays unnamed
		}
	}
	t := op.Offset(r.Min).Push(gtx.Ops)
	fill(gtx, image.Rectangle{Max: d.Size}, gtx.Dp(2), w.pal.Surface)
	call.Add(gtx.Ops)
	t.Pop()
	return append(placed, r)
}

// hoverLayer registers the plot area for pointer moves and draws the
// shared hover rule.
func (w *window) hoverLayer(gtx layout.Context, i, ax, pw int, size image.Point, x func(float64) float32) {
	cs := &w.charts
	cs.plotX[i] = [2]int{ax, pw}
	area := clip.Rect{Min: image.Pt(ax, 0), Max: size}.Push(gtx.Ops)
	event.Op(gtx.Ops, &cs.tags[i])
	area.Pop()
	if cs.hoverOn {
		hx := x(cs.hoverT)
		strokePath(gtx, []f32.Point{{X: hx, Y: 0}, {X: hx, Y: float32(size.Y)}}, 1, faded(w.pal.Fg, 0x90))
	}
}

// dashed draws a horizontal dashed rule.
func dashed(gtx layout.Context, x0, x1, y float32, c color.NRGBA) {
	const dash, gap = 6, 4
	for x := x0; x < x1; x += dash + gap {
		strokePath(gtx, []f32.Point{{X: x, Y: y}, {X: float32(math.Min(float64(x+dash), float64(x1))), Y: y}}, 1, c)
	}
}

// trimZero formats a gridline value with no more decimals than its step
// needs.
func trimZero(v, step float64) string {
	decimals := 0
	for s := step; s < 1 && decimals < 3; s *= 10 {
		decimals++
	}
	if math.Abs(v) < step/1e6 {
		v = 0
	}
	return fmt.Sprintf("%.*f", decimals, v)
}

// timeline draws the mode, output and fault rows, labelled where a span is
// wide enough, with the hover rule.
func (w *window) timeline(gtx layout.Context, m chartsModel) layout.Dimensions {
	tl := m.timeline
	return panel(gtx, w.pal.Surface, w.pal.Border, radiusCard, 8, func(gtx layout.Context) layout.Dimensions {
		// The state at the hovered moment, or the latest.
		moment := -1e-9
		if w.charts.hoverOn {
			moment = w.charts.hoverT
		}
		state := segmentAt(tl.modes, moment) + " · " + segmentAt(tl.output, moment) + " · " + segmentAt(tl.faults, moment)
		return stacked(gtx, 4,
			func(gtx layout.Context) layout.Dimensions {
				return flow(gtx, 10, 2, tracked(w.th, sizeLabel, w.pal.Fg, "Timeline · mode, output, faults"),
					oneLine(valueLabel(w.th, sizeLabel, w.pal.Fg, state)).Layout)
			},
			func(gtx layout.Context) layout.Dimensions {
				row := gtx.Dp(timelineRow)
				h := 3*row + 4
				if tl.missing != "" {
					h = gtx.Dp(28)
				}
				size := image.Pt(gtx.Constraints.Max.X, h)
				gtx.Constraints = layout.Exact(size)
				if tl.missing != "" {
					return valueLabel(w.th, sizeLabel, w.pal.Muted, tl.missing).Layout(gtx)
				}
				ax := gtx.Dp(chartAxisWidth)
				pw := size.X - ax
				t0 := -m.window
				x := func(t float64) float32 { return float32(ax) + float32((math.Max(t, t0)-t0)/(0-t0))*float32(pw) }
				rows := []struct {
					name string
					segs []segment
					fill color.NRGBA
				}{
					{"mode", tl.modes, w.pal.SurfaceAlt},
					{"output", tl.output, faded(w.pal.Green, 0xB0)},
					{"faults", tl.faults, w.pal.AccentText},
				}
				for r, rw := range rows {
					top := r * (row + 2)
					centredAt(gtx, f32.Pt(float32(ax)/2, float32(top+row/2)), size,
						oneLine(valueLabel(w.th, 10, w.pal.Muted, rw.name)).Layout)
					for k, s := range rw.segs {
						if s.t1 < t0 {
							continue
						}
						a, b := int(x(s.t0)), int(x(s.t1))
						if b <= a {
							b = a + 1
						}
						rect := image.Rect(a, top, b, top+row)
						fillCol := rw.fill
						if r == 0 && k%2 == 1 {
							fillCol = faded(w.pal.Muted, 0x40) // alternate modes stay apart
						}
						if r == 0 || s.on {
							fill(gtx, rect, gtx.Dp(2), fillCol)
						}
						if (r == 0 || s.on) && b-a > gtx.Dp(48) {
							txt := w.pal.Fg
							if r == 2 {
								txt = w.pal.OnAccent
							}
							at(gtx, image.Pt(a+4, top+1), oneLine(valueLabel(w.th, 10, txt, s.label)).Layout)
						}
					}
				}
				w.hoverLayer(gtx, chartCount, ax, pw, size, x)
				return layout.Dimensions{Size: size}
			},
		)
	})
}

// timeAxis labels the shared time axis under the charts.
func (w *window) timeAxis(gtx layout.Context, m chartsModel) layout.Dimensions {
	h := gtx.Dp(16)
	size := image.Pt(gtx.Constraints.Max.X, h)
	ax := gtx.Dp(chartAxisWidth) + gtx.Dp(8) // under the plots' left edge, inside the panels' inset
	pw := size.X - ax - gtx.Dp(8)
	ticks := 4
	for k := 0; k <= ticks; k++ {
		t := -m.window * float64(ticks-k) / float64(ticks)
		label := "now"
		if k < ticks {
			label = clockText(t)
		}
		centredAt(gtx, f32.Pt(float32(ax)+float32(pw)*float32(k)/float32(ticks), float32(h)/2), size,
			oneLine(valueLabel(w.th, 10, w.pal.Muted, label)).Layout)
	}
	return layout.Dimensions{Size: size}
}

// clockText is a negative offset in seconds as -m:ss or -s s.
func clockText(t float64) string {
	s := int(math.Round(-t))
	if s < 60 {
		return fmt.Sprintf("−%d s", s)
	}
	return fmt.Sprintf("−%d:%02d", s/60, s%60)
}
