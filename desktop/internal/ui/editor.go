package ui

import (
	"fmt"
	"image"
	"image/color"

	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// Nudge button labels per value field.
var nudgeLabels = [3][4]string{
	{"-5", "-.5", "+.5", "+5"},
	{"-50", "-10", "+10", "+50"},
	{"-50", "-10", "+10", "+50"},
}

// editor: the 18 joints as a 6 × 3 grid of cards, one row per leg.
func (w *window) editor(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			hint := editHint(snap)
			if hint == "" {
				hint = "The keys are listed above."
			}
			if snap.PendingCount > 0 {
				hint = fmt.Sprintf("%d change(s) awaiting the robot's confirmation. ", snap.PendingCount) + hint
			}
			return layout.Inset{Bottom: 8}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
				return sideBySide(gtx, 12,
					tracked(w.th, sizeLabel, w.pal.Fg, "Joint calibration"),
					oneLine(textLabel(w.th, sizeText, w.pal.Muted, hint)).Layout,
				)
			})
		}),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
			if !snap.HaveReport {
				return panel(gtx, w.pal.Surface, w.pal.Border, radiusPanel, 16, func(gtx layout.Context) layout.Dimensions {
					gtx.Constraints.Min.X = gtx.Constraints.Max.X
					return textLabel(w.th, sizeText, w.pal.Muted,
						"No calibration report yet. The robot reports its table in Calibrate (at least once per second) and after every change.").Layout(gtx)
				})
			}
			return material.List(w.th, &w.editorList).Layout(gtx, 6, func(gtx layout.Context, leg int) layout.Dimensions {
				return layout.Inset{Bottom: 8}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
					return layout.Flex{}.Layout(gtx,
						layout.Flexed(1, w.card(leg*3)),
						layout.Rigid(layout.Spacer{Width: 8}.Layout),
						layout.Flexed(1, w.card(leg*3+1)),
						layout.Rigid(layout.Spacer{Width: 8}.Layout),
						layout.Flexed(1, w.card(leg*3+2)),
					)
				})
			})
		}),
	)
}

// card is one joint: its reported values, nudges, and markers for a change
// awaiting confirmation and for the robot's last answer.
func (w *window) card(j int) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		snap := w.snap
		v := snap.Joints[j]
		editable := snap.Editable
		selected := w.sel.joint == j
		border := w.pal.Border
		if selected {
			border = w.pal.Accent
		}
		return w.cards[j].Layout(gtx, func(gtx layout.Context) layout.Dimensions {
			m := op.Record(gtx.Ops)
			dims := layout.UniformInset(10).Layout(gtx, func(gtx layout.Context) layout.Dimensions {
				gtx.Constraints.Min.X = gtx.Constraints.Max.X
				rows := []layout.Widget{w.cardTitle(j, v)}
				for f := calib.FieldOffset; f < calib.FieldCount; f++ {
					rows = append(rows, w.fieldRow(j, f, v, editable, selected && w.sel.field == f))
				}
				return stacked(gtx, 4, rows...)
			})
			call := m.Stop()
			r := image.Rectangle{Max: dims.Size}
			fill(gtx, r, gtx.Dp(radiusCard), w.pal.Surface)
			width := gtx.Dp(1)
			if selected {
				width = gtx.Dp(2)
			}
			outline(gtx, r, gtx.Dp(radiusCard), width, border)
			call.Add(gtx.Ops)
			return dims
		})
	}
}

func (w *window) cardTitle(j int, v jointView) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		ws := []layout.Widget{
			tracked(w.th, sizeLabel, w.pal.Fg, fmt.Sprintf("Leg %d · %s", calib.Leg(j), calib.Name(j))),
			valueLabel(w.th, sizeLabel, w.pal.Muted, fmt.Sprintf("out %d", j+1)).Layout,
		}
		if v.Pending {
			ws = append(ws, sideBySideWidget(4, dot(w.pal.Accent, 7), tracked(w.th, sizeLabel, w.pal.AccentText, "pending")))
		} else if r := resultText(v.Result); r != "" {
			ws = append(ws, w.chip(r, w.pal.Yellow))
		}
		return sideBySide(gtx, 8, ws...)
	}
}

// jointView is the session's view of one joint.
type jointView = session.JointView

func sideBySideWidget(gap unit.Dp, ws ...layout.Widget) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions { return sideBySide(gtx, gap, ws...) }
}

// chip is a small status label on a fill.
func (w *window) chip(label string, bg color.NRGBA) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		m := op.Record(gtx.Ops)
		dims := layout.Inset{Top: 2, Bottom: 2, Left: 6, Right: 6}.Layout(gtx, tracked(w.th, 10, w.pal.OnStatus, label))
		call := m.Stop()
		fill(gtx, image.Rectangle{Max: dims.Size}, gtx.Dp(4), bg)
		call.Add(gtx.Ops)
		return dims
	}
}

// fieldRow: label, reported value, the requested value while pending, and
// the nudges.
func (w *window) fieldRow(j int, f calib.Field, v jointView, editable, selected bool) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		m := op.Record(gtx.Ops)
		dims := layout.Inset{Top: 3, Bottom: 3, Left: 6, Right: 4}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints.Min.X = gtx.Constraints.Max.X
			// The value with its label, then the nudges on their own line,
			// so a card stays legible at any window width.
			return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
				layout.Rigid(func(gtx layout.Context) layout.Dimensions {
					gtx.Constraints.Min.X = gtx.Constraints.Max.X
					return layout.Flex{Alignment: layout.Baseline}.Layout(gtx,
						layout.Rigid(tracked(w.th, 10, w.pal.Muted, fieldLabel(j, f))),
						layout.Flexed(1, layout.Spacer{}.Layout),
						layout.Rigid(func(gtx layout.Context) layout.Dimensions {
							ws := []layout.Widget{valueLabel(w.th, sizeValue, w.pal.Fg, fieldText(j, v.Reported, f)).Layout}
							if v.Pending && fieldDiffers(v.Wanted, v.Reported, f) {
								ws = append(ws, valueLabel(w.th, sizeLabel, w.pal.AccentText, "→ "+fieldText(j, v.Wanted, f)).Layout)
							}
							return sideBySide(gtx, 6, ws...)
						}),
					)
				}),
				layout.Rigid(layout.Spacer{Height: 3}.Layout),
				layout.Rigid(func(gtx layout.Context) layout.Dimensions {
					if f == calib.FieldDirection {
						return w.button(&w.flips[j], kindQuiet, "flip", editable).small().width(60).Layout(gtx)
					}
					var ws []layout.Widget
					for b := range nudgeLabels[f] {
						ws = append(ws, w.button(&w.nudges[j][f][b], kindQuiet, nudgeLabels[f][b], editable).small().width(34).Layout)
					}
					return sideBySide(gtx, 3, ws...)
				}),
			)
		})
		call := m.Stop()
		if selected {
			fill(gtx, image.Rectangle{Max: dims.Size}, gtx.Dp(5), w.pal.SurfaceAlt)
		}
		call.Add(gtx.Ops)
		return dims
	}
}
