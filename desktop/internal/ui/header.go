package ui

import (
	"fmt"
	"image"
	"image/color"

	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// header: the title, then one row of controls (the ports, refresh,
// connect, the control toggle and the e-stop, all the same height and
// centred on one line) with the link state beneath it.
func (w *window) header(gtx layout.Context) layout.Dimensions {
	return layout.Flex{Alignment: layout.Start}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			return stacked(gtx, 2,
				tracked(w.th, sizeTitle, w.pal.Fg, "Spooder"),
				tracked(w.th, sizeLabel, w.pal.Muted, "Calibrator"),
			)
		}),
		layout.Rigid(layout.Spacer{Width: 20}.Layout),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
			return stacked(gtx, 6, w.controlRow, w.linkState)
		}),
	)
}

// controlRow: the ports (scrolling sideways, so the buttons always fit),
// refresh and connect, then the control toggle and the e-stop.
func (w *window) controlRow(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	label := "Connect"
	enabled := w.portSel >= 0
	if w.device != nil {
		label, enabled = "Disconnect", true
	}
	return layout.Flex{Alignment: layout.Middle}.Layout(gtx,
		layout.Rigid(tracked(w.th, sizeLabel, w.pal.Muted, "Port")),
		layout.Rigid(layout.Spacer{Width: 8}.Layout),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
			if len(w.ports) == 0 {
				return valueLabel(w.th, sizeLabel, w.pal.Muted, "no serial ports").Layout(gtx)
			}
			return w.portList.Layout(gtx, len(w.ports), func(gtx layout.Context, i int) layout.Dimensions {
				p := w.ports[i]
				kind := kindQuiet
				if i == w.portSel {
					kind = kindActive
				}
				label := p.Name
				if p.IsServo2040() {
					label += " servo 2040"
				}
				return layout.Inset{Right: 6}.Layout(gtx, w.button(&w.portClicks[i], kind, label, w.device == nil).Layout)
			})
		}),
		layout.Rigid(layout.Spacer{Width: 12}.Layout),
		layout.Rigid(w.button(&w.refresh, kindQuiet, "Refresh", w.device == nil).width(100).Layout),
		layout.Rigid(layout.Spacer{Width: 8}.Layout),
		layout.Rigid(w.button(&w.connect, kindNeutral, label, enabled).width(120).Layout),
		layout.Rigid(layout.Spacer{Width: 24}.Layout),
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			kind, label := kindNeutral, "Take control"
			if snap.Control {
				kind, label = kindActive, "Release control"
			}
			return w.button(&w.control, kind, label, snap.Control || snap.CanControl).width(160).Layout(gtx)
		}),
		layout.Rigid(layout.Spacer{Width: 8}.Layout),
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			label := "E-stop · space"
			if snap.EmergencyStop {
				label = "Release e-stop · space"
			}
			return w.button(&w.estop, kindPrimary, label, true).width(200).Layout(gtx)
		}),
	)
}

// linkState: the connection's state; the message wraps, so the permission
// hint is readable at any window width.
func (w *window) linkState(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	c := w.pal.Muted
	switch snap.Link.Status {
	case link.Connected:
		c = w.pal.Green
	case link.Waiting:
		c = w.pal.Yellow
	}
	return layout.Flex{}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			return layout.Inset{Top: 3, Right: 8}.Layout(gtx, dot(c, 9))
		}),
		layout.Flexed(1, wrapped(valueLabel(w.th, sizeLabel, w.pal.Fg, connectionText(snap.Link)))),
	)
}

// banners: the e-stop, a source conflict, a refused action.
func (w *window) banners(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	var ws []layout.Widget
	banner := func(bg, fg color.NRGBA, title, body string) layout.Widget {
		return func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints.Min.X = gtx.Constraints.Max.X
			return panel(gtx, bg, color.NRGBA{}, 10, func(gtx layout.Context) layout.Dimensions {
				return sideBySide(gtx, 14,
					tracked(w.th, 13, fg, title),
					textLabel(w.th, sizeText, fg, body).Layout,
				)
			})
		}
	}
	if snap.EmergencyStop {
		ws = append(ws, banner(w.pal.Accent, w.pal.OnAccent, "Emergency stop",
			"The app signals an emergency stop. Releasing it does not clear the robot's fault: take control and use Clear fault."))
	}
	if snap.HaveTelemetry && snap.Source == session.SourceConflict {
		ws = append(ws, banner(w.pal.Yellow, w.pal.OnStatus, "Source conflict",
			"The bridge and this app both claim control, so the robot sleeps. Release control here or disconnect the gamepad."))
	}
	if snap.StrayLines > 0 {
		ws = append(ws, banner(w.pal.Yellow, w.pal.OnStatus, "Stray 0xA5",
			fmt.Sprintf("Debug lines with a byte 0xA5 outside a frame: %d. The robot's text must be ASCII (see the log).", snap.StrayLines)))
	}
	if w.notice != "" && gtx.Now.Sub(w.noticeAt) < noticeFor {
		ws = append(ws, banner(w.pal.SurfaceAlt, w.pal.Fg, "Not done", w.notice))
	}
	if len(ws) == 0 {
		return layout.Dimensions{}
	}
	return layout.Inset{Bottom: 12}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
		return stacked(gtx, 8, ws...)
	})
}

// cell is one labelled telemetry value, weight wide.
func (w *window) cell(label, value string, c color.NRGBA) layout.FlexChild {
	return w.wideCell(1, label, value, c)
}

func (w *window) wideCell(weight float32, label, value string, c color.NRGBA) layout.FlexChild {
	return layout.Flexed(weight, func(gtx layout.Context) layout.Dimensions {
		return stacked(gtx, 4,
			tracked(w.th, sizeLabel, w.pal.Muted, label),
			oneLine(valueLabel(w.th, sizeValue, c, value)).Layout,
		)
	})
}

// telemetryStrip: the robot's state from the latest telemetry, and the
// mode controls.
func (w *window) telemetryStrip(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	gtx.Constraints.Min.X = gtx.Constraints.Max.X
	return panel(gtx, w.pal.Surface, w.pal.Border, 14, func(gtx layout.Context) layout.Dimensions {
		return stacked(gtx, 12,
			func(gtx layout.Context) layout.Dimensions {
				if !snap.HaveTelemetry {
					return textLabel(w.th, sizeText, w.pal.Muted, "No telemetry yet.").Layout(gtx)
				}
				t := snap.Telemetry
				c := w.pal.Fg
				if !snap.Fresh {
					c = w.pal.Muted
				}
				faultColour := c
				if len(snap.Faults) > 0 && snap.Fresh {
					faultColour = w.pal.AccentText
				}
				return layout.Flex{}.Layout(gtx,
					w.cell("Mode", snap.Body.String(), c),
					w.cell("Gait", session.GaitName(t.Gait), c),
					w.cell("Output", onOffText(t.OutputsOn), c),
					w.cell("Supply", voltsText(t.VoltageMv), c),
					w.cell("Current", ampsText(t.CurrentMa), c),
					w.wideCell(1.6, "Source", sourceText(snap), c),
					layout.Flexed(2, func(gtx layout.Context) layout.Dimensions {
						return stacked(gtx, 4,
							tracked(w.th, sizeLabel, w.pal.Muted, "Faults"),
							wrapped(valueLabel(w.th, sizeValue, faultColour, faultsText(snap.Faults))),
						)
					}),
					w.wideCell(1.3, "Robot link", fmt.Sprintf("%d ok · %d bad", t.ValidFrames, t.DiscardedFrames), c),
					w.wideCell(1.3, "Host link", fmt.Sprintf("%d ok · %d bad", snap.Stats.Valid, snap.Stats.Discarded), w.pal.Fg),
				)
			},
			func(gtx layout.Context) layout.Dimensions {
				var ws []layout.Widget
				for i, m := range modeButtons {
					kind := kindNeutral
					if snap.Control && snap.Mode == m {
						kind = kindActive
					}
					ws = append(ws, w.button(&w.modes[i], kind, m.String()+" · "+string(modeKeys[i]), snap.ModesEnabled).width(130).Layout)
				}
				ws = append(ws, w.button(&w.clearFault, kindNeutral, "Clear fault · "+string(clearFaultKey), snap.ModesEnabled).width(150).Layout)
				state := "Observing: the app reports no controller. Take control to use these."
				if snap.Control {
					state = "In control: requesting " + snap.Mode.String() + ". Walking is the gamepad's."
				}
				if snap.HaveTelemetry && !snap.Fresh {
					ws = append(ws, w.staleMarker(snap))
				}
				ws = append(ws, textLabel(w.th, sizeText, w.pal.Muted, state).Layout)
				return sideBySide(gtx, 8, ws...)
			},
			func(gtx layout.Context) layout.Dimensions {
				if !snap.Control {
					return layout.Dimensions{}
				}
				return w.keyHints(gtx)
			},
		)
	})
}

// keyHint is one shortcut: its keys and what they do.
type keyHint struct {
	keys   []string
	action string
}

// keyHints are the shortcuts while in control. Space works in any state;
// the editing keys act only in Calibrate.
var keyHints = []keyHint{
	{[]string{"Space"}, "e-stop on / off"},
	{[]string{"1", "2", "3"}, "Stand, Calibrate, Sleep"},
	{[]string{"F"}, "clear fault"},
	{[]string{"←", "→", "↑", "↓"}, "select joint and value"},
	{[]string{"=", "−"}, "nudge, with Shift coarse"},
	{[]string{"R"}, "flip direction"},
}

// keyHints: the keyboard controls, as key caps and their actions, flowing
// onto further lines in a narrow window.
func (w *window) keyHints(gtx layout.Context) layout.Dimensions {
	ws := []layout.Widget{tracked(w.th, sizeLabel, w.pal.Muted, "Keys")}
	for _, h := range keyHints {
		parts := make([]layout.Widget, 0, len(h.keys)+1)
		for _, k := range h.keys {
			parts = append(parts, w.keyCap(k))
		}
		parts = append(parts, oneLine(textLabel(w.th, sizeText, w.pal.Fg, h.action)).Layout)
		ws = append(ws, sideBySideWidget(4, parts...))
	}
	ws = append(ws, oneLine(textLabel(w.th, sizeText, w.pal.Muted, "Editing keys act in Calibrate.")).Layout)
	return flow(gtx, 18, 6, ws...)
}

// keyCap is a key's name in an outlined cap.
func (w *window) keyCap(name string) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		m := op.Record(gtx.Ops)
		dims := layout.Inset{Top: 2, Bottom: 2, Left: 6, Right: 6}.Layout(gtx, valueLabel(w.th, sizeLabel, w.pal.Fg, name).Layout)
		call := m.Stop()
		r := image.Rectangle{Max: dims.Size}
		fill(gtx, r, gtx.Dp(4), w.pal.SurfaceAlt)
		outline(gtx, r, gtx.Dp(4), gtx.Dp(1), w.pal.Border)
		call.Add(gtx.Ops)
		return dims
	}
}

// staleMarker flags values that are not current.
func (w *window) staleMarker(snap session.Snapshot) layout.Widget {
	return func(gtx layout.Context) layout.Dimensions {
		m := op.Record(gtx.Ops)
		dims := layout.Inset{Top: 4, Bottom: 4, Left: 8, Right: 8}.Layout(gtx,
			tracked(w.th, sizeLabel, w.pal.OnStatus, "Not current · "+ageText(snap.TelemetryAge)))
		call := m.Stop()
		fill(gtx, image.Rectangle{Max: dims.Size}, gtx.Dp(unit.Dp(4)), w.pal.Yellow)
		call.Add(gtx.Ops)
		return dims
	}
}
