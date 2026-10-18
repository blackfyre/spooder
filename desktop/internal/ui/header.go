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

// header: title, port picker, connection, control toggle and e-stop.
func (w *window) header(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	return layout.Flex{Alignment: layout.Middle}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			return stacked(gtx, 2,
				tracked(w.th, sizeTitle, w.pal.Fg, "Spooder"),
				tracked(w.th, sizeLabel, w.pal.Muted, "Calibrator"),
			)
		}),
		layout.Rigid(layout.Spacer{Width: 20}.Layout),
		layout.Flexed(1, w.connectionBlock),
		layout.Rigid(layout.Spacer{Width: 12}.Layout),
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			kind, label := kindNeutral, "Take control"
			if snap.Control {
				kind, label = kindActive, "Release control"
			}
			return w.button(&w.control, kind, label, snap.Control || snap.CanControl).width(150).Layout(gtx)
		}),
		layout.Rigid(layout.Spacer{Width: 12}.Layout),
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			label := "E-stop · space"
			if snap.EmergencyStop {
				label = "Release e-stop · space"
			}
			b := w.button(&w.estop, kindPrimary, label, true).width(190)
			b.size = 13
			return b.Layout(gtx)
		}),
	)
}

// connectionBlock: the ports, refresh, connect and the link state.
func (w *window) connectionBlock(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	return stacked(gtx, 6,
		func(gtx layout.Context) layout.Dimensions {
			label := "Connect"
			enabled := w.portSel >= 0
			if w.device != nil {
				label, enabled = "Disconnect", true
			}
			// The ports scroll sideways, so Refresh and Connect always fit.
			return layout.Flex{Alignment: layout.Middle}.Layout(gtx,
				layout.Rigid(tracked(w.th, sizeLabel, w.pal.Muted, "Port")),
				layout.Rigid(layout.Spacer{Width: 6}.Layout),
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
				layout.Rigid(w.button(&w.refresh, kindQuiet, "Refresh", w.device == nil).Layout),
				layout.Rigid(layout.Spacer{Width: 6}.Layout),
				layout.Rigid(w.button(&w.connect, kindNeutral, label, enabled).width(110).Layout),
			)
		},
		func(gtx layout.Context) layout.Dimensions {
			c := w.pal.Muted
			switch snap.Link.Status {
			case link.Connected:
				c = w.pal.Green
			case link.Waiting:
				c = w.pal.Yellow
			}
			// The message wraps: the permission hint must be readable at
			// any window width.
			return layout.Flex{}.Layout(gtx,
				layout.Rigid(func(gtx layout.Context) layout.Dimensions {
					return layout.Inset{Top: 3, Right: 8}.Layout(gtx, dot(c, 9))
				}),
				layout.Flexed(1, wrapped(valueLabel(w.th, sizeLabel, w.pal.Fg, connectionText(snap.Link)))),
			)
		},
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
					ws = append(ws, w.button(&w.modes[i], kind, m.String(), snap.ModesEnabled).width(110).Layout)
				}
				ws = append(ws, w.button(&w.clearFault, kindNeutral, "Clear fault", snap.ModesEnabled).width(120).Layout)
				state := "Observing: the app reports no controller."
				if snap.Control {
					state = "In control: requesting " + snap.Mode.String() + ". Translate and rotate stay zero."
				}
				if snap.HaveTelemetry && !snap.Fresh {
					ws = append(ws, w.staleMarker(snap))
				}
				ws = append(ws, textLabel(w.th, sizeText, w.pal.Muted, state).Layout)
				return sideBySide(gtx, 8, ws...)
			},
		)
	})
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
