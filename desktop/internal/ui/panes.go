package ui

import (
	"fmt"
	"path/filepath"

	"gioui.org/layout"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/export"
)

// sidePanes: the debug log, the decoded frames and the export, as tabs.
func (w *window) sidePanes(gtx layout.Context) layout.Dimensions {
	return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
		layout.Rigid(func(gtx layout.Context) layout.Dimensions {
			var ws []layout.Widget
			for i := range w.tabs {
				kind := kindQuiet
				if i == w.tab {
					kind = kindActive
				}
				ws = append(ws, w.button(&w.tabs[i], kind, tabNames[i], true).Layout)
			}
			return layout.Inset{Bottom: 8}.Layout(gtx, func(gtx layout.Context) layout.Dimensions {
				return sideBySide(gtx, 6, ws...)
			})
		}),
		layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
			gtx.Constraints.Min = gtx.Constraints.Max
			switch w.tab {
			case tabFrames:
				return w.framesPane(gtx)
			case tabExport:
				return w.exportPane(gtx)
			}
			return w.logPane(gtx)
		}),
	)
}

func (w *window) logPane(gtx layout.Context) layout.Dimensions {
	log := w.snap.Log
	return panel(gtx, w.pal.Well, w.pal.Border, 10, func(gtx layout.Context) layout.Dimensions {
		gtx.Constraints.Min = gtx.Constraints.Max
		if len(log) == 0 {
			return valueLabel(w.th, sizeLabel, w.pal.Muted, "The robot's debug text appears here.").Layout(gtx)
		}
		return material.List(w.th, &w.logList).Layout(gtx, len(log), func(gtx layout.Context, i int) layout.Dimensions {
			l := log[i]
			c := w.pal.Fg
			text := l.At.Format("15:04:05.000") + "  " + l.Text
			if l.Stray > 0 {
				c = w.pal.AccentText
				text += fmt.Sprintf("   [%d stray 0xA5]", l.Stray)
			}
			lbl := valueLabel(w.th, 12, c, text)
			lbl.MaxLines = 0
			return lbl.Layout(gtx)
		})
	})
}

func (w *window) framesPane(gtx layout.Context) layout.Dimensions {
	frames := w.snap.Frames
	return panel(gtx, w.pal.Well, w.pal.Border, 10, func(gtx layout.Context) layout.Dimensions {
		gtx.Constraints.Min = gtx.Constraints.Max
		if len(frames) == 0 {
			return valueLabel(w.th, sizeLabel, w.pal.Muted, "Decoded frames appear here.").Layout(gtx)
		}
		return material.List(w.th, &w.framesList).Layout(gtx, len(frames), func(gtx layout.Context, i int) layout.Dimensions {
			f := frames[i]
			lbl := valueLabel(w.th, 12, w.pal.Fg, f.At.Format("15:04:05.000")+"  "+f.Text)
			lbl.MaxLines = 0
			return lbl.Layout(gtx)
		})
	})
}

// exportPane: the checkout, the difference, write and copy.
func (w *window) exportPane(gtx layout.Context) layout.Dimensions {
	snap := w.snap
	table := snap.Table
	return panel(gtx, w.pal.Surface, w.pal.Border, 12, func(gtx layout.Context) layout.Dimensions {
		gtx.Constraints.Min = gtx.Constraints.Max
		return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
			layout.Rigid(func(gtx layout.Context) layout.Dimensions {
				rows := []layout.Widget{
					tracked(w.th, sizeLabel, w.pal.Fg, "Export to "+filepath.ToSlash(export.DefaultsPath)),
				}
				if len(w.exp.candidates) == 0 {
					rows = append(rows, textLabel(w.th, sizeText, w.pal.Muted,
						"No checkout known: start the app in a spooder checkout or pass --checkout DIR.").Layout)
				}
				for i, dir := range w.exp.candidates {
					kind := kindQuiet
					if i == w.exp.chosen {
						kind = kindActive
					}
					status := "spooder checkout"
					if err := export.Check(dir); err != nil {
						status = err.Error()
					}
					rows = append(rows, sideBySideWidget(8,
						w.button(&w.checkoutClicks[i], kind, "use", true).Layout,
						oneLine(valueLabel(w.th, sizeLabel, w.pal.Fg, dir)).Layout,
						oneLine(textLabel(w.th, 12, w.pal.Muted, status)).Layout,
					))
				}
				if snap.PendingCount > 0 {
					rows = append(rows, textLabel(w.th, sizeText, w.pal.AccentText,
						fmt.Sprintf("%d change(s) are not confirmed yet; the export holds the values the robot reported.", snap.PendingCount)).Layout)
				}
				if !snap.HaveReport {
					rows = append(rows, textLabel(w.th, sizeText, w.pal.Muted, "Exports need a calibration report from the robot.").Layout)
				}
				planned := w.exp.plan != nil
				buttons := []layout.Widget{
					w.button(&w.prepare, kindNeutral, "Show difference", snap.HaveReport).Layout,
					w.button(&w.write, kindPrimary, "Write file", w.exp.canWrite(table)).Layout,
					w.button(&w.cancel, kindQuiet, "Cancel", planned).Layout,
					w.button(&w.copyText, kindQuiet, "Copy text", snap.HaveReport).Layout,
				}
				if !w.copied.IsZero() && gtx.Now.Sub(w.copied) < noticeFor {
					buttons = append(buttons, valueLabel(w.th, sizeLabel, w.pal.Muted, "copied").Layout)
				}
				rows = append(rows, sideBySideWidget(6, buttons...))
				status, c := w.exp.status, w.pal.Muted
				if w.exp.failed {
					c = w.pal.AccentText
				}
				if w.exp.stale(table) {
					status, c = "The robot's table changed since the difference was made: show it again.", w.pal.AccentText
				}
				if status != "" {
					rows = append(rows, textLabel(w.th, sizeText, c, status).Layout)
				}
				return stacked(gtx, 8, rows...)
			}),
			layout.Rigid(layout.Spacer{Height: 10}.Layout),
			layout.Flexed(1, w.diffView),
		)
	})
}

// diffView shows the plan's difference: every line, removed ones marked -
// and added ones + in the accent.
func (w *window) diffView(gtx layout.Context) layout.Dimensions {
	plan := w.exp.plan
	return panel(gtx, w.pal.Well, w.pal.Border, 8, func(gtx layout.Context) layout.Dimensions {
		gtx.Constraints.Min = gtx.Constraints.Max
		if plan == nil {
			return valueLabel(w.th, sizeLabel, w.pal.Muted, "The difference from the file on disk appears here.").Layout(gtx)
		}
		lines := plan.Diff
		return material.List(w.th, &w.diffList).Layout(gtx, len(lines), func(gtx layout.Context, i int) layout.Dimensions {
			l := lines[i]
			c := w.pal.Muted
			switch l.Kind {
			case export.Added:
				c = w.pal.AccentText
			case export.Removed:
				c = w.pal.Fg
			}
			lbl := valueLabel(w.th, 12, c, string(l.Kind)+" "+l.Text)
			return lbl.Layout(gtx)
		})
	})
}
