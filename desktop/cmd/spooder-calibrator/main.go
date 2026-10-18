// Command spooder-calibrator is the desktop front-end for bench calibration
// and diagnostics of the Spooder robot on its USB port.
package main

import (
	"os"

	"gioui.org/app"
	"gioui.org/op"
	"gioui.org/unit"
)

func main() {
	go func() {
		w := new(app.Window)
		w.Option(app.Title("Spooder calibrator"), app.Size(unit.Dp(1200), unit.Dp(800)))
		var ops op.Ops
		for {
			switch e := w.Event().(type) {
			case app.DestroyEvent:
				if e.Err != nil {
					os.Exit(1)
				}
				os.Exit(0)
			case app.FrameEvent:
				ops.Reset()
				e.Frame(&ops)
			}
		}
	}()
	app.Main()
}
