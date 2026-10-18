package ui

import (
	"image"
	"image/png"
	"os"
	"path/filepath"
	"testing"
	"time"

	"gioui.org/gpu/headless"
	"gioui.org/io/input"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// TestScreenshots renders the window off screen into PNG files, for a look
// without a display: SPOODER_SCREENSHOTS=<dir> go test -run Screenshots
// ./internal/ui. It is skipped otherwise.
func TestScreenshots(t *testing.T) {
	dir := os.Getenv("SPOODER_SCREENSHOTS")
	if dir == "" {
		t.Skip("set SPOODER_SCREENSHOTS to a directory")
	}
	const width, height = 1360, 900
	for _, dark := range []bool{false, true} {
		w, router, _ := testWindow(t, dark)
		shoot := func(name string) {
			hw, err := headless.NewWindow(width, height)
			if err != nil {
				t.Skip("no headless GPU:", err)
			}
			defer hw.Release()
			var ops op.Ops
			gtx := layout.Context{Ops: &ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
				Constraints: layout.Exact(image.Pt(width, height)), Now: nextFrameTime(), Source: router.Source()}
			w.frame(gtx)
			router.Frame(&ops)
			if err := hw.Frame(&ops); err != nil {
				t.Fatal(err)
			}
			img := image.NewRGBA(image.Rectangle{Max: image.Pt(width, height)})
			if err := hw.Screenshot(img); err != nil {
				t.Fatal(err)
			}
			theme := "light"
			if dark {
				theme = "dark"
			}
			f, err := os.Create(filepath.Join(dir, theme+"-"+name+".png"))
			if err != nil {
				t.Fatal(err)
			}
			defer f.Close()
			if err := png.Encode(f, img); err != nil {
				t.Fatal(err)
			}
		}
		shoot("disconnected")
		calibrating(t, w, router)
		s := w.env.Session
		s.Line(link.Line{Text: "spooder: USB link up, source usb"}, link.Stats{})
		s.Line(link.Line{Text: `volt\xA5age`, Stray: 1}, link.Stats{Stray: 1})
		_ = s.Nudge(8, calib.FieldPulse0, 1, false)
		r := protocol.CalibrationReport{LastRequestID: 0, Result: uint8(protocol.ResultNoneYet)}
		for i := range r.Joints {
			r.Joints[i] = protocol.CalibrationJoint{OffsetCentideg: 9000, Pulse0Us: 500, Pulse180Us: 2500}
		}
		w.sel = selection{joint: 8, field: calib.FieldPulse0}
		frame(w, router)
		shoot("calibrate")
		s.ToggleEmergencyStop()
		w.tab = tabExport
		frame(w, router)
		click(w, router, &w.prepare)
		shoot("export-estop")
		s.ToggleEmergencyStop()

		// The legs pane: calibration pose, walking, a joint at its limit,
		// and no pose.
		w.tab = tabLegs
		w.sel = selection{joint: 7, field: calib.FieldOffset} // leg 2 femur
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(calibrationAngles(), true)}, link.Stats{})
		frame(w, router)
		shoot("legs-calibration")
		w.sel = selection{joint: 14} // leg 4 tibia
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(walkingAngles, true)}, link.Stats{})
		frame(w, router)
		shoot("legs-walking")
		w.sel = selection{joint: 6} // leg 2 coxa
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(limitAngles, false)}, link.Stats{})
		frame(w, router)
		shoot("legs-limit-output-off")
		clock = clock.Add(2 * time.Second)
		frame(w, router)
		shoot("legs-no-pose")
		_ = input.Router{}
	}
}
