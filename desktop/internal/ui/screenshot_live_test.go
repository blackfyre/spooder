package ui

import (
	"image"
	"image/png"
	"os"
	"path/filepath"
	"sync/atomic"
	"testing"
	"time"

	"gioui.org/gpu/headless"
	"gioui.org/io/input"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
	"github.com/blackfyre/spooder/desktop/internal/virtualrobot"
)

// TestLiveScreenshots renders the window against the virtual robot, its
// session driven by the real link to the robot's terminal (--port's path):
// SPOODER_SCREENSHOTS=<dir> go test -run LiveScreenshots ./internal/ui. It
// is skipped without the directory or the virtual robot.
func TestLiveScreenshots(t *testing.T) {
	dir := os.Getenv("SPOODER_SCREENSHOTS")
	if dir == "" {
		t.Skip("set SPOODER_SCREENSHOTS to a directory")
	}
	bin, ok := virtualrobot.Binary()
	if !ok {
		t.Skip("virtual robot not built (mise run virtual-robot):", bin)
	}
	const width, height = 1360, 900
	for _, dark := range []bool{false, true} {
		theme := "light"
		if dark {
			theme = "dark"
		}
		// The GPU context first, so capturing a frame later is quick (the
		// pending change is caught within a command period).
		hw, err := headless.NewWindow(width, height)
		if err != nil {
			t.Skip("no headless GPU:", err)
		}
		t.Cleanup(hw.Release)
		vr, err := virtualrobot.Start(bin, filepath.Join(t.TempDir(), "usb"))
		if err != nil {
			t.Fatal(err)
		}
		t.Cleanup(vr.Close)
		s := session.New(nil, nil)
		sys := vr.System()
		l := link.New(sys, s, s, link.Options{})
		t.Cleanup(func() { s.Shutdown(); l.Close() }) // runs before the robot's: last registered, first run
		w, err := newWindow(Env{Session: s, Link: l, Ports: sys.List, Port: vr.Link,
			Checkouts: []string{checkoutDir(t)}, Appearance: appearance.Settings{Dark: dark}})
		if err != nil {
			t.Fatal(err)
		}
		ports, _ := sys.List()
		w.setPorts(ports)
		router := new(input.Router)
		frame(w, router)

		waitFor := func(what string, cond func(session.Snapshot) bool) {
			t.Helper()
			deadline := time.Now().Add(15 * time.Second)
			for !cond(s.Snapshot()) {
				if time.Now().After(deadline) {
					t.Fatalf("%s: timed out waiting for %s", theme, what)
				}
				time.Sleep(10 * time.Millisecond)
			}
			frame(w, router)
		}
		save := func(name string, img image.Image) {
			t.Helper()
			f, err := os.Create(filepath.Join(dir, "live-"+theme+"-"+name+".png"))
			if err != nil {
				t.Fatal(err)
			}
			defer f.Close()
			if err := png.Encode(f, img); err != nil {
				t.Fatal(err)
			}
		}
		shoot := func(name string) {
			t.Helper()
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
			save(name, img)
		}
		// The four-view window, on the same session.
		viewsEnv := w.env
		viewsEnv.Selection = new(atomic.Int32)
		views, err := newViewsWindow(viewsEnv)
		if err != nil {
			t.Fatal(err)
		}
		shootViews := func(name string) { // on the same GPU context, at the same size
			t.Helper()
			var ops op.Ops
			gtx := layout.Context{Ops: &ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
				Constraints: layout.Exact(image.Pt(width, height)), Now: nextFrameTime(), Source: new(input.Router).Source()}
			views.frame(gtx)
			if err := hw.Frame(&ops); err != nil {
				t.Fatal(err)
			}
			img := image.NewRGBA(image.Rectangle{Max: image.Pt(width, height)})
			if err := hw.Screenshot(img); err != nil {
				t.Fatal(err)
			}
			save(name, img)
		}

		// Connected and observing: the preselected --port, Connect.
		if w.portSel < 0 || w.ports[w.portSel].Name != vr.Link {
			t.Fatalf("the robot's port is not preselected: %v", w.ports)
		}
		click(w, router, &w.connect)
		waitFor("telemetry and the status line", func(s session.Snapshot) bool {
			return s.Fresh && len(s.Log) > 0 && s.Telemetry.VoltageMv > 0
		})
		time.Sleep(1200 * time.Millisecond) // a status line with the supply settled
		shoot("observing")

		// In control, walking forward from the keyboard: the legs pane, the
		// charts and the four views.
		click(w, router, &w.control)
		click(w, router, &w.modes[0]) // Stand
		waitFor("Stand", func(s session.Snapshot) bool {
			return s.Body == session.BodyStand && s.Telemetry.OutputsOn
		})
		time.Sleep(2 * time.Second) // settled
		if err := s.SetDrive(session.Drive{Forward: true}); err != nil {
			t.Fatal(err)
		}
		waitFor("Walk", func(s session.Snapshot) bool { return s.Body == session.BodyWalk })
		time.Sleep(4 * time.Second) // into the stride, with history to chart
		w.tab = tabLegs
		frame(w, router)
		shoot("walking-legs")
		w.tab = tabCharts
		frame(w, router)
		shoot("walking-charts")
		shootViews("walking-views")
		if err := s.SetDrive(session.Drive{}); err != nil {
			t.Fatal(err)
		}
		w.tab = tabLog

		// Calibrate, a change awaiting the robot's confirmation.
		click(w, router, &w.modes[1]) // Calibrate
		waitFor("Calibrate", func(s session.Snapshot) bool {
			return s.Body == session.BodyCalibrate && s.Editable && s.Telemetry.OutputsOn
		})
		// The legs pane with the robot's live pose in the calibration pose.
		time.Sleep(3 * time.Second) // the legs reach the calibration pose
		w.tab = tabLegs
		waitFor("a live pose with output on", func(s session.Snapshot) bool {
			return s.HavePose && s.PoseFresh && s.Pose.Flags&protocol.PoseOutputsOn != 0
		})
		shoot("calibrate-legs")
		w.tab = tabLog

		w.sel = selection{joint: 8, field: calib.FieldPulse0}
		pending := false
		for try := 0; try < 20 && !pending; try++ {
			time.Sleep(300 * time.Millisecond) // the previous change confirmed
			if err := s.Nudge(8, calib.FieldPulse0, 1, false); err != nil {
				t.Fatal(err)
			}
			shoot("calibrate-pending") // within a command period: still unconfirmed
			pending = w.snap.Joints[8].Pending
		}
		if !pending {
			t.Fatal("never caught the change pending")
		}

		// E-stop.
		s.ToggleEmergencyStop()
		waitFor("the e-stop latched", func(s session.Snapshot) bool {
			return s.Telemetry.Faults&session.FaultEmergencyStop != 0 && !s.Telemetry.OutputsOn
		})
		time.Sleep(1200 * time.Millisecond)
		shoot("estop")
		w.tab = tabLegs
		frame(w, router)
		shoot("estop-legs")

	}
}
