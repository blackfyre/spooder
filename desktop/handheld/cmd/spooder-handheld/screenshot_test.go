package main

import (
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/veandco/go-sdl2/img"
	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/internal/legview"
)

// walkingPose is a standing pose with the legs swung as in a stride, one
// femur near its servo limit and one tibia at it.
func walkingPose() app.Pose {
	p := legview.Pose{OutputsOn: true}
	for leg := range p.Legs {
		coxa := []float64{15, -10, 12, -15, 10, -12}[leg]
		a := legview.Angles{Coxa: coxa, Femur: 17, Tibia: 95}
		l := legview.Leg{Set: true, Angles: a}
		for j, deg := range []float64{a.Coxa + 90, a.Femur + 90, a.Tibia} {
			l.Joints[j] = legview.Joint{Commanded: deg, Servo: deg, PulseUs: 1500, Limit: legview.Classify(deg)}
		}
		p.Legs[leg] = l
	}
	p.Legs[1].Joints[1].Limit = legview.LimitNear
	p.Legs[4].Joints[2].Limit = legview.LimitAt
	return app.Pose{Have: true, Pose: p, Legs: legview.Body3(p)}
}

// surfaceWindow is a window drawn by a software renderer into a surface.
type surfaceWindow struct{ w, h int32 }

func (s surfaceWindow) GetWidth() int32  { return s.w }
func (s surfaceWindow) GetHeight() int32 { return s.h }
func (surfaceWindow) Present()           {}

// The drive screen's states, for screenshots and the drawing smoke test.
var screenshotCases = []struct {
	name string
	sc   app.Screen
}{
	{"driving", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, Gesture: "", VoltageMv: 7840, CurrentMa: 3420, Source: "bridge"},
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
	}},
	{"fault", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Sleep", Gait: "Tripod", Faults: []string{"emergency stop", "undervoltage"}, VoltageMv: 50, Source: "none"},
		Requested: app.Requested{Mode: "Sleep", Height: 0, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer", EStop: true},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, Lost: 3, HaveFrame: true, SinceLast: 60 * time.Millisecond, SignalKnown: true, SignalDBm: -71, Sent: 402},
		State:     app.Driving,
		Held:      padmap.HandSelect | padmap.HandY,
		Clearing:  0.6,
	}},
	{"stale", app.Screen{
		Robot:     app.Robot{Have: true, Stale: true, Body: "Stand", Gait: "Ripple", OutputsOn: true, VoltageMv: 7600, CurrentMa: 1200, Source: "bridge"},
		Requested: app.Requested{Mode: "Stand", Height: 80, Speed: 44, Gait: "Ripple", Stride: "Fixed", StickMode: "Crab"},
		Link:      app.LinkView{HaveFrame: true, SinceLast: 2300 * time.Millisecond, Sent: 9120, SendErrors: 4},
		State:     app.Driving,
		Notice:    "send: network is unreachable",
	}},
	{"leaving", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, VoltageMv: 7810, CurrentMa: 2900, Source: "bridge"},
		Requested: app.Requested{Mode: "Sleep", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 20 * time.Millisecond, SignalKnown: true, SignalDBm: -40, Sent: 30111},
		State:     app.Leaving,
		Leave:     app.ActionExit,
	}},
	{"driveview", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, Faults: []string{"undervoltage"}, VoltageMv: 7840, CurrentMa: 3420, Source: "bridge"},
		Pose:      walkingPose(),
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
		Held:      padmap.HandSelect | padmap.HandX,
		DriveView: true,
	}},
	{"driveview-nopose", app.Screen{
		Requested: app.Requested{Mode: "Sleep", Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		State:     app.Driving,
		DriveView: true,
	}},
	{"simulation", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, VoltageMv: 6000, CurrentMa: 2160, Source: "bridge", Simulated: true},
		Pose:      walkingPose(),
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
	}},
	{"waiting", app.Screen{
		Requested: app.Requested{Mode: "Sleep", Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		State:     app.Driving,
	}},
}

// TestDrawsEveryState draws each state on a software renderer at the
// device's size. With SPOODER_SCREENSHOTS set to a directory, it also saves
// each as a PNG there.
func TestDrawsEveryState(t *testing.T) {
	if err := img.Init(img.INIT_PNG); err != nil {
		t.Fatal(err)
	}
	defer img.Quit()
	f := testFonts(t)
	dir := os.Getenv("SPOODER_SCREENSHOTS")
	for _, c := range screenshotCases {
		surf, err := sdl.CreateRGBSurfaceWithFormat(0, 1280, 720, 32, sdl.PIXELFORMAT_ARGB8888)
		if err != nil {
			t.Fatal(err)
		}
		r, err := sdl.CreateSoftwareRenderer(surf)
		if err != nil {
			surf.Free()
			t.Fatal(err)
		}
		win := surfaceWindow{1280, 720}
		s := &screen{win: win, r: r, f: f, l: newLayout(win.w, win.h, f)}
		s.draw(c.sc)
		if dir != "" {
			if err := img.SavePNG(surf, filepath.Join(dir, c.name+".png")); err != nil {
				t.Errorf("%s: %v", c.name, err)
			}
		}
		_ = r.Destroy()
		surf.Free()
	}
}
