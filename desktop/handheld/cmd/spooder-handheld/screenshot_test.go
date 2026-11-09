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
	"github.com/blackfyre/spooder/desktop/handheld/internal/supplyband"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
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

// exampleLimits are power_limits.example.h's, as diagnostics report them.
var exampleLimits = supplyband.Limits{BrownoutMv: 4800, MinMv: 5200, MaxMv: 6800, SustainedCurrentMa: 6000, TripCurrentMa: 9000}

// surfaceWindow is a window drawn by a software renderer into a surface.
type surfaceWindow struct{ w, h int32 }

func (s surfaceWindow) GetWidth() int32  { return s.w }
func (s surfaceWindow) GetHeight() int32 { return s.h }
func (surfaceWindow) Present()           {}

// The drive screen's states, for screenshots and the drawing smoke test.
var screenshotCases = []struct {
	name string
	sc   app.Screen
	// open draws the case twice, that far apart: a gesture overlay held
	// open that long.
	open time.Duration
}{
	{"driving", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, Gesture: "", VoltageMv: 6040, CurrentMa: 3420, Source: "bridge", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
		SoundOn:   true,
		Held:      padmap.HandUp,
	}, 0},
	{"fault", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Sleep", Gait: "Tripod", Faults: []string{"emergency stop", "undervoltage"}, VoltageMv: 50, Source: "none", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Sleep", Height: 0, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer", EStop: true},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, Lost: 3, HaveFrame: true, SinceLast: 60 * time.Millisecond, SignalKnown: true, SignalDBm: -71, Sent: 402},
		State:     app.Driving,
		SoundOn:   true,
		Held:      padmap.HandSelect | padmap.HandY,
		Clearing:  0.6,
	}, 0},
	{"stale", app.Screen{
		Robot:     app.Robot{Have: true, Stale: true, Body: "Stand", Gait: "Ripple", OutputsOn: true, VoltageMv: 5950, CurrentMa: 1200, Source: "bridge", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Stand", Height: 80, Speed: 44, Gait: "Ripple", Stride: "Fixed", StickMode: "Crab"},
		Link:      app.LinkView{HaveFrame: true, SinceLast: 2300 * time.Millisecond, Sent: 9120, SendErrors: 4},
		State:     app.Driving,
		Notice:    "send: network is unreachable",
		// Select + B switched the sound off.
		SoundOn: false,
	}, 0},
	{"offnetwork", app.Screen{
		Robot:     app.Robot{Have: true, Stale: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, VoltageMv: 6020, CurrentMa: 1800, Source: "bridge", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Walk", Height: 40, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{HaveFrame: true, SinceLast: 1400 * time.Millisecond, SignalKnown: true, SignalDBm: -44, Sent: 6120, OffNetwork: true, Network: "spooder-drive-1a2b", NowOn: "Home WiFi"},
		State:     app.Driving,
		SoundOn:   true,
	}, 0},
	{"leaving", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, VoltageMv: 6020, CurrentMa: 2900, Source: "bridge", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Sleep", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 20 * time.Millisecond, SignalKnown: true, SignalDBm: -40, Sent: 30111},
		State:     app.Leaving,
		SoundOn:   true,
		Leave:     app.ActionExit,
	}, 0},
	{"driveview", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, Faults: []string{"undervoltage"}, VoltageMv: 5150, CurrentMa: 6420, Source: "bridge", SupplyLimits: exampleLimits},
		Pose:      walkingPose(),
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
		SoundOn:   true,
		Held:      padmap.HandSelect | padmap.HandX,
		DriveView: true,
	}, 0},
	{"driveview-nopose", app.Screen{
		Requested: app.Requested{Mode: "Sleep", Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		State:     app.Driving,
		SoundOn:   true,
		DriveView: true,
	}, 0},
	{"simulation", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Walk", Gait: "Tripod", OutputsOn: true, VoltageMv: 6000, CurrentMa: 2160, Source: "bridge", Simulated: true, SupplyLimits: exampleLimits},
		Pose:      walkingPose(),
		Requested: app.Requested{Mode: "Walk", Height: 120, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 40 * time.Millisecond, SignalKnown: true, SignalDBm: -33, Sent: 18250},
		State:     app.Driving,
		SoundOn:   true,
	}, 0},
	{"waiting", app.Screen{
		Requested: app.Requested{Mode: "Sleep", Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		State:     app.Driving,
		SoundOn:   true,
	}, 0},
	{"gestures", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Perform", Gait: "Tripod", OutputsOn: true, Gesture: "wiggle", GestureID: 2, VoltageMv: 6010, CurrentMa: 2300, Source: "bridge", SupplyLimits: exampleLimits},
		Requested: app.Requested{Mode: "Stand", Height: 0, Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer", Gesture: 2, GestureSeq: 3},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 30 * time.Millisecond, SignalKnown: true, SignalDBm: -48, Sent: 12040},
		State:     app.Driving,
		SoundOn:   true,
		Held:      padmap.HandL2,
	}, 650 * time.Millisecond},
	{"gestures-stand-first", app.Screen{
		Robot:     app.Robot{Have: true, Body: "Sleep", Gait: "Tripod", OutputsOn: true, VoltageMv: 6080, CurrentMa: 400, Source: "bridge", SupplyLimits: exampleLimits},
		Pose:      walkingPose(),
		Requested: app.Requested{Mode: "Sleep", Speed: 44, Gait: "Tripod", Stride: "Dynamic", StickMode: "Steer"},
		Link:      app.LinkView{Telemetry: 10, Pose: 10, Diagnostics: 10, HaveFrame: true, SinceLast: 30 * time.Millisecond, SignalKnown: true, SignalDBm: -48, Sent: 2040},
		State:     app.Driving,
		SoundOn:   true,
		Held:      padmap.HandL2 | padmap.HandA,
		DriveView: true,
	}, 1200 * time.Millisecond},
}

// The start screen's states (handheld-start-screen).
var startScreenshotCases = []struct {
	name string
	page startPage
	held padmap.Buttons
}{
	{"join", failedPage(&wlan.NotJoinedError{Network: "HomeNetwork"}), padmap.HandA},
	{"checking", checkingPage(true, ""), 0},
}

// TestDrawsEveryState draws each state of the drive screen and the start
// screen on a software renderer at the device's size. With
// SPOODER_SCREENSHOTS set to a directory, it also saves each as a PNG there.
func TestDrawsEveryState(t *testing.T) {
	if err := img.Init(img.INIT_PNG); err != nil {
		t.Fatal(err)
	}
	defer img.Quit()
	f := testFonts(t)
	dir := os.Getenv("SPOODER_SCREENSHOTS")
	render := func(name string, draw func(s *screen, now *time.Time)) {
		surf, err := sdl.CreateRGBSurfaceWithFormat(0, 1280, 720, 32, sdl.PIXELFORMAT_ARGB8888)
		if err != nil {
			t.Fatal(err)
		}
		defer surf.Free()
		r, err := sdl.CreateSoftwareRenderer(surf)
		if err != nil {
			t.Fatal(err)
		}
		defer func() { _ = r.Destroy() }()
		win := surfaceWindow{1280, 720}
		now := time.Unix(1000, 0)
		s := &screen{win: win, r: r, f: f, l: newLayout(win.w, win.h, f), now: func() time.Time { return now }}
		draw(s, &now)
		if dir != "" {
			if err := img.SavePNG(surf, filepath.Join(dir, name+".png")); err != nil {
				t.Errorf("%s: %v", name, err)
			}
		}
	}
	for _, c := range screenshotCases {
		render(c.name, func(s *screen, now *time.Time) {
			if c.open > 0 {
				s.draw(c.sc)
				*now = now.Add(c.open)
			}
			s.draw(c.sc)
		})
	}
	for _, c := range startScreenshotCases {
		render(c.name, func(s *screen, _ *time.Time) { s.start(c.page, c.held) })
	}
}
