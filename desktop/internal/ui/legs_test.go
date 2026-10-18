package ui

import (
	"math"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/legview"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// The compiled default offsets (core/movement/calibration_defaults.cpp), in
// degrees, servo output order.
var defaultOffsets = [18]float64{90, 75, -18, 93, 75, -15, 93, 75, -18, 87, 80, -26, 85, 89, -16, 93, 85, -24}

func calibrationAngles() [6][3]float64 {
	var a [6][3]float64
	for leg := range a {
		a[leg] = [3]float64{0, 90, 90}
	}
	return a
}

// Walking feet from test/golden/leg_pose.txt: corner reach, forward end,
// corner push, swing lift, backward end, standing.
var walkingAngles = [6][3]float64{
	{16.9275, 19.7523, 80.2062},
	{17.5256, 15.9789, 91.1842},
	{-26.5651, 18.7169, 103.4371},
	{6.0090, 38.9591, 109.8698},
	{-17.5256, 15.9789, 91.1842},
	{0, 17.0395, 95.0254},
}

// Standing, with leg 2's femur driven past 180° and its coxa near 0°, and
// leg 5 never set (all angles NaN: pulse 0).
var limitAngles = [6][3]float64{
	{0, 17.0395, 95.0254},
	{0, 17.0395, 95.0254},
	{-88, 108, 95.0254},
	{0, 17.0395, 95.0254},
	{0, 17.0395, 95.0254},
	{math.NaN(), math.NaN(), math.NaN()},
}

// testPose builds a pose frame as the robot would: servo angles with the
// default offsets, pulses through 500–2500 µs; NaN angles leave a joint
// unset.
func testPose(angles [6][3]float64, outputsOn bool) protocol.Pose {
	var p protocol.Pose
	if outputsOn {
		p.Flags = protocol.PoseOutputsOn
	}
	for leg := range angles {
		for j, deg := range angles[leg] {
			i := leg*3 + j
			if math.IsNaN(deg) {
				continue
			}
			servo := deg + defaultOffsets[i]
			clamped := math.Max(0, math.Min(180, servo))
			p.Joints[i] = protocol.PoseJoint{
				CommandedCentideg: int16(math.Round(deg * 100)),
				ServoCentideg:     int16(math.Round(servo * 100)),
				PulseUs:           uint16(math.Round(500 + clamped/180*2000)),
			}
		}
	}
	return p
}

// Spec "No stale pose" and the editor's selection.
func TestLegsState(t *testing.T) {
	now := time.Date(2026, 10, 18, 9, 0, 0, 0, time.UTC)
	connected := link.State{Status: link.Connected}
	pose := testPose(calibrationAngles(), true)
	for _, tc := range []struct {
		name    string
		snap    session.Snapshot
		current bool
		reason  string
	}{
		{"disconnected", session.Snapshot{Now: now, HavePose: true, PoseFresh: true, Pose: pose}, false, "Connect"},
		{"no pose yet", session.Snapshot{Now: now, Link: connected}, false, "No pose frame yet"},
		{"stale", session.Snapshot{Now: now, Link: connected, HavePose: true, PoseAge: 1500 * time.Millisecond, Pose: pose}, false, "1.5 s old"},
		{"current", session.Snapshot{Now: now, Link: connected, HavePose: true, PoseFresh: true, PoseAge: 80 * time.Millisecond, Pose: pose}, true, ""},
	} {
		st := legsFor(tc.snap, selection{joint: 14})
		if st.Current != tc.current || !strings.Contains(st.Reason, tc.reason) || (tc.reason == "") != (st.Reason == "") {
			t.Errorf("%s: %+v", tc.name, st)
		}
		if st.Leg != 4 || st.Joint != legview.Tibia {
			t.Errorf("%s: selection leg %d joint %d, want leg 4 tibia", tc.name, st.Leg, st.Joint)
		}
		if tc.current && (!st.Pose.OutputsOn || st.Pose.Legs[2].Angles != (legview.Angles{Coxa: 0, Femur: 90, Tibia: 90})) {
			t.Errorf("%s: pose %+v", tc.name, st.Pose)
		}
	}
}

// Spec "Joint at its limit": a servo angle of 183° marks leg 2's femur.
func TestLimitPose(t *testing.T) {
	p := legview.FromFrame(testPose(limitAngles, false))
	if p.Legs[2].Joints[legview.Femur].Limit != legview.LimitAt || p.Legs[2].Joints[legview.Coxa].Limit != legview.LimitNear {
		t.Fatalf("leg 2 %+v", p.Legs[2])
	}
	if p.Legs[5].Set || !p.Legs[4].Set {
		t.Fatal("leg 5 should be unset, leg 4 set")
	}
}

// The legs pane lays out in every state, in both palettes.
func TestLegsPaneLaysOut(t *testing.T) {
	for _, dark := range []bool{false, true} {
		w, router, _ := testWindow(t, dark)
		w.tab = tabLegs
		frame(w, router) // disconnected
		s := w.env.Session
		s.LinkState(link.State{Status: link.Connected, Port: servo})
		frame(w, router) // no pose yet
		for _, a := range [][6][3]float64{calibrationAngles(), walkingAngles, limitAngles} {
			for sel := 0; sel < 18; sel += 4 {
				w.sel = selection{joint: sel}
				s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(a, sel%2 == 0)}, link.Stats{})
				frame(w, router)
			}
		}
		clock = clock.Add(2 * time.Second)
		frame(w, router) // stale
		if legsFor(w.snap, w.sel).Current {
			t.Fatal("pose still current after 2 s")
		}
	}
}
