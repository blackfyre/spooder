package legview

import (
	"bufio"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// goldenPath is test/golden/leg_pose.txt, written from the C++ IK by
// test/test_leg_pose_golden.
var goldenPath = filepath.Join("..", "..", "..", "test", "golden", "leg_pose.txt")

type ikVector struct {
	name   string
	target Vec3
	angles Angles
}

func readIK(t *testing.T) []ikVector {
	t.Helper()
	f, err := os.Open(goldenPath)
	if err != nil {
		t.Fatalf("golden IK vectors: %v", err)
	}
	defer f.Close()
	var out []ikVector
	sc := bufio.NewScanner(f)
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		parts := strings.Split(text, " ")
		if len(parts) != 7 {
			t.Fatalf("line %d: %q", line, text)
		}
		var v [6]float64
		for i := range v {
			if v[i], err = strconv.ParseFloat(parts[i+1], 64); err != nil {
				t.Fatalf("line %d: %v", line, err)
			}
		}
		out = append(out, ikVector{parts[0], Vec3{v[0], v[1], v[2]}, Angles{v[3], v[4], v[5]}})
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	if len(out) == 0 {
		t.Fatal("no golden IK vectors")
	}
	return out
}

func near(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

// Forward kinematics of the C++ IK's angles returns its target.
func TestForwardKinematicsInvertsTheIK(t *testing.T) {
	for _, v := range readIK(t) {
		got := Foot(v.angles)
		if !near(got.X, v.target.X, 0.05) || !near(got.Y, v.target.Y, 0.05) || !near(got.Z, v.target.Z, 0.05) {
			t.Errorf("%s: foot %+v, want %+v", v.name, got, v.target)
		}
	}
}

// The same holds after the pose frame's centidegree rounding, within half
// a millimetre.
func TestForwardKinematicsAfterCentidegreeRounding(t *testing.T) {
	for _, v := range readIK(t) {
		var f protocol.Pose
		for j, deg := range []float64{v.angles.Coxa, v.angles.Femur, v.angles.Tibia} {
			f.Joints[j] = protocol.PoseJoint{CommandedCentideg: int16(math.Round(deg * 100)), PulseUs: 1500}
		}
		got := Foot(FromFrame(f).Legs[0].Angles)
		if !near(got.X, v.target.X, 0.5) || !near(got.Y, v.target.Y, 0.5) || !near(got.Z, v.target.Z, 0.5) {
			t.Errorf("%s: foot %+v, want %+v", v.name, got, v.target)
		}
	}
}

// Spec "Calibration pose": 0/90/90, the femur straight up and the tibia
// horizontal outward, the foot 224 mm out and 116 mm up.
func TestCalibrationPose(t *testing.T) {
	var cal *ikVector
	for _, v := range readIK(t) {
		if v.name == "calibration" {
			cal = &v
		}
	}
	if cal == nil || cal.angles != (Angles{0, 90, 90}) || cal.target != (Vec3{224, 0, 116}) {
		t.Fatalf("calibration vector %+v", cal)
	}
	s := SideView(Angles{0, 90, 90})
	want := Side{Pivot: Vec2{0, 0}, CoxaEnd: Vec2{41, 0}, Knee: Vec2{41, 116}, Foot: Vec2{224, 116}}
	for _, p := range [][2]Vec2{{s.Pivot, want.Pivot}, {s.CoxaEnd, want.CoxaEnd}, {s.Knee, want.Knee}, {s.Foot, want.Foot}} {
		if !near(p[0].X, p[1].X, 1e-9) || !near(p[0].Y, p[1].Y, 1e-9) {
			t.Fatalf("side view %+v, want %+v", s, want)
		}
	}
}

// The mounting matches the gait code: walking straight forward or to the
// right moves every foot on the ground straight backward or to the left
// in the body frame. gait.cpp's ground segment runs from
// (-v.x·side, v.y·side) to (v.x·side, -v.y·side), turned by the leg's
// placement (STRIDE_MULTIPLIER and LEG_PLACEMENT_ANGLE · ROTATION_MULTIPLIER).
func TestMountingMatchesTheGait(t *testing.T) {
	side := [Legs]float64{1, 1, 1, -1, -1, -1}
	placement := [Legs]float64{-56, 0, 56, -56, 0, 56}
	for _, v := range []Vec2{{0, 50}, {50, 0}, {30, -40}} {
		for leg, m := range Mounts {
			d := Vec2{2 * v.X * side[leg], -2 * v.Y * side[leg]}
			p := rad(placement[leg])
			d = Vec2{d.X*math.Cos(p) - d.Y*math.Sin(p), d.X*math.Sin(p) + d.Y*math.Cos(p)}
			o := m.ToBody(Vec3{})
			b := m.ToBody(Vec3{d.X, d.Y, 0})
			got := Vec2{b.X - o.X, b.Y - o.Y}
			if !near(got.X, -2*v.X, 1e-9) || !near(got.Y, -2*v.Y, 1e-9) {
				t.Errorf("leg %d, input %+v: foot moves %+v in the body, want %+v", leg, v, got, Vec2{-2 * v.X, -2 * v.Y})
			}
		}
	}
}

// Straight out, each leg points away from the body: left legs to -X,
// right legs to +X, corners forward or backward.
func TestLegsPointOutward(t *testing.T) {
	wantFront := [Legs]int{1, 0, -1, -1, 0, 1}
	for leg, m := range Mounts {
		foot := m.ToBody(Foot(Angles{0, 17, 95}))
		out := foot.X - m.Pivot.X
		if (leg < 3) != (out < 0) {
			t.Errorf("leg %d: foot %+v not outward of %+v", leg, foot, m.Pivot)
		}
		fwd := foot.Y - m.Pivot.Y
		switch {
		case wantFront[leg] == 0 && !near(fwd, 0, 1e-9),
			wantFront[leg] > 0 && fwd <= 0,
			wantFront[leg] < 0 && fwd >= 0:
			t.Errorf("leg %d: foot %+v from %+v", leg, foot, m.Pivot)
		}
	}
}

// A positive coxa angle turns a leg clockwise seen from above.
func TestPositiveCoxaTurnsClockwise(t *testing.T) {
	for leg, m := range Mounts {
		a := m.ToBody(Foot(Angles{0, 17, 95}))
		b := m.ToBody(Foot(Angles{20, 17, 95}))
		ax, ay := a.X-m.Pivot.X, a.Y-m.Pivot.Y
		bx, by := b.X-m.Pivot.X, b.Y-m.Pivot.Y
		if cross := ax*by - ay*bx; cross >= 0 {
			t.Errorf("leg %d turned counter-clockwise", leg)
		}
	}
}

func TestClassify(t *testing.T) {
	for _, tc := range []struct {
		deg  float64
		want Limit
	}{
		{-0.01, LimitAt}, {0, LimitNear}, {5, LimitNear}, {5.01, LimitNone}, {90, LimitNone},
		{174.99, LimitNone}, {175, LimitNear}, {180, LimitNear}, {180.01, LimitAt}, {183, LimitAt},
	} {
		if got := Classify(tc.deg); got != tc.want {
			t.Errorf("Classify(%v) = %v, want %v", tc.deg, got, tc.want)
		}
	}
}

func TestFromFrame(t *testing.T) {
	var f protocol.Pose
	f.Flags = protocol.PoseOutputsOn
	for i := 0; i < 12; i++ {
		f.Joints[i] = protocol.PoseJoint{CommandedCentideg: int16(-1234 + i), ServoCentideg: int16(9000 + i), PulseUs: 1500}
	}
	f.Joints[7].ServoCentideg = 18300 // leg 2 femur at its limit
	f.Joints[9].PulseUs = 0           // leg 3 coxa unset, its other joints set
	p := FromFrame(f)
	if !p.OutputsOn {
		t.Fatal("outputs on")
	}
	if l := p.Legs[0]; !l.Set || l.Angles != (Angles{-12.34, -12.33, -12.32}) || l.Joints[1].Servo != 90.01 || l.Joints[2].PulseUs != 1500 {
		t.Fatalf("leg 0 %+v", l)
	}
	if p.Legs[2].Joints[Femur].Limit != LimitAt || p.Legs[2].Joints[Coxa].Limit != LimitNone {
		t.Fatalf("leg 2 %+v", p.Legs[2])
	}
	if !p.Legs[3].Set || p.Legs[4].Set || p.Legs[5].Set {
		t.Fatalf("set %v %v %v", p.Legs[3].Set, p.Legs[4].Set, p.Legs[5].Set)
	}
	if FromFrame(protocol.Pose{}).OutputsOn {
		t.Fatal("outputs off")
	}
}
