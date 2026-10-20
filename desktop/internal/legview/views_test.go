package legview

import (
	"math"
	"testing"
)

func posed(a Angles) Pose {
	var p Pose
	for leg := range p.Legs {
		p.Legs[leg] = Leg{Set: true, Angles: a}
	}
	return p
}

// Spec "Calibration pose": femurs vertical, tibias horizontal and outward,
// every leg straight out from its mount.
func TestCalibrationPoseInTheViews(t *testing.T) {
	legs := Body3(posed(Angles{0, 90, 90}))
	for leg, l := range legs {
		pivot, coxa, knee, foot := l.Points[0], l.Points[1], l.Points[2], l.Points[3]
		// Femur vertical: the knee straight above the coxa end, 116 mm up.
		if !near(knee.X, coxa.X, 1e-9) || !near(knee.Y, coxa.Y, 1e-9) || !near(knee.Z, 116, 1e-9) {
			t.Fatalf("leg %d knee %+v over coxa end %+v", leg, knee, coxa)
		}
		// Tibia horizontal, outward: the foot at the knee's height, 183 mm
		// further along the leg's direction.
		dir := Mounts[leg].ToBody(Vec3{X: 1})
		out := Vec2{dir.X - pivot.X, dir.Y - pivot.Y}
		if !near(foot.Z, 116, 1e-9) || !near(foot.X-knee.X, 183*out.X, 1e-9) || !near(foot.Y-knee.Y, 183*out.Y, 1e-9) {
			t.Fatalf("leg %d foot %+v from knee %+v", leg, foot, knee)
		}
		// In the front and left views the femur is vertical on screen.
		for _, v := range []View{ViewFront, ViewLeft} {
			k, _ := Project(v, knee)
			c, _ := Project(v, coxa)
			if !near(k.X, c.X, 1e-9) || !near(k.Y-c.Y, 116, 1e-9) {
				t.Fatalf("leg %d view %d femur from %+v to %+v", leg, v, c, k)
			}
		}
	}
}

// Spec "Standing": every foot on the ground line.
func TestStandingFeetOnTheGround(t *testing.T) {
	// The standing foot (190, 0, -60) solves to these angles (golden IK).
	var stand Angles
	for _, v := range readIK(t) {
		if v.target == (Vec3{190, 0, -60}) {
			stand = v.angles
		}
	}
	if stand == (Angles{}) {
		t.Skip("no standing vector in the golden file")
	}
	legs := Body3(posed(stand))
	g, ok := Ground(legs)
	if !ok || !near(g, -60, 0.05) {
		t.Fatalf("ground %v %v", g, ok)
	}
	for leg, l := range legs {
		for _, v := range []View{ViewFront, ViewLeft} {
			f, _ := Project(v, l.Points[3])
			gp, _ := Project(v, Point3{l.Points[3].X, l.Points[3].Y, g})
			if !near(f.Y, gp.Y, 0.05) {
				t.Fatalf("leg %d view %d foot %v, ground %v", leg, v, f.Y, gp.Y)
			}
		}
	}
}

func TestProjections(t *testing.T) {
	p := Point3{10, 20, 30}
	cases := []struct {
		v     View
		want  Vec2
		depth float64
	}{
		{ViewTop, Vec2{10, 20}, 30},
		{ViewFront, Vec2{-10, 30}, 20}, // the robot's left (−X) on the right
		{ViewLeft, Vec2{20, 30}, -10},  // the front (+Y) on the right
	}
	for _, c := range cases {
		got, d := Project(c.v, p)
		if got != c.want || d != c.depth {
			t.Errorf("view %d: %+v depth %v, want %+v depth %v", c.v, got, d, c.want, c.depth)
		}
	}
	// The isometric axes and the viewing direction are orthonormal, and up
	// points up.
	for _, a := range []Point3{isoFrom, isoRight, isoUp} {
		if !near(dot3(a, a), 1, 1e-12) {
			t.Fatalf("not unit: %+v", a)
		}
	}
	if !near(dot3(isoFrom, isoRight), 0, 1e-12) || !near(dot3(isoFrom, isoUp), 0, 1e-12) || !near(dot3(isoRight, isoUp), 0, 1e-12) {
		t.Fatal("isometric axes not orthogonal")
	}
	if up, _ := Project(ViewIsometric, Point3{0, 0, 1}); up.Y <= 0 || !near(up.X, 0, 1e-12) {
		t.Fatalf("isometric up %+v", up)
	}
	// Seen from the front left, the front left leg is nearest and the rear
	// right farthest.
	nearest, farthest := math.Inf(-1), math.Inf(1)
	ni, fi := -1, -1
	for leg, m := range Mounts {
		_, d := Project(ViewIsometric, Point3{m.Pivot.X, m.Pivot.Y, 0})
		if d > nearest {
			nearest, ni = d, leg
		}
		if d < farthest {
			farthest, fi = d, leg
		}
	}
	if Mounts[ni].Name != "front left" || Mounts[fi].Name != "rear right" {
		t.Fatalf("nearest %s, farthest %s", Mounts[ni].Name, Mounts[fi].Name)
	}
}

// The fixed bounds hold every reachable point, and an unset leg has only
// its pivot.
func TestViewBoundsAndUnsetLegs(t *testing.T) {
	for v := View(0); v < ViewCount; v++ {
		b := ViewBounds(v)
		var all []Leg3
		for _, a := range []Angles{{0, 0, 0}, {0, 90, 0}, {0, -90, 0}, {60, 45, 20}, {-60, -45, -20}} { // out, up, down, turned
			l := Body3(posed(a))
			all = append(all, l[:]...)
		}
		for _, l := range all {
			for _, p := range l.Points {
				q, _ := Project(v, p)
				if q.X < b[0]-1e-9 || q.Y < b[1]-1e-9 || q.X > b[2]+1e-9 || q.Y > b[3]+1e-9 {
					t.Fatalf("view %d: %+v outside %v", v, q, b)
				}
			}
		}
	}
	var p Pose
	l := Body3(p)
	if l[0].Set || l[0].Points[0] != (Point3{Mounts[0].Pivot.X, Mounts[0].Pivot.Y, 0}) {
		t.Fatalf("unset leg %+v", l[0])
	}
	if _, ok := Ground(l); ok {
		t.Fatal("ground with no foot set")
	}
	top, bottom := Plate()
	if len(top) != 6 || top[0].Z-bottom[0].Z != PlateThickness {
		t.Fatalf("plate %v %v", top, bottom)
	}
}
