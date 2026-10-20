package legview

import "math"

// Point3 is a point in the body frame, in millimetres: X to the right, Y
// forward, Z up, the coxa pivots at Z = 0.
type Point3 struct{ X, Y, Z float64 }

// Leg3 is one leg in the body frame: coxa pivot, coxa end, knee and foot.
type Leg3 struct {
	Set    bool
	Points [4]Point3
}

// Body3 places every leg of a pose in the body frame.
func Body3(p Pose) [Legs]Leg3 {
	var out [Legs]Leg3
	for leg, l := range p.Legs {
		out[leg].Set = l.Set
		if !l.Set {
			m := Mounts[leg]
			out[leg].Points[0] = Point3{m.Pivot.X, m.Pivot.Y, 0}
			continue
		}
		s := SideView(l.Angles)
		for i, q := range [4]Vec2{s.Pivot, s.CoxaEnd, s.Knee, s.Foot} {
			out[leg].Points[i] = bodyPoint(leg, Plane(l.Angles, q))
		}
	}
	return out
}

func bodyPoint(leg int, p Vec3) Point3 {
	b := Mounts[leg].ToBody(p)
	return Point3{b.X, b.Y, p.Z}
}

// Ground is the lowest set foot's height, and whether any foot is set.
func Ground(legs [Legs]Leg3) (float64, bool) {
	z, any := math.Inf(1), false
	for _, l := range legs {
		if l.Set {
			z, any = math.Min(z, l.Points[3].Z), true
		}
	}
	return z, any
}

// PlateThickness is the schematic body plate's thickness, centred on the
// coxa pivots' plane.
const PlateThickness = 40.0

// Plate is the schematic body: the outline through the coxa pivots, as its
// top and bottom faces.
func Plate() (top, bottom []Point3) {
	for _, q := range BodyOutline() {
		top = append(top, Point3{q.X, q.Y, PlateThickness / 2})
		bottom = append(bottom, Point3{q.X, q.Y, -PlateThickness / 2})
	}
	return top, bottom
}

// View is a projection of the body frame onto a screen plane.
type View int

const (
	ViewTop       View = iota // from above, front up
	ViewFront                 // from ahead: the robot's left is on the right
	ViewLeft                  // from the robot's left: the front is on the right
	ViewIsometric             // from the front left, above
	ViewCount
)

// ViewNames caption the views.
var ViewNames = [ViewCount]string{"top · front ↑", "front · from ahead", "left · front →", "isometric · front left"}

// The isometric view looks from isoFrom (front left, above) towards the
// body; isoRight and isoUp are the screen's axes, orthonormal with it.
var (
	isoFrom  = Point3{-1 / math.Sqrt(3), 1 / math.Sqrt(3), 1 / math.Sqrt(3)}
	isoRight = Point3{1 / math.Sqrt(2), 1 / math.Sqrt(2), 0}
	isoUp    = Point3{1 / math.Sqrt(6), -1 / math.Sqrt(6), 2 / math.Sqrt(6)}
)

func dot3(a, b Point3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// Project maps a body point to the view's screen plane (x right, y up, in
// millimetres) and its depth (larger is nearer the viewer).
func Project(v View, p Point3) (Vec2, float64) {
	switch v {
	case ViewTop:
		return Vec2{p.X, p.Y}, p.Z
	case ViewFront:
		return Vec2{-p.X, p.Z}, p.Y
	case ViewLeft:
		return Vec2{p.Y, p.Z}, -p.X
	}
	return Vec2{dot3(p, isoRight), dot3(p, isoUp)}, dot3(p, isoFrom)
}

// reach is the farthest a foot gets from its coxa pivot.
const reach = CoxaLength + FemurLength + TibiaLength

// ViewBounds are a view's fixed screen-plane extents (minX, minY, maxX,
// maxY), from the mounts and the legs' full reach, so the drawing does
// not jump as the pose changes.
func ViewBounds(v View) [4]float64 {
	lo := Point3{math.Inf(1), math.Inf(1), -reach}
	hi := Point3{math.Inf(-1), math.Inf(-1), FemurLength + TibiaLength} // a leg pointing straight up
	for _, m := range Mounts {
		lo.X, hi.X = math.Min(lo.X, m.Pivot.X-reach), math.Max(hi.X, m.Pivot.X+reach)
		lo.Y, hi.Y = math.Min(lo.Y, m.Pivot.Y-reach), math.Max(hi.Y, m.Pivot.Y+reach)
	}
	b := [4]float64{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	for _, x := range []float64{lo.X, hi.X} {
		for _, y := range []float64{lo.Y, hi.Y} {
			for _, z := range []float64{lo.Z, hi.Z} {
				q, _ := Project(v, Point3{x, y, z})
				b[0], b[1] = math.Min(b[0], q.X), math.Min(b[1], q.Y)
				b[2], b[3] = math.Max(b[2], q.X), math.Max(b[3], q.Y)
			}
		}
	}
	return b
}
