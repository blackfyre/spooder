package legview

import (
	"math"
	"sort"
)

// The estimated centre of gravity: core/movement/centre_of_gravity's point
// masses on this package's geometry, pinned to it by
// test/golden/centre_of_gravity.txt. A static estimate: no accelerations,
// no foot forces.

// Masses in grams (centre_of_gravity.h). The servo and the battery are
// weighed; the rest are estimates until the rebuilt robot is weighed.
const (
	BodyMassG    = 420.0 // frame, Servo 2040, BEC, wiring; no battery
	BatteryMassG = 284.0 // 2S 5000 mAh, weighed
	servoMassG   = 55.0  // MG996R with horn and screws, weighed; one per joint
	coxaLinkG    = 30.0  // halfway along the coxa
	femurLinkG   = 45.0  // halfway along the femur
	tibiaLinkG   = 50.0  // tibiaLinkAt of the way from the knee to the foot
	tibiaLinkAt  = 0.4
)

// Where the body's and the battery's masses sit, in the body frame.
var (
	bodyAt    = Point3{}
	batteryAt = Point3{}
)

// ContactMM: a foot higher than this above the lowest is off the ground.
const ContactMM = 10.0

// Gravity is an estimate: the centre in the body frame and the total mass.
type Gravity struct {
	Centre Point3
	MassG  float64
}

func lerp3(a, b Point3, f float64) Point3 {
	return Point3{a.X + (b.X-a.X)*f, a.Y + (b.Y-a.Y)*f, a.Z + (b.Z-a.Z)*f}
}

// CentreOfGravity estimates the centre of gravity of legs placed by Body3,
// with or without the battery. Without every leg set there is none.
func CentreOfGravity(legs [Legs]Leg3, battery bool) (Gravity, bool) {
	var m Point3
	mass := 0.0
	add := func(p Point3, g float64) {
		m.X, m.Y, m.Z = m.X+p.X*g, m.Y+p.Y*g, m.Z+p.Z*g
		mass += g
	}
	add(bodyAt, BodyMassG)
	if battery {
		add(batteryAt, BatteryMassG)
	}
	for _, l := range legs {
		if !l.Set {
			return Gravity{}, false
		}
		p := l.Points
		add(p[0], servoMassG) // coxa servo
		add(p[1], servoMassG) // femur servo
		add(p[2], servoMassG) // tibia servo
		add(lerp3(p[0], p[1], 0.5), coxaLinkG)
		add(lerp3(p[1], p[2], 0.5), femurLinkG)
		add(lerp3(p[2], p[3], tibiaLinkAt), tibiaLinkG)
	}
	return Gravity{Point3{m.X / mass, m.Y / mass, m.Z / mass}, mass}, true
}

func cross2(o, a, b Vec2) float64 { return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X) }

// SupportHull is the support polygon: the convex hull of the set feet within
// ContactMM of the lowest, anticlockwise from above. With fewer than three
// such feet, or all in one line, it is nil.
func SupportHull(legs [Legs]Leg3) []Vec2 {
	ground, ok := Ground(legs)
	if !ok {
		return nil
	}
	var pts []Vec2
	for _, l := range legs {
		if f := l.Points[3]; l.Set && f.Z <= ground+ContactMM {
			pts = append(pts, Vec2{f.X, f.Y})
		}
	}
	if len(pts) < 3 {
		return nil
	}
	sort.Slice(pts, func(i, j int) bool {
		return pts[i].X < pts[j].X || (pts[i].X == pts[j].X && pts[i].Y < pts[j].Y)
	})
	// Andrew's monotone chain, collinear points dropped.
	h := make([]Vec2, 0, 2*len(pts))
	for _, p := range pts {
		for len(h) >= 2 && cross2(h[len(h)-2], h[len(h)-1], p) <= 0 {
			h = h[:len(h)-1]
		}
		h = append(h, p)
	}
	lower := len(h) + 1
	for i := len(pts) - 2; i >= 0; i-- {
		for len(h) >= lower && cross2(h[len(h)-2], h[len(h)-1], pts[i]) <= 0 {
			h = h[:len(h)-1]
		}
		h = append(h, pts[i])
	}
	h = h[:len(h)-1] // the last point repeats the first
	if len(h) < 3 {
		return nil
	}
	return h
}

// SupportMargin is how far p lies inside the hull: the distance to its
// nearest edge, negative outside.
func SupportMargin(hull []Vec2, p Vec2) float64 {
	m := math.Inf(1)
	for i, a := range hull {
		b := hull[(i+1)%len(hull)]
		ex, ey := b.X-a.X, b.Y-a.Y
		m = math.Min(m, (ex*(p.Y-a.Y)-ey*(p.X-a.X))/math.Hypot(ex, ey))
	}
	return m
}
