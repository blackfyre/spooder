// Package legview is the geometry of the calibrator's leg view (design D5,
// D6 of add-desktop-leg-view): forward kinematics matching the robot's
// kinematics::solveLeg, the side and top projections, the legs' mounting on
// the body, and the servo-limit classification. It draws nothing; the UI
// maps its millimetres to pixels.
package legview

import (
	"math"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Link lengths in millimetres (core/movement/kinematics.h).
const (
	CoxaLength  = 41.0
	FemurLength = 116.0
	TibiaLength = 183.0
)

// Legs and joints, in servo output order: output 3·leg + joint.
const (
	Legs         = 6
	JointsPerLeg = 3
)

// Joint indices within a leg.
const (
	Coxa = iota
	Femur
	Tibia
)

// JointNames name the joints of a leg.
var JointNames = [JointsPerLeg]string{"coxa", "femur", "tibia"}

// Vec2 is a point in a plane, in millimetres.
type Vec2 struct{ X, Y float64 }

// Vec3 is a point in a leg frame (origin at the coxa pivot, x outward, z
// up), in millimetres.
type Vec3 struct{ X, Y, Z float64 }

// Angles are a leg's commanded joint angles in degrees, as solveLeg returns
// them before calibration offsets.
type Angles struct{ Coxa, Femur, Tibia float64 }

func rad(deg float64) float64 { return deg * math.Pi / 180 }

// Side is a leg in its own vertical plane: X is the distance outward from
// the coxa pivot along the coxa, Y the height above it.
type Side struct {
	Pivot, CoxaEnd, Knee, Foot Vec2
}

// SideView is the leg's forward kinematics in its vertical plane. The femur
// angle is its elevation above horizontal; the tibia angle is 180° minus
// the interior knee angle, so the tibia's elevation is femur − tibia.
func SideView(a Angles) Side {
	coxaEnd := Vec2{CoxaLength, 0}
	femur := rad(a.Femur)
	knee := Vec2{coxaEnd.X + FemurLength*math.Cos(femur), FemurLength * math.Sin(femur)}
	tibia := rad(a.Femur - a.Tibia)
	foot := Vec2{knee.X + TibiaLength*math.Cos(tibia), knee.Y + TibiaLength*math.Sin(tibia)}
	return Side{Pivot: Vec2{}, CoxaEnd: coxaEnd, Knee: knee, Foot: foot}
}

// Plane maps a point of the leg's vertical plane into the leg frame: the
// plane is turned by the coxa angle from the x axis towards +y.
func Plane(a Angles, p Vec2) Vec3 {
	c := rad(a.Coxa)
	return Vec3{p.X * math.Cos(c), p.X * math.Sin(c), p.Y}
}

// Foot is the foot position in the leg frame: the inverse of solveLeg.
func Foot(a Angles) Vec3 { return Plane(a, SideView(a).Foot) }

// Mount places a leg on the body. The body frame is seen from above: X to
// the right, Y forward. A leg frame is mirrored relative to it: its x axis
// points along Angle (degrees counter-clockwise from +X), and its y axis is
// x turned 90° clockwise, so a positive coxa angle turns the leg clockwise.
type Mount struct {
	Name  string
	Pivot Vec2    // coxa pivot, schematic
	Angle float64 // the leg frame's x axis
}

// Mounts are derived from gait.cpp (design D6): legs 0–2 on the left,
// 3–5 on the right, corners at LEG_PLACEMENT_ANGLE (56°) from the middle
// legs. The leg order is confirmed on the robot: legs 1 and 6 (indices 0
// and 5) are the front pair, 3 and 4 the rear. The pivots are schematic, not
// measured; upstream's frame STLs can supply real positions.
var Mounts = [Legs]Mount{
	{"front left", Vec2{-70, 120}, 180 - 56},
	{"middle left", Vec2{-90, 0}, 180},
	{"rear left", Vec2{-70, -120}, 180 + 56},
	{"rear right", Vec2{70, -120}, -56},
	{"middle right", Vec2{90, 0}, 0},
	{"front right", Vec2{70, 120}, 56},
}

// ToBody maps a leg-frame point (its x and y) into the body frame.
func (m Mount) ToBody(p Vec3) Vec2 {
	t := rad(m.Angle)
	ex := Vec2{math.Cos(t), math.Sin(t)}
	ey := Vec2{math.Sin(t), -math.Cos(t)} // ex turned 90° clockwise
	return Vec2{m.Pivot.X + p.X*ex.X + p.Y*ey.X, m.Pivot.Y + p.X*ex.Y + p.Y*ey.Y}
}

// BodyOutline is the schematic body: a hexagon through the coxa pivots.
func BodyOutline() []Vec2 {
	return []Vec2{Mounts[0].Pivot, Mounts[5].Pivot, Mounts[4].Pivot, Mounts[3].Pivot, Mounts[2].Pivot, Mounts[1].Pivot}
}

// Limit classifies a servo angle against the pulse conversion's 0–180°
// range.
type Limit int

const (
	LimitNone Limit = iota
	LimitNear       // within NearLimitDeg of 0° or 180°
	LimitAt         // outside 0–180°: the pulse is clamped
)

// NearLimitDeg is how close to 0° or 180° a servo angle counts as near its
// limit.
const NearLimitDeg = 5.0

// Classify returns a servo angle's limit state.
func Classify(servoDeg float64) Limit {
	switch {
	case servoDeg < 0 || servoDeg > 180:
		return LimitAt
	case servoDeg <= NearLimitDeg || servoDeg >= 180-NearLimitDeg:
		return LimitNear
	}
	return LimitNone
}

func (l Limit) String() string {
	switch l {
	case LimitNear:
		return "near limit"
	case LimitAt:
		return "at limit"
	}
	return ""
}

// Joint is one joint of a reported pose.
type Joint struct {
	Commanded float64 // degrees, before calibration
	Servo     float64 // degrees, commanded + applied offset
	PulseUs   uint16  // 0 = never set
	Limit     Limit
}

// Leg is one leg of a reported pose. A leg whose three pulses are 0 has
// never been set since the robot started, and has no geometry.
type Leg struct {
	Set    bool
	Angles Angles
	Joints [JointsPerLeg]Joint
}

// Pose is a decoded pose frame, by leg.
type Pose struct {
	OutputsOn bool
	Legs      [Legs]Leg
}

// FromFrame converts a pose frame.
func FromFrame(f protocol.Pose) Pose {
	p := Pose{OutputsOn: f.Flags&protocol.PoseOutputsOn != 0}
	for leg := range p.Legs {
		l := &p.Legs[leg]
		var deg [JointsPerLeg]float64
		for j := range l.Joints {
			pj := f.Joints[leg*JointsPerLeg+j]
			deg[j] = float64(pj.CommandedCentideg) / 100
			servo := float64(pj.ServoCentideg) / 100
			l.Joints[j] = Joint{Commanded: deg[j], Servo: servo, PulseUs: pj.PulseUs, Limit: Classify(servo)}
			if pj.PulseUs != 0 {
				l.Set = true
			}
		}
		l.Angles = Angles{deg[Coxa], deg[Femur], deg[Tibia]}
	}
	return p
}
