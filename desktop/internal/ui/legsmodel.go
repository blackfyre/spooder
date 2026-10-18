package ui

import (
	"fmt"
	"math"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/legview"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// legsState is what the legs pane draws (design D7 of
// add-desktop-leg-view): the pose when it is current, otherwise why there
// is none; and the editor's selection as a leg and joint.
type legsState struct {
	Current bool   // a pose no older than session.StaleAfter
	Reason  string // why there is no pose, when not Current
	Pose    legview.Pose
	Age     time.Duration
	Leg     int // the selected leg
	Joint   int // its selected joint (legview.Coxa, Femur, Tibia)
}

func legsFor(snap session.Snapshot, sel selection) legsState {
	st := legsState{Leg: sel.joint / legview.JointsPerLeg, Joint: sel.joint % legview.JointsPerLeg}
	switch {
	case snap.Link.Status != link.Connected:
		st.Reason = "Connect to the robot to see its legs."
	case !snap.HavePose:
		st.Reason = "No pose frame yet. The robot sends its pose on USB ten times a second."
	case !snap.PoseFresh:
		st.Reason = "The last pose frame is " + ageText(snap.PoseAge) + " old."
	default:
		st.Current = true
		st.Pose = legview.FromFrame(snap.Pose)
		st.Age = snap.PoseAge
	}
	return st
}

// legNumber is a leg's number as the editor shows it (1–6; the robot's
// leg index plus one).
func legNumber(leg int) int { return calib.Leg(leg * legview.JointsPerLeg) }

// legTitle names a leg as the editor does, with its assumed place.
func legTitle(leg int) string {
	return fmt.Sprintf("Leg %d · %s", legNumber(leg), legview.Mounts[leg].Name)
}

// angleText is a reported angle in degrees, to the frame's resolution.
func angleText(deg float64) string {
	return fmt.Sprintf("%.2f°", deg)
}

// jointPulseText is a reported pulse, or "not set".
func jointPulseText(us uint16) string {
	if us == 0 {
		return "not set"
	}
	return pulseText(us)
}

// footText is a foot position in the leg frame, to the millimetre.
func footText(p legview.Vec3) string {
	r := func(v float64) int { return int(math.Round(v)) }
	return fmt.Sprintf("foot %d, %d, %d mm", r(p.X), r(p.Y), r(p.Z))
}
