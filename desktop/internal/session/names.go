package session

import (
	"strconv"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// BodyMode is the robot's body::Mode, as telemetry reports it.
type BodyMode uint8

const (
	BodyInitialise BodyMode = iota
	BodyStand
	BodyWalk
	BodySleep
	BodyCalibrate
	BodyPerform // playing a gesture
)

func (m BodyMode) String() string {
	switch m {
	case BodyInitialise:
		return "Initialise"
	case BodyStand:
		return "Stand"
	case BodyWalk:
		return "Walk"
	case BodySleep:
		return "Sleep"
	case BodyCalibrate:
		return "Calibrate"
	case BodyPerform:
		return "Perform"
	}
	return "mode " + strconv.Itoa(int(m))
}

// Gesture names, by protocol.Gesture* id.
var gestureNames = []string{"none", "say hi", "wiggle", "dance", "attack", "ball"}

// GestureName names a gesture id.
func GestureName(g uint8) string {
	if int(g) < len(gestureNames) {
		return gestureNames[g]
	}
	return "gesture " + strconv.Itoa(int(g))
}

// RequestedMode is the command frame's mode (RequestedMode in
// core/movement/movement_command.h).
type RequestedMode uint8

const (
	RequestStand RequestedMode = iota
	RequestWalk
	RequestSleep
	RequestCalibrate
)

func (m RequestedMode) String() string {
	switch m {
	case RequestStand:
		return "Stand"
	case RequestWalk:
		return "Walk"
	case RequestSleep:
		return "Sleep"
	case RequestCalibrate:
		return "Calibrate"
	}
	return "mode " + strconv.Itoa(int(m))
}

// Gait names, in gait::Gait order.
var gaitNames = []string{"Tri", "Wave", "Ripple", "Bi", "Quad", "Hop"}

// GaitName names a gait index.
func GaitName(g uint8) string {
	if int(g) < len(gaitNames) {
		return gaitNames[g]
	}
	return "gait " + strconv.Itoa(int(g))
}

// Fault bits (protection::Fault).
const (
	FaultOvercurrent      uint16 = 1 << 0
	FaultSustainedCurrent uint16 = 1 << 1
	FaultUndervoltage     uint16 = 1 << 2
	FaultBrownout         uint16 = 1 << 3
	FaultOvervoltage      uint16 = 1 << 4
	FaultEmergencyStop    uint16 = 1 << 5
)

var faultNames = []struct {
	bit  uint16
	name string
}{
	{FaultOvercurrent, "overcurrent"},
	{FaultSustainedCurrent, "sustained current"},
	{FaultUndervoltage, "undervoltage"},
	{FaultBrownout, "brown-out"},
	{FaultOvervoltage, "overvoltage"},
	{FaultEmergencyStop, "emergency stop"},
}

// FaultNames names every set fault bit, in bit order; unknown bits are
// named by number.
func FaultNames(bits uint16) []string {
	var out []string
	known := uint16(0)
	for _, f := range faultNames {
		known |= f.bit
		if bits&f.bit != 0 {
			out = append(out, f.name)
		}
	}
	for i := 0; i < 16; i++ {
		if b := uint16(1) << i; bits&b != 0 && known&b == 0 {
			out = append(out, "fault bit "+strconv.Itoa(i))
		}
	}
	return out
}

// Source is the operator source the robot follows.
type Source int

const (
	SourceNone Source = iota
	SourceBridge
	SourceUSB
	SourceConflict // both claim control: the robot sleeps
)

func (s Source) String() string {
	switch s {
	case SourceBridge:
		return "bridge"
	case SourceUSB:
		return "USB"
	case SourceConflict:
		return "conflict"
	}
	return "none"
}

// SourceFrom reads the source from telemetry flags.
func SourceFrom(flags uint8) Source {
	switch {
	case flags&protocol.TelemetrySourceConflict != 0:
		return SourceConflict
	case flags&protocol.TelemetryUSBInControl != 0:
		return SourceUSB
	case flags&protocol.TelemetryBridgeInControl != 0:
		return SourceBridge
	}
	return SourceNone
}
