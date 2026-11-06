// Package supplyband is the band a supply reading is in against the
// protection limits the robot reports in its diagnostics frames (openspec
// change handheld-observations, design D3): ok, warning, or at or past the
// level at which protection faults. It is a line-for-line copy of
// core/status/supply_band, kept in step through test/golden/supply_bands.txt.
package supplyband

import "github.com/blackfyre/spooder/desktop/internal/protocol"

// Band is a reading's band.
type Band int

const (
	Unknown Band = iota // no limits reported, or limits out of order
	Ok
	Warning
	Fault
)

func (b Band) String() string {
	switch b {
	case Ok:
		return "ok"
	case Warning:
		return "warning"
	case Fault:
		return "fault"
	}
	return "unknown"
}

// Limits are the robot's protection limits.
type Limits struct {
	BrownoutMv         uint16
	MinMv              uint16 // undervoltage
	MaxMv              uint16 // overvoltage
	SustainedCurrentMa uint16
	TripCurrentMa      uint16
}

// FromDiagnostics is the limits a diagnostics frame reports.
func FromDiagnostics(d protocol.Diagnostics) Limits {
	return Limits{d.BrownoutMv, d.MinMv, d.MaxMv, d.SustainedCurrentMa, d.TripCurrentMa}
}

// Valid is whether l is ordered as protection requires: brown-out below the
// minimum below the maximum voltage, and a sustained current above zero and
// below the trip current.
func (l Limits) Valid() bool {
	return l.BrownoutMv < l.MinMv && l.MinMv < l.MaxMv && l.SustainedCurrentMa > 0 &&
		l.SustainedCurrentMa < l.TripCurrentMa
}

// Voltage is mv's band: Fault at or below the minimum or at or above the
// maximum; Warning within the brown-out-to-minimum gap of either; else Ok.
// Unknown for limits that are not valid.
func Voltage(l Limits, mv uint16) Band {
	if !l.Valid() {
		return Unknown
	}
	if mv <= l.MinMv || mv >= l.MaxMv {
		return Fault
	}
	// The warning margin: the gap protection leaves between undervoltage
	// and brown-out, on both sides; in int, as the sums can pass 65535.
	margin := int(l.MinMv) - int(l.BrownoutMv)
	if int(mv) < int(l.MinMv)+margin || int(mv) > int(l.MaxMv)-margin {
		return Warning
	}
	return Ok
}

// Current is ma's band: Fault at or above the trip current; Warning at or
// above the sustained current; else Ok. Unknown for limits that are not
// valid.
func Current(l Limits, ma uint16) Band {
	if !l.Valid() {
		return Unknown
	}
	if ma >= l.TripCurrentMa {
		return Fault
	}
	if ma >= l.SustainedCurrentMa {
		return Warning
	}
	return Ok
}
