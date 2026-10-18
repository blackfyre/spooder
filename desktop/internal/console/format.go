package console

import (
	"fmt"
	"strconv"
	"strings"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// Line markers (design D4): text lines carry none, so the robot's text
// never looks like the monitor's own lines.
const (
	markFrame  = "> "
	markReject = "! "
	markStatus = "-- "
)

// sep separates the fields of a frame line; names such as "emergency stop"
// hold single spaces.
const sep = "  "

// FormatFrame is the one-line decode of a frame, without the marker.
func FormatFrame(f protocol.Frame) string {
	switch f.Type {
	case protocol.TypeTelemetry:
		return formatTelemetry(f.Seq, f.Telemetry)
	case protocol.TypeCalibrationReport:
		return formatReport(f.Seq, f.CalibrationReport)
	}
	return protocol.Summary(f)
}

func formatTelemetry(seq uint8, t protocol.Telemetry) string {
	output := "off"
	if t.OutputsOn {
		output = "on"
	}
	faults := "none"
	if names := session.FaultNames(t.Faults); len(names) > 0 {
		faults = strings.Join(names, ", ")
	}
	gait := session.GaitName(t.Gait)
	if !strings.HasPrefix(gait, "gait ") { // an unknown index is named "gait N"
		gait = "gait " + gait
	}
	source := session.SourceFrom(t.Flags).String()
	if session.SourceFrom(t.Flags) == session.SourceConflict {
		source += " (bridge and USB)"
	}
	parts := []string{
		"telemetry #" + strconv.Itoa(int(seq)),
		session.BodyMode(t.BodyMode).String(),
		gait,
		"output " + output,
		fmt.Sprintf("%.2f V", float64(t.VoltageMv)/1000),
		fmt.Sprintf("%.2f A", float64(t.CurrentMa)/1000),
		"faults " + faults,
		fmt.Sprintf("link %d ok %d bad", t.ValidFrames, t.DiscardedFrames),
		"source " + source,
	}
	if t.SensorsAvailable&protocol.SensorToF != 0 {
		parts = append(parts, fmt.Sprintf("tof %d mm (status %d)", t.ToFMm, t.ToFStatus))
	}
	if t.SensorsAvailable&protocol.SensorAttitude != 0 {
		parts = append(parts, fmt.Sprintf("pitch %.2f deg roll %.2f deg", float64(t.PitchCentideg)/100, float64(t.RollCentideg)/100))
	}
	return strings.Join(parts, sep)
}

// ResultName names a calibration result.
func ResultName(r uint8) string {
	switch protocol.CalibrationResult(r) {
	case protocol.ResultAccepted:
		return "accepted"
	case protocol.ResultClamped:
		return "clamped"
	case protocol.ResultRejected:
		return "rejected"
	case protocol.ResultNoneYet:
		return "none yet"
	}
	return "result " + strconv.Itoa(int(r))
}

func formatReport(seq uint8, r protocol.CalibrationReport) string {
	reversed := 0
	for _, j := range r.Joints {
		if j.Flags&protocol.CalibrationReversed != 0 {
			reversed++
		}
	}
	return strings.Join([]string{
		"calibration report #" + strconv.Itoa(int(seq)),
		"request " + strconv.Itoa(int(r.LastRequestID)),
		ResultName(r.Result),
		"reversed " + strconv.Itoa(reversed),
	}, sep)
}

// telemetryKey is what makes a throttled telemetry frame print at once.
type telemetryKey struct {
	mode, gait uint8
	output     bool
	faults     uint16
	source     session.Source
}

func keyOf(t protocol.Telemetry) telemetryKey {
	return telemetryKey{t.BodyMode, t.Gait, t.OutputsOn, t.Faults, session.SourceFrom(t.Flags)}
}
