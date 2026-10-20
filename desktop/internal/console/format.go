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
	case protocol.TypeDiagnostics:
		return formatDiagnostics(f.Seq, f.Diagnostics)
	}
	return protocol.Summary(f)
}

// formatDiagnostics is the robot's internals: the budget's motion share,
// the backstop count, the tick's working time and the averaged current.
func formatDiagnostics(seq uint8, d protocol.Diagnostics) string {
	return strings.Join([]string{
		fmt.Sprintf("diagnostics #%d", seq),
		fmt.Sprintf("motion %.1f %%", float64(d.MotionSharePermille)/10),
		fmt.Sprintf("backstop %d", d.BackstopTicks),
		fmt.Sprintf("tick %.2f ms max %.2f ms mean", float64(d.TickMaxUs)/1000, float64(d.TickMeanUs)/1000),
		fmt.Sprintf("average %.2f A", float64(d.AverageCurrentMa)/1000),
	}, sep)
}

// bridgeText is the bridge link as the robot reports it.
func bridgeText(t protocol.Telemetry) string {
	if t.Flags&protocol.TelemetryBridgeLinkUp == 0 {
		return "bridge down"
	}
	pad := "no gamepad"
	if t.Flags&protocol.TelemetryBridgeGamepad != 0 {
		pad = "gamepad"
	}
	return fmt.Sprintf("bridge up (%s) %d ok %d bad", pad, t.BridgeValidFrames, t.BridgeDiscardedFrames)
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
	parts = append(parts, bridgeText(t))
	if t.Gesture != protocol.GestureNone {
		gesture := session.GestureName(t.Gesture)
		if !strings.HasPrefix(gesture, "gesture ") { // an unknown id is named "gesture N"
			gesture = "gesture " + gesture
		}
		parts = append(parts, gesture)
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
	bridge     uint8 // the TelemetryBridge* bits: a change of the link or its gamepad
	gesture    uint8
}

func keyOf(t protocol.Telemetry) telemetryKey {
	return telemetryKey{t.BodyMode, t.Gait, t.OutputsOn, t.Faults, session.SourceFrom(t.Flags),
		t.Flags & (protocol.TelemetryBridgeLinkUp | protocol.TelemetryBridgeGamepad), t.Gesture}
}
