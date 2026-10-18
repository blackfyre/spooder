package protocol

import (
	"fmt"
	"strconv"
	"strings"
)

// Name is the frame type's name in test/golden/protocol_v2.txt.
func (t FrameType) Name() string {
	switch t {
	case TypeCommand:
		return "command"
	case TypeTelemetry:
		return "telemetry"
	case TypeCalibrationSet:
		return "calibration_set"
	case TypeCalibrationReport:
		return "calibration_report"
	}
	return "type" + strconv.Itoa(int(t))
}

// Fields lists a decoded frame's fields as name=value, in the order and
// spelling of test/golden/protocol_v2.txt: the header fields, then the
// payload fields in payload order, values in decimal.
func Fields(f Frame) []string {
	out := []string{"version=" + strconv.Itoa(Version), "type=" + f.Type.Name(), "seq=" + strconv.Itoa(int(f.Seq))}
	add := func(name string, v int) { out = append(out, name+"="+strconv.Itoa(v)) }
	b := func(v bool) int {
		if v {
			return 1
		}
		return 0
	}
	switch f.Type {
	case TypeCommand:
		c := f.Command
		add("mode", int(c.Mode))
		add("translateX", int(c.TranslateX))
		add("translateY", int(c.TranslateY))
		add("rotate", int(c.Rotate))
		add("height", int(c.Height))
		add("speed", int(c.Speed))
		add("gait", int(c.Gait))
		add("stride", int(c.Stride))
		add("flags", int(c.Flags))
		add("clearCounter", int(c.ClearCounter))
		add("modeRequestCounter", int(c.ModeRequestCounter))
	case TypeTelemetry:
		t := f.Telemetry
		add("bodyMode", int(t.BodyMode))
		add("gait", int(t.Gait))
		add("outputsOn", b(t.OutputsOn))
		add("faults", int(t.Faults))
		add("voltageMv", int(t.VoltageMv))
		add("currentMa", int(t.CurrentMa))
		add("validFrames", int(t.ValidFrames))
		add("discardedFrames", int(t.DiscardedFrames))
		add("sensorsAvailable", int(t.SensorsAvailable))
		add("tofMm", int(t.ToFMm))
		add("tofStatus", int(t.ToFStatus))
		add("pitchCentideg", int(t.PitchCentideg))
		add("rollCentideg", int(t.RollCentideg))
		add("flags", int(t.Flags))
	case TypeCalibrationSet:
		c := f.CalibrationSet
		add("requestId", int(c.RequestID))
		add("joint", int(c.Joint))
		add("offsetCentideg", int(c.OffsetCentideg))
		add("pulse0Us", int(c.Pulse0Us))
		add("pulse180Us", int(c.Pulse180Us))
		add("flags", int(c.Flags))
	case TypeCalibrationReport:
		r := f.CalibrationReport
		add("lastRequestId", int(r.LastRequestID))
		add("result", int(r.Result))
		for i, j := range r.Joints {
			p := fmt.Sprintf("joint%d.", i)
			add(p+"offsetCentideg", int(j.OffsetCentideg))
			add(p+"pulse0Us", int(j.Pulse0Us))
			add(p+"pulse180Us", int(j.Pulse180Us))
			add(p+"flags", int(j.Flags))
		}
	}
	return out
}

// Encode re-encodes a decoded frame.
func Encode(f Frame) []byte {
	switch f.Type {
	case TypeCommand:
		return EncodeCommand(f.Command, f.Seq)
	case TypeTelemetry:
		return EncodeTelemetry(f.Telemetry, f.Seq)
	case TypeCalibrationSet:
		return EncodeCalibrationSet(f.CalibrationSet, f.Seq)
	case TypeCalibrationReport:
		return EncodeCalibrationReport(f.CalibrationReport, f.Seq)
	}
	return nil
}

// Summary is a one-line description of a frame for a log view.
func Summary(f Frame) string {
	fields := Fields(f)[2:] // without version and type
	if f.Type == TypeCalibrationReport {
		fields = fields[:3]
	}
	return f.Type.Name() + " " + strings.Join(fields, " ")
}
