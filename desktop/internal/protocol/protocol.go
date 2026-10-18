// Package protocol is the Go implementation of the Spooder link protocol,
// version 2: frame layout, CRC, encoders and an incremental decoder. It
// mirrors core/protocol/protocol.{h,cpp} byte for byte; both are pinned by
// test/golden/protocol_v2.txt.
//
// Frame:
//
//	0      start marker 0xA5
//	1      protocol version
//	2      type (1 = command, 2 = telemetry, 3 = calibration set,
//	       4 = calibration report, 5 = pose)
//	3      sequence number (wraps)
//	4      payload length N (fixed per type and version)
//	5      payload, N bytes, little-endian
//	5+N    CRC-16/CCITT-FALSE over bytes 1..4+N, little-endian
package protocol

const (
	Start   = 0xA5
	Version = 2
)

// FrameType is the frame's type byte.
type FrameType uint8

const (
	TypeCommand           FrameType = 1
	TypeTelemetry         FrameType = 2
	TypeCalibrationSet    FrameType = 3
	TypeCalibrationReport FrameType = 4
	TypePose              FrameType = 5 // robot to the USB host only
)

const (
	HeaderSize                   = 5
	CRCSize                      = 2
	CommandPayloadSize           = 12
	TelemetryPayloadSize         = 32
	CalibrationJoints            = 18
	CalibrationJointSize         = 7
	CalibrationSetPayloadSize    = 9
	CalibrationReportPayloadSize = 2 + CalibrationJoints*CalibrationJointSize // 128
	PoseJointSize                = 6
	PosePayloadSize              = 2 + CalibrationJoints*PoseJointSize // 110
	CommandFrameSize             = HeaderSize + CommandPayloadSize + CRCSize
	TelemetryFrameSize           = HeaderSize + TelemetryPayloadSize + CRCSize
	CalibrationSetFrameSize      = HeaderSize + CalibrationSetPayloadSize + CRCSize
	CalibrationReportFrameSize   = HeaderSize + CalibrationReportPayloadSize + CRCSize
	PoseFrameSize                = HeaderSize + PosePayloadSize + CRCSize
	MaxFrameSize                 = CalibrationReportFrameSize // 135
)

// Command flags.
const (
	FlagControllerConnected uint8 = 1 << 0
	FlagEmergencyStop       uint8 = 1 << 1
)

// Command is sent to the robot every 20 ms: the complete desired state,
// never events.
type Command struct {
	Mode               uint8 // RequestedMode value
	TranslateX         int8  // -100..100
	TranslateY         int8  // -100..100, + = forward
	Rotate             int8  // -100..100
	Height             uint8 // 0..100
	Speed              uint8 // 0..100
	Gait               uint8 // gait index
	Stride             uint8 // StrideMode value
	Flags              uint8 // Flag*
	ClearCounter       uint8 // incremented once per fault-clear request
	ModeRequestCounter uint8 // incremented on every explicit mode request
}

// Sensor availability bits.
const (
	SensorToF      uint8 = 1 << 0
	SensorAttitude uint8 = 1 << 1
)

// Telemetry flags.
const (
	TelemetrySourceConflict  uint8 = 1 << 0 // both links claim control: the robot sleeps
	TelemetryUSBInControl    uint8 = 1 << 1
	TelemetryBridgeInControl uint8 = 1 << 2
)

// Telemetry is sent by the robot to every link, every 100 ms.
type Telemetry struct {
	BodyMode         uint8 // body::Mode value
	Gait             uint8
	OutputsOn        bool
	Faults           uint16 // protection::Fault bits
	VoltageMv        uint16
	CurrentMa        uint16
	ValidFrames      uint16 // command frames accepted (wraps)
	DiscardedFrames  uint16 // frames rejected (wraps)
	SensorsAvailable uint8  // Sensor* bits; the fields below are meaningful only when set
	ToFMm            uint16
	ToFStatus        uint8
	PitchCentideg    int16
	RollCentideg     int16
	Flags            uint8 // Telemetry* bits
}

// CalibrationReversed is the calibration set flag and report joint flag for
// a reversed joint.
const CalibrationReversed uint8 = 1 << 0

// CalibrationSet asks for the complete calibration of one joint.
type CalibrationSet struct {
	RequestID      uint8 // a repeat on the same link is a retransmission
	Joint          uint8 // 0..17, servo output order
	OffsetCentideg int16
	Pulse0Us       uint16
	Pulse180Us     uint16
	Flags          uint8 // CalibrationReversed
}

// CalibrationResult is the result of the last calibration set processed
// from a link.
type CalibrationResult uint8

const (
	ResultAccepted CalibrationResult = 0
	ResultClamped  CalibrationResult = 1 // accepted after clamping to the safe range
	ResultRejected CalibrationResult = 2 // not in Calibrate, or output not allowed
	ResultNoneYet  CalibrationResult = 3 // no calibration set processed from this link
)

// CalibrationJoint is one joint of a calibration report.
type CalibrationJoint struct {
	OffsetCentideg int16
	Pulse0Us       uint16
	Pulse180Us     uint16
	Flags          uint8 // CalibrationReversed
}

// CalibrationReport carries the robot's calibration table in effect, and
// the result of the last calibration set processed from the link it is
// sent on.
type CalibrationReport struct {
	LastRequestID uint8
	Result        uint8 // CalibrationResult value
	Joints        [CalibrationJoints]CalibrationJoint
}

// PoseOutputsOn is the pose flag for servo output on.
const PoseOutputsOn uint8 = 1 << 0

// PoseJoint is one joint of a pose frame.
type PoseJoint struct {
	CommandedCentideg int16  // IK result, before the calibration offset
	ServoCentideg     int16  // commanded + applied offset, before the 0–180° clamp
	PulseUs           uint16 // last pulse set for the output; 0 = never set
}

// Pose is what the robot commands each joint, sent to the USB host only,
// at 10 Hz.
type Pose struct {
	Flags  uint8 // Pose* bits
	Joints [CalibrationJoints]PoseJoint
}

// CRC16 is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection).
func CRC16(data []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, b := range data {
		crc ^= uint16(b) << 8
		for bit := 0; bit < 8; bit++ {
			if crc&0x8000 != 0 {
				crc = crc<<1 ^ 0x1021
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

func put16(p []byte, v uint16) {
	p[0] = byte(v)
	p[1] = byte(v >> 8)
}

func get16(p []byte) uint16 {
	return uint16(p[0]) | uint16(p[1])<<8
}

// PayloadSize is the payload length of a frame type, or -1 for an unknown
// type.
func PayloadSize(t FrameType) int {
	switch t {
	case TypeCommand:
		return CommandPayloadSize
	case TypeTelemetry:
		return TelemetryPayloadSize
	case TypeCalibrationSet:
		return CalibrationSetPayloadSize
	case TypeCalibrationReport:
		return CalibrationReportPayloadSize
	case TypePose:
		return PosePayloadSize
	}
	return -1
}

// newFrame returns a frame of type t with its header filled in and a zero
// payload, and the payload slice.
func newFrame(t FrameType, seq uint8) (frame, payload []byte) {
	n := PayloadSize(t)
	frame = make([]byte, HeaderSize+n+CRCSize)
	frame[0] = Start
	frame[1] = Version
	frame[2] = byte(t)
	frame[3] = seq
	frame[4] = byte(n)
	return frame, frame[HeaderSize : HeaderSize+n]
}

func finish(frame []byte) []byte {
	n := len(frame) - HeaderSize - CRCSize
	put16(frame[HeaderSize+n:], CRC16(frame[1:HeaderSize+n]))
	return frame
}

// EncodeCommand returns the complete command frame.
func EncodeCommand(c Command, seq uint8) []byte {
	f, p := newFrame(TypeCommand, seq)
	p[0] = c.Mode
	p[1] = byte(c.TranslateX)
	p[2] = byte(c.TranslateY)
	p[3] = byte(c.Rotate)
	p[4] = c.Height
	p[5] = c.Speed
	p[6] = c.Gait
	p[7] = c.Stride
	p[8] = c.Flags
	p[9] = c.ClearCounter
	p[10] = c.ModeRequestCounter
	// p[11] reserved, zero
	return finish(f)
}

// EncodeTelemetry returns the complete telemetry frame.
func EncodeTelemetry(t Telemetry, seq uint8) []byte {
	f, p := newFrame(TypeTelemetry, seq)
	p[0] = t.BodyMode
	p[1] = t.Gait
	if t.OutputsOn {
		p[2] = 1
	}
	put16(p[3:], t.Faults)
	put16(p[5:], t.VoltageMv)
	put16(p[7:], t.CurrentMa)
	put16(p[9:], t.ValidFrames)
	put16(p[11:], t.DiscardedFrames)
	p[13] = t.SensorsAvailable
	put16(p[14:], t.ToFMm)
	p[16] = t.ToFStatus
	put16(p[17:], uint16(t.PitchCentideg))
	put16(p[19:], uint16(t.RollCentideg))
	p[21] = t.Flags
	return finish(f)
}

// EncodeCalibrationSet returns the complete calibration set frame.
func EncodeCalibrationSet(c CalibrationSet, seq uint8) []byte {
	f, p := newFrame(TypeCalibrationSet, seq)
	p[0] = c.RequestID
	p[1] = c.Joint
	put16(p[2:], uint16(c.OffsetCentideg))
	put16(p[4:], c.Pulse0Us)
	put16(p[6:], c.Pulse180Us)
	p[8] = c.Flags
	return finish(f)
}

// EncodeCalibrationReport returns the complete calibration report frame.
func EncodeCalibrationReport(r CalibrationReport, seq uint8) []byte {
	f, p := newFrame(TypeCalibrationReport, seq)
	p[0] = r.LastRequestID
	p[1] = r.Result
	for i, j := range r.Joints {
		b := p[2+i*CalibrationJointSize:]
		put16(b, uint16(j.OffsetCentideg))
		put16(b[2:], j.Pulse0Us)
		put16(b[4:], j.Pulse180Us)
		b[6] = j.Flags
	}
	return finish(f)
}

// EncodePose returns the complete pose frame.
func EncodePose(p Pose, seq uint8) []byte {
	f, b := newFrame(TypePose, seq)
	b[0] = p.Flags
	// b[1] reserved, zero
	for i, j := range p.Joints {
		o := b[2+i*PoseJointSize:]
		put16(o, uint16(j.CommandedCentideg))
		put16(o[2:], uint16(j.ServoCentideg))
		put16(o[4:], j.PulseUs)
	}
	return finish(f)
}

func parseCommand(p []byte) Command {
	return Command{
		Mode: p[0], TranslateX: int8(p[1]), TranslateY: int8(p[2]), Rotate: int8(p[3]),
		Height: p[4], Speed: p[5], Gait: p[6], Stride: p[7], Flags: p[8],
		ClearCounter: p[9], ModeRequestCounter: p[10],
	}
}

func parseTelemetry(p []byte) Telemetry {
	return Telemetry{
		BodyMode: p[0], Gait: p[1], OutputsOn: p[2] != 0,
		Faults: get16(p[3:]), VoltageMv: get16(p[5:]), CurrentMa: get16(p[7:]),
		ValidFrames: get16(p[9:]), DiscardedFrames: get16(p[11:]),
		SensorsAvailable: p[13], ToFMm: get16(p[14:]), ToFStatus: p[16],
		PitchCentideg: int16(get16(p[17:])), RollCentideg: int16(get16(p[19:])),
		Flags: p[21],
	}
}

func parseCalibrationSet(p []byte) CalibrationSet {
	return CalibrationSet{
		RequestID: p[0], Joint: p[1], OffsetCentideg: int16(get16(p[2:])),
		Pulse0Us: get16(p[4:]), Pulse180Us: get16(p[6:]), Flags: p[8],
	}
}

func parseCalibrationReport(p []byte) CalibrationReport {
	r := CalibrationReport{LastRequestID: p[0], Result: p[1]}
	for i := range r.Joints {
		b := p[2+i*CalibrationJointSize:]
		r.Joints[i] = CalibrationJoint{
			OffsetCentideg: int16(get16(b)), Pulse0Us: get16(b[2:]), Pulse180Us: get16(b[4:]), Flags: b[6],
		}
	}
	return r
}

func parsePose(b []byte) Pose {
	p := Pose{Flags: b[0]}
	for i := range p.Joints {
		o := b[2+i*PoseJointSize:]
		p.Joints[i] = PoseJoint{
			CommandedCentideg: int16(get16(o)), ServoCentideg: int16(get16(o[2:])), PulseUs: get16(o[4:]),
		}
	}
	return p
}

// Frame is one decoded frame; only the field of its Type is meaningful.
type Frame struct {
	Type              FrameType
	Seq               uint8
	Command           Command
	Telemetry         Telemetry
	CalibrationSet    CalibrationSet
	CalibrationReport CalibrationReport
	Pose              Pose
}
