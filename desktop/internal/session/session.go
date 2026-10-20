// Package session is the desktop calibrator's single state owner (design
// D4): what the link has read (telemetry, calibration reports, debug text),
// what the operator asked for (control, e-stop, mode and fault-clear
// requests, joint edits), and therefore what the link writes. The UI reads
// copies through Snapshot and never touches the port.
//
// Control semantics:
//   - Observe (default): command frames report no controller and request
//     Sleep with zero input, so the robot ignores the app for movement but
//     accepts its e-stop.
//   - Control: frames report a connected controller; Stand, Calibrate and
//     Sleep advance the mode-request counter, Clear fault the clear counter
//     and a gesture the gesture-request counter. Held drive keys request
//     Walk from Stand and set translate and rotate while walking; the
//     walking settings (speed, height, gait, stride) travel in every frame.
//   - Releasing control, losing the connection or shutting down stops
//     reporting a controller and abandons unconfirmed joint edits.
package session

import (
	"errors"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// StaleAfter marks telemetry as not current (the robot's own stale-command
// rule uses the same second).
const StaleAfter = time.Second

// Command defaults while observing, and the walking settings' starting
// values, as the bridge's gamepad mapping starts: body height 0, speed 44,
// gait Tri, dynamic stride.
const (
	commandHeight = 0
	commandSpeed  = 44
)

// Log sizes.
const (
	MaxLogLines   = 2000
	MaxFrameLines = 200
)

// Errors for refused operator actions.
var (
	ErrNotConnected = errors.New("not connected")
	ErrNotInControl = errors.New("take control first")
	ErrNotEditable  = errors.New("editing needs control, a current Calibrate report from the robot, and its calibration table")
)

// LogLine is one line of the robot's debug text.
type LogLine struct {
	At    time.Time
	Text  string
	Stray int // 0xA5 bytes outside frames in this line
}

// FrameLine is one decoded frame, for the frames view.
type FrameLine struct {
	At   time.Time
	Text string
}

// Session is safe for concurrent use: the link calls it from its
// goroutines, the UI from its own.
type Session struct {
	now      func() time.Time
	onChange func()

	mu           sync.Mutex
	link         link.State
	control      bool
	estop        bool
	mode         RequestedMode
	modeCounter  uint8
	clearCounter uint8
	// The latest gesture request and its counter (protocol.Gesture* ids).
	gesture        uint8
	gestureCounter uint8
	// Driving: the held drive keys and the walking settings.
	drive                       Drive
	speed, height, gait, stride uint8
	telemetry                   protocol.Telemetry
	telemetryAt                 time.Time
	haveTelemetry               bool
	stats                       link.Stats
	calib                       calib.Model
	reportAt                    time.Time
	pose                        protocol.Pose
	poseAt                      time.Time
	havePose                    bool
	history                     History
	historyVersion              uint64
	log                         []LogLine
	frames                      []FrameLine
	strayLines                  int
}

// New returns a session in observe mode. now defaults to time.Now; onChange,
// if set, is called (without locks held) after every change, for example to
// redraw the window.
func New(now func() time.Time, onChange func()) *Session {
	if now == nil {
		now = time.Now
	}
	return &Session{now: now, onChange: onChange, mode: RequestSleep, speed: commandSpeed, height: commandHeight}
}

func (s *Session) changed() {
	if s.onChange != nil {
		s.onChange()
	}
}

// update runs f under the lock, then reports a change.
func (s *Session) update(f func() error) error {
	s.mu.Lock()
	err := f()
	s.mu.Unlock()
	s.changed()
	return err
}

func (s *Session) connected() bool { return s.link.Status == link.Connected }

// release returns to observing; s.mu is held.
func (s *Session) release() {
	s.control = false
	s.mode = RequestSleep
	s.drive = Drive{}
	s.calib.DropPending()
}

// LinkState implements link.Handler.
func (s *Session) LinkState(st link.State) {
	_ = s.update(func() error {
		wasConnected := s.connected()
		s.link = st
		if st.Status != link.Connected {
			s.release()
		} else if !wasConnected {
			// A new connection: its own counts, and no table until the
			// robot reports one.
			s.stats = link.Stats{}
			s.calib.Forget()
			s.haveTelemetry = false
			s.havePose = false
			s.history = History{Start: s.now()}
			s.historyVersion++
		}
		return nil
	})
}

// Frame implements link.Handler.
func (s *Session) Frame(f protocol.Frame, st link.Stats) {
	_ = s.update(func() error {
		now := s.now()
		s.stats = st
		switch f.Type {
		case protocol.TypeTelemetry:
			s.telemetry, s.telemetryAt, s.haveTelemetry = f.Telemetry, now, true
			s.history.addTelemetry(TelemetrySample{At: now, T: f.Telemetry, Host: st})
			s.historyVersion++
		case protocol.TypeDiagnostics:
			s.history.addDiagnostics(DiagnosticsSample{At: now, D: f.Diagnostics})
			s.historyVersion++
		case protocol.TypeCalibrationReport:
			s.calib.Report(f.CalibrationReport)
			s.reportAt = now
		case protocol.TypePose:
			s.pose, s.poseAt, s.havePose = f.Pose, now, true
			s.history.addPose(now, f.Pose)
			s.historyVersion++
		}
		s.frames = appendCapped(s.frames, FrameLine{At: now, Text: protocol.Summary(f)}, MaxFrameLines)
		return nil
	})
}

// Line implements link.Handler.
func (s *Session) Line(l link.Line, st link.Stats) {
	_ = s.update(func() error {
		s.stats = st
		if l.Stray > 0 {
			s.strayLines++
		}
		s.log = appendCapped(s.log, LogLine{At: s.now(), Text: l.Text, Stray: l.Stray}, MaxLogLines)
		return nil
	})
}

func appendCapped[T any](list []T, v T, max int) []T {
	if len(list) >= max {
		list = append(list[:0], list[len(list)-max+1:]...)
	}
	return append(list, v)
}

// Command implements link.Source: the frame for the current state.
func (s *Session) Command() protocol.Command {
	s.mu.Lock()
	defer s.mu.Unlock()
	c := protocol.Command{
		Mode:               uint8(RequestSleep),
		Height:             commandHeight,
		Speed:              commandSpeed,
		ClearCounter:       s.clearCounter,
		ModeRequestCounter: s.modeCounter,
		// The robot counts a gesture request only from the source in
		// control, so the counter can travel in every frame.
		Gesture:               s.gesture,
		GestureRequestCounter: s.gestureCounter,
	}
	if s.control && s.connected() {
		c.Flags |= protocol.FlagControllerConnected
		c.Mode = uint8(s.mode)
		s.drives(&c)
	}
	if s.estop {
		c.Flags |= protocol.FlagEmergencyStop
	}
	return c
}

// CalibrationSet implements link.Source: the next joint change to send,
// only while the app is in control.
func (s *Session) CalibrationSet(now time.Time) (protocol.CalibrationSet, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !s.control || !s.connected() {
		return protocol.CalibrationSet{}, false
	}
	return s.calib.Next(now)
}

// TakeControl makes the app's frames report a connected controller,
// requesting Sleep until a mode is chosen.
func (s *Session) TakeControl() error {
	return s.update(func() error {
		if !s.connected() {
			return ErrNotConnected
		}
		if !s.control {
			s.control = true
			s.mode = RequestSleep
		}
		return nil
	})
}

// ReleaseControl returns to observing; the robot then sleeps unless the
// bridge controls it.
func (s *Session) ReleaseControl() {
	_ = s.update(func() error {
		s.release()
		return nil
	})
}

// Shutdown releases control before the link sends its last frame.
func (s *Session) Shutdown() { s.ReleaseControl() }

// RequestMode asks for Stand, Calibrate or Sleep; each call is one mode
// request.
func (s *Session) RequestMode(m RequestedMode) error {
	return s.update(func() error {
		if !s.control || !s.connected() {
			return ErrNotInControl
		}
		switch m {
		case RequestStand, RequestCalibrate, RequestSleep:
		default:
			return errors.New("walk by driving: hold a drive key in Stand")
		}
		s.mode = m
		s.modeCounter++
		return nil
	})
}

// RequestGesture asks the robot to play gesture g (a protocol.Gesture* id,
// not GestureNone) once. Only in control; the robot plays it from a settled
// Stand and ignores it otherwise.
func (s *Session) RequestGesture(g uint8) error {
	return s.update(func() error {
		if !s.control || !s.connected() {
			return ErrNotInControl
		}
		if g == protocol.GestureNone || g >= protocol.GestureCount {
			return errors.New("no such gesture")
		}
		s.gesture = g
		s.gestureCounter++
		return nil
	})
}

// ToggleEmergencyStop sets or clears the e-stop flag, in or out of
// control. Clearing it does not clear the robot's latched fault.
func (s *Session) ToggleEmergencyStop() {
	_ = s.update(func() error {
		s.estop = !s.estop
		return nil
	})
}

// ClearFault asks the robot to clear its latched faults; only the source in
// control may.
func (s *Session) ClearFault() error {
	return s.update(func() error {
		if !s.control || !s.connected() {
			return ErrNotInControl
		}
		s.clearCounter++
		return nil
	})
}

// editable is the editing gate; s.mu is held.
func (s *Session) editable() bool {
	return s.control && s.connected() && s.fresh() &&
		BodyMode(s.telemetry.BodyMode) == BodyCalibrate && s.calib.HaveReport()
}

func (s *Session) fresh() bool {
	return s.haveTelemetry && s.now().Sub(s.telemetryAt) <= StaleAfter
}

// Nudge changes one joint field by steps fine or coarse increments.
func (s *Session) Nudge(joint int, f calib.Field, steps int, coarse bool) error {
	return s.update(func() error {
		if !s.editable() {
			return ErrNotEditable
		}
		s.calib.Nudge(joint, f, steps, coarse)
		return nil
	})
}

// JointView is one joint as the editor shows it.
type JointView struct {
	Reported calib.Joint
	Pending  bool
	Wanted   calib.Joint // valid when Pending
	Result   calib.Result
}

// Snapshot is a copy of the state for drawing.
type Snapshot struct {
	Now  time.Time
	Link link.State

	Control       bool
	EmergencyStop bool // the app's e-stop flag
	Mode          RequestedMode
	// Driving: the held keys and the walking settings the app sends in
	// control.
	Drive                       Drive
	Speed, Height, Gait, Stride uint8
	CanControl                  bool // Take control is possible
	ModesEnabled                bool // Stand, Calibrate, Sleep and Clear fault are live
	Editable                    bool

	HaveTelemetry bool
	Fresh         bool // telemetry no older than StaleAfter
	TelemetryAge  time.Duration
	Telemetry     protocol.Telemetry
	Body          BodyMode
	Gesture       uint8 // the gesture being played (protocol.Gesture*), 0 when none
	Source        Source
	Faults        []string

	Stats      link.Stats
	StrayLines int

	// The latest pose frame on this connection; PoseFresh when it is no
	// older than StaleAfter.
	HavePose  bool
	PoseFresh bool
	PoseAge   time.Duration
	Pose      protocol.Pose

	HaveReport   bool
	ReportAge    time.Duration
	Joints       [calib.Joints]JointView
	Table        [calib.Joints]calib.Joint
	PendingCount int

	Log    []LogLine
	Frames []FrameLine
}

// Snapshot copies the current state.
func (s *Session) Snapshot() Snapshot {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	snap := Snapshot{
		Now:           now,
		Link:          s.link,
		Control:       s.control,
		EmergencyStop: s.estop,
		Mode:          s.mode,
		Drive:         s.drive,
		Speed:         s.speed,
		Height:        s.height,
		Gait:          s.gait,
		Stride:        s.stride,
		CanControl:    s.connected(),
		ModesEnabled:  s.control && s.connected(),
		Editable:      s.editable(),
		HaveTelemetry: s.haveTelemetry,
		Fresh:         s.fresh(),
		Telemetry:     s.telemetry,
		Body:          BodyMode(s.telemetry.BodyMode),
		Gesture:       s.telemetry.Gesture,
		Source:        SourceFrom(s.telemetry.Flags),
		Faults:        FaultNames(s.telemetry.Faults),
		Stats:         s.stats,
		StrayLines:    s.strayLines,
		HaveReport:    s.calib.HaveReport(),
		Table:         s.calib.Table(),
		PendingCount:  s.calib.PendingCount(),
		Log:           append([]LogLine(nil), s.log...),
		Frames:        append([]FrameLine(nil), s.frames...),
	}
	if s.haveTelemetry {
		snap.TelemetryAge = now.Sub(s.telemetryAt)
	}
	if snap.HaveReport {
		snap.ReportAge = now.Sub(s.reportAt)
	}
	if s.havePose {
		snap.HavePose, snap.Pose, snap.PoseAge = true, s.pose, now.Sub(s.poseAt)
		snap.PoseFresh = snap.PoseAge <= StaleAfter
	}
	for i := range snap.Joints {
		v := JointView{Reported: s.calib.Reported(i), Result: s.calib.Result(i)}
		v.Wanted, v.Pending = s.calib.Pending(i)
		snap.Joints[i] = v
	}
	return snap
}
