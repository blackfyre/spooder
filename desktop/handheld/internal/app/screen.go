package app

import (
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/internal/legview"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// Robot is the robot's side of the drive screen, from the shown telemetry.
type Robot struct {
	Have      bool // a telemetry frame has been taken
	Stale     bool // none has arrived for StaleAfter (or none yet)
	Body      string
	Gait      string
	OutputsOn bool
	Faults    []string // by name; empty when none
	Gesture   string
	GestureID uint8 // the gesture being played (protocol.Gesture*), 0 when none
	VoltageMv uint16
	CurrentMa uint16
	// Source is the robot's source in control (session.Source names): a
	// fault clear counts only from it.
	Source string
	// Simulated is the robot's bench simulation flag: it models its servos
	// and supply, and no servo moves.
	Simulated bool
}

// Pose is the robot's legs from the shown pose frame, for the driving view.
type Pose struct {
	Have  bool // a pose frame has been taken
	Stale bool // none has arrived for StaleAfter (or none yet)
	Pose  legview.Pose
	Legs  [legview.Legs]legview.Leg3 // Pose in the body frame
}

// Requested is what the pak requests.
type Requested struct {
	Mode      string
	Height    uint8
	Speed     uint8
	Gait      string
	Stride    string
	StickMode string
	EStop     bool
	// Gesture is the latest gesture request (protocol.Gesture*) and
	// GestureSeq its counter, advanced on every request.
	Gesture    uint8
	GestureSeq uint8
}

// LinkView is the link's quality (design D3).
type LinkView struct {
	// Robot frames received in the last whole second, by type.
	Telemetry, Pose, Diagnostics, CalibrationReport int
	Invalid, Lost, Late                             int
	Sent, SendErrors                                int
	HaveFrame                                       bool
	SinceLast                                       time.Duration
	SignalKnown                                     bool
	SignalDBm                                       int
}

// Screen is the drive screen's model.
type Screen struct {
	Robot     Robot
	Pose      Pose
	Requested Requested
	Link      LinkView
	State     State
	Leave     Action // the leave in progress, while State is Leaving or Acting
	Notice    string // the last error worth showing, or ""
	// Held is the controls held at the last step; Clearing the Select + Y
	// hold's progress towards its fault clear (0 to 1).
	Held     padmap.Buttons
	Clearing float64
	// DriveView shows the driving view in place of the panels (Select + X).
	DriveView bool
}

func strideName(s padmap.StrideMode) string {
	if s == padmap.StrideFixed {
		return "Fixed"
	}
	return "Dynamic"
}

func stickName(s padmap.StickMode) string {
	if s == padmap.StickCrab {
		return "Crab"
	}
	return "Steer"
}

// Screen returns the drive screen's model at now.
func (a *App) Screen(now time.Time) Screen {
	st := a.link.Status(now)
	a.mu.Lock()
	d, estop := a.desired, a.hand.EmergencyStop()
	sc := Screen{State: a.state, Notice: a.notice, Held: a.hand.Held(), DriveView: a.hand.DriveView()}
	if a.begun {
		sc.Clearing = a.hand.ClearProgress(a.ms(now))
	}
	if a.state == Leaving || a.state == Acting {
		sc.Leave = a.leave.action
	}
	signalKnown, signalDBm := a.signalKnown, a.signalDBm
	a.mu.Unlock()

	sc.Requested = Requested{
		Mode:       session.RequestedMode(d.Mode).String(),
		Height:     d.Height,
		Speed:      d.Speed,
		Gait:       session.GaitName(d.Gait),
		Stride:     strideName(d.Stride),
		StickMode:  stickName(d.StickMode),
		EStop:      estop,
		Gesture:    d.Gesture,
		GestureSeq: d.GestureRequestCounter,
	}

	tel := st.Shown[protocol.TypeTelemetry]
	sc.Robot.Have = tel.Have
	sc.Robot.Stale = !tel.Have || now.Sub(tel.ArrivedAt) >= StaleAfter
	if tel.Have {
		t := tel.Frame.Telemetry
		sc.Robot.Body = session.BodyMode(t.BodyMode).String()
		sc.Robot.Gait = session.GaitName(t.Gait)
		sc.Robot.OutputsOn = t.OutputsOn
		sc.Robot.Faults = session.FaultNames(t.Faults)
		sc.Robot.Gesture = session.GestureName(t.Gesture)
		sc.Robot.GestureID = t.Gesture
		sc.Robot.VoltageMv, sc.Robot.CurrentMa = t.VoltageMv, t.CurrentMa
		sc.Robot.Source = session.SourceFrom(t.Flags).String()
		sc.Robot.Simulated = t.Flags&protocol.TelemetrySimulated != 0
	}

	pose := st.Shown[protocol.TypePose]
	sc.Pose.Have = pose.Have
	sc.Pose.Stale = !pose.Have || now.Sub(pose.ArrivedAt) >= StaleAfter
	if pose.Have {
		sc.Pose.Pose = legview.FromFrame(pose.Frame.Pose)
		sc.Pose.Legs = legview.Body3(sc.Pose.Pose)
	}

	sc.Link = LinkView{
		Telemetry:         st.PerSecond[protocol.TypeTelemetry],
		Pose:              st.PerSecond[protocol.TypePose],
		Diagnostics:       st.PerSecond[protocol.TypeDiagnostics],
		CalibrationReport: st.PerSecond[protocol.TypeCalibrationReport],
		Invalid:           st.Invalid,
		Lost:              st.Lost,
		Late:              st.Late,
		Sent:              st.Sent,
		SendErrors:        st.SendErrors,
		HaveFrame:         st.HaveFrame,
		SinceLast:         st.SinceLast(now),
	}
	sc.Link.SignalKnown, sc.Link.SignalDBm = signalKnown, signalDBm
	return sc
}
