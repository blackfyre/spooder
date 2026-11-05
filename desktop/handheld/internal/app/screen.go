package app

import (
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
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
	VoltageMv uint16
	CurrentMa uint16
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
	Requested Requested
	Link      LinkView
	State     State
	Leave     Action // the leave in progress, while State is Leaving or Acting
	Notice    string // the last error worth showing, or ""
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
	sc := Screen{State: a.state, Notice: a.notice}
	if a.state == Leaving || a.state == Acting {
		sc.Leave = a.leave.action
	}
	a.mu.Unlock()

	sc.Requested = Requested{
		Mode:      session.RequestedMode(d.Mode).String(),
		Height:    d.Height,
		Speed:     d.Speed,
		Gait:      session.GaitName(d.Gait),
		Stride:    strideName(d.Stride),
		StickMode: stickName(d.StickMode),
		EStop:     estop,
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
		sc.Robot.VoltageMv, sc.Robot.CurrentMa = t.VoltageMv, t.CurrentMa
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
	if a.readFile != nil {
		if dbm, err := wlan.Signal(a.readFile); err == nil {
			sc.Link.SignalKnown, sc.Link.SignalDBm = true, dbm
		}
	}
	return sc
}
