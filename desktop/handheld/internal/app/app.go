// Package app is the handheld's run loop without SDL (openspec change
// add-handheld-controller, designs D4, D5 and D11): the controls in, a
// command frame out every tick, the screen model, and the leave state
// machine with its three actions. cmd/spooder-handheld only feeds it the
// controls and the Menu and Power presses and draws its Screen.
package app

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
	"github.com/blackfyre/spooder/desktop/handheld/internal/udplink"
	"github.com/blackfyre/spooder/desktop/handheld/internal/wlan"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

const (
	// LeaveTimeout is the longest wait for the robot to confirm Sleep
	// before a leave acts: Sleep from walking means standing and folding
	// the legs, and once the pak stops sending the bridge's own lapse
	// makes the robot sleep anyway.
	LeaveTimeout = 3 * time.Second
	// LeaveConfirmations is the telemetry frames in a row, newer than the
	// press, that must report Sleep or output off (design D5).
	LeaveConfirmations = 2
	// StaleAfter marks the robot's values stale without telemetry.
	StaleAfter = 500 * time.Millisecond
	// SuspendScript is NextUI's suspend on the tg5040; it returns on
	// resume.
	SuspendScript = "/mnt/SDCARD/.system/tg5040/bin/suspend"
	// PowerOffCommand powers the device off.
	PowerOffCommand = "/sbin/poweroff"
)

// Action is what a leave does once Sleep is confirmed or the wait ends.
type Action int

const (
	ActionNone     Action = iota
	ActionExit            // Menu
	ActionSuspend         // a short Power press
	ActionPowerOff        // Power held 2 s
)

func (a Action) String() string {
	switch a {
	case ActionExit:
		return "exit"
	case ActionSuspend:
		return "suspend"
	case ActionPowerOff:
		return "power off"
	}
	return "none"
}

// State is the leave state machine's state.
type State int

const (
	Driving State = iota
	Leaving       // Sleep requested, waiting for its confirmation
	Acting        // the action runs (a suspend blocks here until resume)
	Done          // the run has ended
)

// Link is what the app needs of udplink.Link.
type Link interface {
	TakeEvents() []udplink.Event
	Status(now time.Time) udplink.Status
	Rebaseline()
}

// PowerSave is what the app needs of wlan.PowerSave.
type PowerSave interface {
	Off(ctx context.Context) error
	Restore(ctx context.Context) error
}

// leave is one leave in progress.
type leave struct {
	action    Action
	since     time.Time
	ref       uint8 // the link-wide newest sequence number at the press
	refValid  bool
	confirmed int // telemetry frames in a row confirming Sleep
}

// App is the run's state. Its methods are safe for concurrent use: the
// sender calls Step, the input loop SetControls and Press, the screen
// Screen.
type App struct {
	link     Link
	run      wlan.Runner
	power    PowerSave                    // nil off the device
	readFile func(string) ([]byte, error) // nil off the device: no signal level

	mu       sync.Mutex
	hand     *padmap.Handheld
	controls padmap.Controls
	desired  padmap.Desired
	begun    bool
	start    time.Time
	state    State
	leave    leave
	done     Action
	notice   string
	// The signal level from the last sample (SampleWLAN), off the screen's
	// redraw: reading it may run iw.
	signalKnown bool
	signalDBm   int
	// network is the network the run started on, "" off the device: no
	// network check (openspec change handheld-network-check). offNetwork:
	// the last sample read wlan0 on another network (nowOn, "" for none),
	// and every tick is held; rejoined: it read the started network again
	// after that, for the next step to resume.
	network    string
	offNetwork bool
	nowOn      string
	rejoined   bool
}

// New returns an app driving through link. power and readFile may be nil.
func New(link Link, run wlan.Runner, power PowerSave, readFile func(string) ([]byte, error)) *App {
	return &App{link: link, run: run, power: power, readFile: readFile, hand: padmap.NewHandheld()}
}

// SignalPeriod is how often the run samples wlan0's network and signal
// level.
const SignalPeriod = time.Second

// signalTimeout bounds one sample's iw.
const signalTimeout = 500 * time.Millisecond

// SampleWLAN reads wlan0's network and signal level (wlan.Read: one iw,
// the level from the kernel's statistics first). Off the device it does
// nothing. A failed level read makes the level unknown; a failed network
// read keeps the last verdict on the network (handheld-network-check, D3).
func (a *App) SampleWLAN(ctx context.Context) {
	if a.readFile == nil {
		return
	}
	ctx, cancel := context.WithTimeout(ctx, signalTimeout)
	defer cancel()
	s := wlan.Read(ctx, a.run, a.readFile)
	a.mu.Lock()
	defer a.mu.Unlock()
	a.signalKnown, a.signalDBm = s.SignalKnown, s.SignalDBm
	if a.network == "" || !s.NetworkKnown {
		return
	}
	off := s.Network != a.network
	if a.offNetwork && !off {
		a.rejoined = true
	}
	a.offNetwork, a.nowOn = off, s.Network
}

// ms is the mapping's clock: milliseconds since the run's first step.
func (a *App) ms(now time.Time) uint32 { return uint32(now.Sub(a.start) / time.Millisecond) }

// SetControls sets the controls the next step reads.
func (a *App) SetControls(c padmap.Controls) {
	a.mu.Lock()
	a.controls = c
	a.mu.Unlock()
}

// SetSoundOn sets the pak's sound toggle (Select + B), as kept from an
// earlier run.
func (a *App) SetSoundOn(on bool) {
	a.mu.Lock()
	a.hand.SetSoundOn(on)
	a.mu.Unlock()
}

// Press starts a leave with act (Menu: ActionExit; Power: ActionSuspend or
// ActionPowerOff). It requests Sleep at once and makes the mapping see a
// neutral pad; a press while a leave is in progress is ignored.
func (a *App) Press(act Action, now time.Time) {
	if act == ActionNone {
		return
	}
	a.mu.Lock()
	defer a.mu.Unlock()
	if a.state != Driving {
		return
	}
	st := a.link.Status(now)
	a.state = Leaving
	a.leave = leave{action: act, since: now, ref: st.Newest, refValid: st.HaveNewest}
	a.hand.RequestSleep()
	a.hand.SetNeutral(true)
}

// confirms tells whether telemetry reports the body asleep or output off.
func confirms(t protocol.Telemetry) bool {
	return session.BodyMode(t.BodyMode) == session.BodySleep || !t.OutputsOn
}

// observe takes one received frame into the leave's confirmation.
func (l *leave) observe(ev udplink.Event) {
	if ev.Rebaseline && ev.At.After(l.since) {
		// After a rebaseline (Gap of silence, received after the press)
		// the frames are taken as sent after the press: the press's
		// reference no longer orders them. A rebaselining frame received
		// before the press is the reference itself.
		l.refValid = false
	}
	if ev.Frame.Type != protocol.TypeTelemetry || ev.Late {
		return // a late frame neither counts nor breaks the run
	}
	if l.refValid {
		if d := ev.Frame.Seq - l.ref; d < 1 || d > 127 {
			return // received before the press, taken after it
		}
	}
	if confirms(ev.Frame.Telemetry) {
		l.confirmed++
	} else {
		l.confirmed = 0
	}
}

// Step is one tick at now: it takes the received frames, advances the
// leave, and returns the command and whether to send it, hold it (off the
// network the run started on) or stop (the run ended). It runs a leave's
// action itself, so a suspend blocks it until the resume.
func (a *App) Step(now time.Time) (protocol.Command, udplink.Outcome) {
	a.mu.Lock()
	if a.state == Done {
		a.mu.Unlock()
		return protocol.Command{}, udplink.Stop
	}
	if !a.begun {
		a.begun, a.start = true, now
		a.hand.Begin(a.ms(now))
	}
	if a.rejoined && !a.offNetwork {
		// Back on the network after holding (handheld-network-check, D4): as
		// after a resume, the robot frames rebaselined, Sleep requested and
		// the controls held now ignored until released. A leave in progress
		// goes on.
		a.rejoined = false
		a.link.Rebaseline()
		a.link.TakeEvents()
		a.hand.RequestSleep()
		a.hand.MaskHeld()
	}
	events := a.link.TakeEvents()
	if a.state == Leaving {
		for _, ev := range events {
			a.leave.observe(ev)
		}
		if a.leave.confirmed >= LeaveConfirmations || now.Sub(a.leave.since) >= LeaveTimeout {
			a.state = Acting
			act := a.leave.action
			a.mu.Unlock()
			return a.act(act, now)
		}
	}
	a.desired = a.hand.Step(a.controls, a.ms(now))
	cmd := a.hand.Command(a.desired)
	out := udplink.Send
	if a.offNetwork {
		out = udplink.Hold
	}
	a.mu.Unlock()
	return cmd, out
}

func (a *App) setNotice(format string, args ...any) {
	a.mu.Lock()
	a.notice = fmt.Sprintf(format, args...)
	a.mu.Unlock()
}

// restorePower restores the power-saving setting before a suspend or a
// power-off, which leave the process no chance to run its deferred
// restore.
func (a *App) restorePower(ctx context.Context) {
	if a.power == nil {
		return
	}
	if err := a.power.Restore(ctx); err != nil {
		a.setNotice("%v", err)
	}
}

// act runs a leave's action, without the lock held.
func (a *App) act(act Action, now time.Time) (protocol.Command, udplink.Outcome) {
	ctx := context.Background()
	switch act {
	case ActionSuspend:
		a.restorePower(ctx)
		if _, err := a.run.Run(ctx, SuspendScript); err != nil {
			a.setNotice("suspend: %v", err)
		}
		if a.power != nil {
			if err := a.power.Off(ctx); err != nil {
				a.setNotice("%v", err)
			}
		}
		return a.resume(now)
	case ActionPowerOff:
		a.restorePower(ctx)
		if _, err := a.run.Run(ctx, PowerOffCommand); err != nil {
			a.setNotice("power off: %v", err)
		}
	}
	a.mu.Lock()
	a.state, a.done = Done, act
	a.mu.Unlock()
	return protocol.Command{}, udplink.Stop
}

// resume returns to driving after a suspend, with Sleep requested, the
// controls held at that moment masked until released, and the link's
// sequence numbers rebaselined.
func (a *App) resume(now time.Time) (protocol.Command, udplink.Outcome) {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.link.Rebaseline()
	a.link.TakeEvents() // frames from before the suspend
	a.hand.SetNeutral(false)
	a.hand.RequestSleep()
	a.state = Driving
	a.leave = leave{}
	a.desired = a.hand.Step(a.controls, a.ms(now))
	if a.offNetwork {
		return a.hand.Command(a.desired), udplink.Hold
	}
	return a.hand.Command(a.desired), udplink.Send
}

// State is the leave state machine's state.
func (a *App) State() State {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.state
}

// Ended is the action that ended the run, ActionNone while it runs.
func (a *App) Ended() Action {
	a.mu.Lock()
	defer a.mu.Unlock()
	return a.done
}
