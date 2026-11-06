// Package feedback is the pak's sound feedback (openspec change
// handheld-observations, design D4): which events sound, judged from the
// drive screen's model as the bridge's rumble judges them
// (core/gamepad_mapping/rumble), the tone each plays, the 1 s repeat guard
// and the priorities, and the tones rendered as 16-bit PCM. The audio output
// is the Player interface: cmd/spooder-handheld plays through SDL, the tests
// through a recorder, so nothing here needs a device.
package feedback

import (
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
)

// Event is what sounds.
type Event int

const (
	None            Event = iota
	FaultLatched          // a protection fault newly latched in fresh telemetry (the e-stop's excepted)
	EstopOn               // the pak's e-stop switched on
	LinkLost              // telemetry went stale
	Leaving               // a leave started (Menu or Power)
	GestureRefused        // a gesture request not playing RefusedAfter later
	EstopOff              // the pak's e-stop switched off
	LinkRegained          // telemetry fresh again
	GestureAccepted       // the robot plays the gesture requested
	eventCount
)

func (e Event) String() string {
	return [...]string{"none", "fault latched", "e-stop on", "link lost", "leaving", "gesture refused",
		"e-stop off", "link regained", "gesture accepted"}[e]
}

const (
	// RepeatAfter is the shortest time between two plays of one event, and
	// how long a detected event waits to play before it is dropped (rumble's
	// REPEAT_MS).
	RepeatAfter = time.Second
	// RefusedAfter is how long a gesture request may go without the robot
	// playing it (rumble's GESTURE_REFUSED_MS).
	RefusedAfter = 600 * time.Millisecond
	// faultEstop is protection::EMERGENCY_STOP, which has its own sound.
	faultEstop = 0x20
)

// Inputs is what the detector reads at one moment.
type Inputs struct {
	Now            time.Time
	TelemetryFresh bool
	Faults         uint16 // the robot's latched fault bits
	EStop          bool   // the pak's own e-stop
	GestureSeq     uint8  // the pak's gesture request counter
	Gesture        uint8  // the gesture last requested
	Playing        uint8  // the gesture telemetry reports playing
	Leaving        bool   // a leave in progress
}

// FromScreen takes the inputs from the drive screen's model at now.
func FromScreen(sc app.Screen, now time.Time) Inputs {
	return Inputs{
		Now:            now,
		TelemetryFresh: sc.Robot.Have && !sc.Robot.Stale,
		Faults:         sc.Robot.FaultBits,
		EStop:          sc.Requested.EStop,
		GestureSeq:     sc.Requested.GestureSeq,
		Gesture:        sc.Requested.Gesture,
		Playing:        sc.Robot.GestureID,
		Leaving:        sc.State == app.Leaving || sc.State == app.Acting,
	}
}

// Detector turns successive Inputs into events to play, one at a time:
// a more important event replaces a less important one still playing, a
// less important one waits for it, and an event plays at most once per
// RepeatAfter.
type Detector struct {
	started        bool
	seen           bool // fresh telemetry taken since the run started or the last leave ended
	wasFresh       bool
	faults         uint16
	estop          bool
	leaving        bool
	gestureSeq     uint8
	gesturePending bool
	pendingGesture uint8
	sawNotPlaying  bool
	gestureAt      time.Time

	waiting   [eventCount]bool
	waitingAt [eventCount]time.Time
	played    [eventCount]bool
	lastAt    [eventCount]time.Time
	playing   Event
	playUntil time.Time
}

func (d *Detector) detected(e Event, now time.Time) {
	d.waiting[e] = true
	d.waitingAt[e] = now
}

// Update takes in and returns the event to play now, or None. The first
// call only takes what is already so (the e-stop, the gesture counter) as
// known; the first fresh telemetry since the run started, or since a leave
// ended (a resume), is the baseline (its latched faults), not the link
// regained: the link is lost only after telemetry was fresh, and regained
// only after it was lost.
func (d *Detector) Update(in Inputs) Event {
	now := in.Now
	if !d.started {
		d.started = true
		d.seen = in.TelemetryFresh && !in.Leaving
		d.wasFresh = d.seen
		if d.seen {
			d.faults = in.Faults
		}
		d.estop, d.leaving, d.gestureSeq = in.EStop, in.Leaving, in.GestureSeq
		return None
	}

	// What happened.
	if in.EStop != d.estop {
		if in.EStop {
			d.detected(EstopOn, now)
		} else {
			d.detected(EstopOff, now)
		}
	}
	d.estop = in.EStop
	if in.Leaving && !d.leaving {
		d.detected(Leaving, now)
	}
	d.leaving = in.Leaving

	switch {
	case in.Leaving:
		// A leave pauses the link's judging, and what follows it (a resume
		// after a suspend) starts anew, as a run does.
		d.seen = false
	case in.TelemetryFresh && !d.seen:
		d.seen = true
		d.faults = in.Faults // the baseline at the start
	case in.TelemetryFresh && !d.wasFresh:
		d.detected(LinkRegained, now)
		d.faults = in.Faults // the baseline after a gap
	case !in.TelemetryFresh && d.wasFresh:
		d.detected(LinkLost, now)
	case in.TelemetryFresh:
		if in.Faults&^d.faults&^faultEstop != 0 {
			d.detected(FaultLatched, now)
		}
		d.faults = in.Faults
	}
	d.wasFresh = in.TelemetryFresh && d.seen

	if in.GestureSeq != d.gestureSeq {
		d.gestureSeq = in.GestureSeq
		d.gesturePending = true
		d.pendingGesture = in.Gesture
		// The same gesture already playing does not count as acting on this one.
		d.sawNotPlaying = in.Playing != in.Gesture
		d.gestureAt = now
	}
	if d.gesturePending {
		switch {
		case !in.TelemetryFresh:
			d.gesturePending = false // the link loss says it all
		case in.Playing != d.pendingGesture:
			d.sawNotPlaying = true
		}
		if d.gesturePending && d.sawNotPlaying && in.Playing == d.pendingGesture {
			d.gesturePending = false
			d.detected(GestureAccepted, now)
		} else if d.gesturePending && now.Sub(d.gestureAt) >= RefusedAfter {
			d.gesturePending = false
			d.detected(GestureRefused, now)
		}
	}

	// What plays.
	if d.playing != None && !now.Before(d.playUntil) {
		d.playing = None
	}
	best := None
	for e := Event(1); e < eventCount; e++ {
		if !d.waiting[e] {
			continue
		}
		// Too old, or too soon after the last time it played: dropped.
		if now.Sub(d.waitingAt[e]) >= RepeatAfter || (d.played[e] && d.waitingAt[e].Sub(d.lastAt[e]) < RepeatAfter) {
			d.waiting[e] = false
			continue
		}
		if best == None || ToneFor(e).Priority > ToneFor(best).Priority {
			best = e
		}
	}
	if best == None {
		return None
	}
	t := ToneFor(best)
	if d.playing != None && ToneFor(d.playing).Priority > t.Priority {
		return None // it waits for the more important sound
	}
	d.waiting[best] = false
	d.played[best] = true
	d.lastAt[best] = now
	d.playing = best
	d.playUntil = now.Add(t.Duration())
	return best
}

// Player plays PCM: 16-bit signed mono samples at SampleRate, replacing
// whatever still plays.
type Player interface {
	Play(pcm []int16) error
}

// Feedback is the detector with the rendered tones and a player.
type Feedback struct {
	det    Detector
	player Player
	pcm    [eventCount][]int16
}

// New renders every tone once for p.
func New(p Player) *Feedback {
	f := &Feedback{player: p}
	for e := Event(1); e < eventCount; e++ {
		f.pcm[e] = Render(ToneFor(e))
	}
	return f
}

// Update runs the detector on in and plays the event it returns while
// soundOn; with the sound off, events are still judged (so that switching
// it on does not play old ones) but nothing plays. It returns the event
// played, or None.
func (f *Feedback) Update(in Inputs, soundOn bool) (Event, error) {
	e := f.det.Update(in)
	if e == None || !soundOn {
		return None, nil
	}
	return e, f.player.Play(f.pcm[e])
}
