package feedback

import (
	"math"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
)

var t0 = time.Unix(1000, 0)

// rig steps a detector from a fresh, quiet baseline.
type rig struct {
	t  *testing.T
	d  Detector
	in Inputs
}

func newRig(t *testing.T) *rig {
	r := &rig{t: t, in: Inputs{Now: t0, TelemetryFresh: true}}
	if e := r.d.Update(r.in); e != None {
		t.Fatalf("baseline played %v", e)
	}
	return r
}

// at advances to ms after t0 and updates.
func (r *rig) at(ms int) Event {
	r.in.Now = t0.Add(time.Duration(ms) * time.Millisecond)
	return r.d.Update(r.in)
}

func (r *rig) want(ms int, want Event) {
	r.t.Helper()
	if got := r.at(ms); got != want {
		r.t.Fatalf("at %d ms: %v, want %v", ms, got, want)
	}
}

// What is already so at the start never sounds: a latched fault, the e-stop.
func TestBaselineIsSilent(t *testing.T) {
	var d Detector
	in := Inputs{Now: t0, TelemetryFresh: true, Faults: 0x04, EStop: true, GestureSeq: 3}
	for ms := 0; ms < 2000; ms += 20 {
		in.Now = t0.Add(time.Duration(ms) * time.Millisecond)
		if e := d.Update(in); e != None {
			t.Fatalf("at %d ms: %v", ms, e)
		}
	}
}

// The pak's real start: no telemetry at the first redraw (the link sends
// nothing for its first 150 ms, telemetry comes every 100 ms), then fresh
// telemetry. Its arrival is the baseline, not the link regained; a later
// lapse and its return sound once each.
func TestFirstTelemetryIsSilent(t *testing.T) {
	r := &rig{t: t, in: Inputs{Now: t0}}
	r.want(0, None)
	r.want(100, None)
	r.in.TelemetryFresh, r.in.Faults = true, 0x04 // a fault latched before the run
	for ms := 200; ms < 3000; ms += 50 {
		r.want(ms, None)
	}
	r.in.TelemetryFresh = false
	r.want(3000, LinkLost)
	r.want(3500, None)
	r.in.TelemetryFresh = true
	r.want(4500, LinkRegained)
	r.want(5000, None)
}

// A resume after a suspend starts anew as a run does: the stale telemetry
// at the wake and its first fresh frame sound nothing; a lapse after it does.
func TestResumeIsSilent(t *testing.T) {
	r := newRig(t)
	r.in.Leaving = true
	r.want(100, Leaving)
	r.want(400, None)
	r.in.TelemetryFresh = false // the suspend, and the wake while it ends
	r.want(30000, None)
	r.in.Leaving = false // resumed
	r.want(30050, None)
	r.want(30150, None)
	r.in.TelemetryFresh = true
	for ms := 30250; ms < 32000; ms += 50 {
		r.want(ms, None)
	}
	r.in.TelemetryFresh = false
	r.want(32000, LinkLost)
	r.in.TelemetryFresh = true
	r.want(33100, LinkRegained)
}

// Each event, from the change that makes it.
func TestEvents(t *testing.T) {
	r := newRig(t)
	r.in.EStop = true
	r.want(20, EstopOn)
	r.in.EStop = false
	r.want(2000, EstopOff)

	r.in.Faults = faultEstop // the e-stop's fault has its own sound
	r.want(4000, None)
	r.in.Faults |= 0x01 // overcurrent
	r.want(6000, FaultLatched)
	r.want(8000, None) // still latched: nothing new

	r.in.TelemetryFresh = false
	r.want(10000, LinkLost)
	r.in.TelemetryFresh, r.in.Faults = true, 0x01|0x04 // a fault latched during the gap is the new baseline
	r.want(12000, LinkRegained)
	r.want(12100, None)

	r.in.Leaving = true
	r.want(14000, Leaving)
	r.want(16000, None)
}

// A request the robot plays sounds accepted; one it does not play within
// 600 ms sounds refused; a link loss meanwhile only sounds the loss.
func TestGestures(t *testing.T) {
	r := newRig(t)
	r.in.GestureSeq, r.in.Gesture = 1, 1
	r.want(20, None)
	r.in.Playing = 1
	r.want(120, GestureAccepted)
	r.want(2000, None)

	r.in.Playing = 0
	r.in.GestureSeq, r.in.Gesture = 2, 2
	r.want(4000, None)
	r.want(4000+int(RefusedAfter/time.Millisecond)-1, None)
	r.want(4000+int(RefusedAfter/time.Millisecond), GestureRefused)

	// The same gesture still playing from before is not taken as accepted.
	r.in.Playing = 1
	r.want(6000, None)
	r.in.GestureSeq, r.in.Gesture = 3, 1
	r.want(6020, None)
	r.want(6700, GestureRefused)

	r.in.Playing = 0
	r.in.GestureSeq, r.in.Gesture = 4, 2
	r.want(9000, None)
	r.in.TelemetryFresh = false
	r.want(9100, LinkLost)
	r.want(10000, None) // no refusal on top of the loss
}

// One event sounds at most once a second; an event that cannot play within
// a second of happening is dropped.
func TestRepeatGuard(t *testing.T) {
	r := newRig(t)
	r.in.EStop = true
	r.want(0, EstopOn)
	r.in.EStop = false
	r.want(500, EstopOff)
	r.in.EStop = true
	r.want(900, None) // e-stop on again 900 ms after it last sounded: dropped
	r.in.EStop = false
	r.want(1200, None) // e-stop off 700 ms after it sounded: dropped
	r.in.EStop = true
	r.want(1600, EstopOn) // 1.6 s after: sounds
}

// A more important event replaces a less important one still playing; a
// less important one waits for the more important one to end, and is
// dropped if that takes a second.
func TestPriorities(t *testing.T) {
	r := newRig(t)
	r.in.GestureSeq, r.in.Gesture, r.in.Playing = 1, 1, 0
	r.at(0)
	r.in.Playing = 1
	r.want(20, GestureAccepted)
	r.in.Faults = 0x01
	r.want(30, FaultLatched) // replaces the blip

	r.in.EStop = true // e-stop on: lower than the fault's tone
	end := 30 + int(ToneFor(FaultLatched).Duration()/time.Millisecond)
	r.want(40, None)
	r.want(end-1, None)
	r.want(end, EstopOn) // the fault's tone has ended

	// A refusal waiting behind a long run of tones is dropped after a second.
	r2 := newRig(t)
	r2.in.Faults = 0x01
	r2.want(0, FaultLatched)
	if ToneFor(FaultLatched).Duration() >= RepeatAfter {
		t.Fatal("the fault's tone is a second or longer")
	}
}

// Every event has a tone; the safety events outrank the confirmations, and
// no two events sound alike.
func TestToneForEveryEvent(t *testing.T) {
	seen := map[string]Event{}
	for e := Event(1); e < eventCount; e++ {
		tn := ToneFor(e)
		if len(tn.Segments) == 0 || tn.Duration() <= 0 || tn.Duration() >= RepeatAfter {
			t.Errorf("%v: tone %+v", e, tn)
		}
		key := ""
		for _, s := range tn.Segments {
			key += time.Duration(s.Ms).String() + "@" + time.Duration(s.Hz).String() + "/" + string(rune('0'+s.Wave)) + " "
		}
		if o, ok := seen[key]; ok {
			t.Errorf("%v sounds like %v", e, o)
		}
		seen[key] = e
	}
	for _, safety := range []Event{FaultLatched, EstopOn, LinkLost} {
		for _, confirm := range []Event{EstopOff, LinkRegained, GestureAccepted} {
			if ToneFor(safety).Priority <= ToneFor(confirm).Priority {
				t.Errorf("%v does not outrank %v", safety, confirm)
			}
		}
	}
}

// A rendered tone lasts its segments, peaks at the level and starts and
// ends at zero (no click).
func TestRender(t *testing.T) {
	for e := Event(1); e < eventCount; e++ {
		tn := ToneFor(e)
		pcm := Render(tn)
		if want := int(tn.Duration() / time.Millisecond * SampleRate / 1000); len(pcm) != want {
			t.Errorf("%v: %d samples, want %d", e, len(pcm), want)
		}
		peak := 0
		for _, v := range pcm {
			peak = max(peak, int(math.Abs(float64(v))))
		}
		top := Level * math.MaxInt16
		if float64(peak) > top+1 || float64(peak) < 0.9*top {
			t.Errorf("%v: peak %d, want about %.0f", e, peak, top)
		}
		if pcm[0] != 0 || pcm[len(pcm)-1] != 0 {
			t.Errorf("%v: starts at %d, ends at %d", e, pcm[0], pcm[len(pcm)-1])
		}
	}
}

type recorder struct{ played [][]int16 }

func (r *recorder) Play(pcm []int16) error { r.played = append(r.played, pcm); return nil }

// The feedback plays an event's tone through the player, and nothing while
// the sound is off, without playing the old event when it is switched on.
func TestFeedbackPlaysWhileSoundOn(t *testing.T) {
	rec := &recorder{}
	f := New(rec)
	sc := app.Screen{Robot: app.Robot{Have: true}}
	step := func(ms int, on bool) Event {
		e, err := f.Update(FromScreen(sc, t0.Add(time.Duration(ms)*time.Millisecond)), on)
		if err != nil {
			t.Fatal(err)
		}
		return e
	}
	step(0, true)
	sc.Requested.EStop = true
	if e := step(20, true); e != EstopOn || len(rec.played) != 1 || len(rec.played[0]) != len(Render(ToneFor(EstopOn))) {
		t.Fatalf("e-stop on: %v, %d played", e, len(rec.played))
	}
	sc.Requested.EStop = false
	if e := step(2000, false); e != None || len(rec.played) != 1 {
		t.Fatalf("with the sound off: %v, %d played", e, len(rec.played))
	}
	if e := step(2020, true); e != None || len(rec.played) != 1 {
		t.Fatalf("switched on: %v, %d played", e, len(rec.played))
	}
	sc.State = app.Leaving
	if e := step(4000, true); e != Leaving || len(rec.played) != 2 {
		t.Fatalf("leaving: %v, %d played", e, len(rec.played))
	}
}

// The inputs come from the screen model: fresh telemetry only while not
// stale, the fault bits, the pak's e-stop and gesture request, the gesture
// playing and the leave.
func TestFromScreen(t *testing.T) {
	sc := app.Screen{
		Robot:     app.Robot{Have: true, FaultBits: 0x05, GestureID: 2},
		Requested: app.Requested{EStop: true, Gesture: 2, GestureSeq: 7},
		State:     app.Acting,
	}
	in := FromScreen(sc, t0)
	want := Inputs{Now: t0, TelemetryFresh: true, Faults: 0x05, EStop: true, GestureSeq: 7, Gesture: 2, Playing: 2, Leaving: true}
	if in != want {
		t.Errorf("FromScreen = %+v, want %+v", in, want)
	}
	sc.Robot.Stale = true
	if FromScreen(sc, t0).TelemetryFresh {
		t.Error("stale telemetry taken as fresh")
	}
}
