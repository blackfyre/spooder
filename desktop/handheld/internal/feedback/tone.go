package feedback

import (
	"math"
	"time"
)

// SampleRate is the tones' sample rate.
const SampleRate = 48000

// Level is every tone's peak, about -12 dBFS (design D4).
const Level = 0.25

// rampMs is each segment's attack and release, so that no tone clicks.
const rampMs = 5

// Wave is a segment's waveform.
type Wave int

const (
	Sine Wave = iota
	Square
)

// Segment is one part of a tone: a tone at Hz for Ms, or silence at 0 Hz.
type Segment struct {
	Hz   float64
	Ms   int
	Wave Wave
}

// Tone is an event's sound and its priority: a higher one replaces a lower
// one still playing.
type Tone struct {
	Segments []Segment
	Priority int
}

// Duration is the tone's length.
func (t Tone) Duration() time.Duration {
	var ms int
	for _, s := range t.Segments {
		ms += s.Ms
	}
	return time.Duration(ms) * time.Millisecond
}

// ToneFor is e's tone (design D4): safety events lower, longer and more
// important; confirmations short and high.
func ToneFor(e Event) Tone {
	switch e {
	case FaultLatched: // three short low beeps
		return Tone{[]Segment{{330, 90, Sine}, {0, 60, Sine}, {330, 90, Sine}, {0, 60, Sine}, {330, 90, Sine}}, 5}
	case EstopOn: // falling two-tone
		return Tone{[]Segment{{880, 120, Sine}, {440, 200, Sine}}, 4}
	case LinkLost: // one low tone
		return Tone{[]Segment{{300, 250, Sine}}, 3}
	case Leaving: // falling three-tone
		return Tone{[]Segment{{660, 100, Sine}, {440, 100, Sine}, {330, 150, Sine}}, 3}
	case GestureRefused: // a short low buzz
		return Tone{[]Segment{{200, 150, Square}}, 2}
	case EstopOff: // rising two-tone
		return Tone{[]Segment{{440, 80, Sine}, {880, 80, Sine}}, 1}
	case LinkRegained: // one high tone
		return Tone{[]Segment{{1200, 80, Sine}}, 1}
	case GestureAccepted: // a short high blip
		return Tone{[]Segment{{1500, 50, Sine}}, 1}
	}
	return Tone{}
}

// Render is t as 16-bit mono PCM at SampleRate and Level, each sounding
// segment ramped in and out over rampMs.
func Render(t Tone) []int16 {
	var out []int16
	for _, s := range t.Segments {
		n := s.Ms * SampleRate / 1000
		ramp := min(rampMs*SampleRate/1000, n/2)
		for i := 0; i < n; i++ {
			if s.Hz == 0 {
				out = append(out, 0)
				continue
			}
			v := math.Sin(2 * math.Pi * s.Hz * float64(i) / SampleRate)
			if s.Wave == Square {
				v = math.Copysign(1, v)
			}
			g := 1.0
			if i < ramp {
				g = float64(i) / float64(ramp)
			} else if n-1-i < ramp {
				g = float64(n-1-i) / float64(ramp)
			}
			out = append(out, int16(math.Round(v*g*Level*math.MaxInt16)))
		}
	}
	return out
}
