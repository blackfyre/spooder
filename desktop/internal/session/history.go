package session

import (
	"math"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// HistoryLength is how far back the charts' history reaches; at the robot's
// 10 Hz that is HistorySamples per kind.
const (
	HistoryLength  = 10 * time.Minute
	HistorySamples = 6000
)

// TelemetrySample is a telemetry frame, with the host link's counts when it
// arrived.
type TelemetrySample struct {
	At   time.Time
	T    protocol.Telemetry
	Host link.Stats
}

// DiagnosticsSample is a diagnostics frame.
type DiagnosticsSample struct {
	At time.Time
	D  protocol.Diagnostics
}

// Joint kinds, in servo output order within a leg.
const (
	Coxa = iota
	Femur
	Tibia
	JointKinds
)

// PoseSample is a pose frame: every joint's servo angle, and the fastest
// servo angle change of each joint kind over all legs since the previous
// pose frame (NaN for the first, or when a joint was not set in both).
// Pose frames come every 100 ms, so the speed is that interval's average.
type PoseSample struct {
	At         time.Time
	OutputsOn  bool
	ServoDeg   [protocol.CalibrationJoints]float64 // NaN where never set
	FastestDeg [JointKinds]float64                 // degrees per second
}

// History is what the charts draw: the samples of the current connection,
// oldest first, at most HistoryLength back.
type History struct {
	Start       time.Time // the connection's start
	Telemetry   []TelemetrySample
	Diagnostics []DiagnosticsSample
	Poses       []PoseSample
}

// trimmed drops samples older than HistoryLength before now, and beyond
// HistorySamples.
func trimmed[T any](list []T, at func(T) time.Time, now time.Time) []T {
	cut := 0
	for cut < len(list) && (len(list)-cut > HistorySamples || now.Sub(at(list[cut])) > HistoryLength) {
		cut++
	}
	if cut == 0 {
		return list
	}
	// Copy down once the dropped head is large, so the backing array does
	// not grow without bound.
	if cut > HistorySamples/4 {
		return append(list[:0:0], list[cut:]...)
	}
	return list[cut:]
}

// prune drops every kind's samples older than HistoryLength before now, so
// a kind that stopped arriving does not keep old samples.
func (h *History) prune(now time.Time) {
	h.Telemetry = trimmed(h.Telemetry, func(v TelemetrySample) time.Time { return v.At }, now)
	h.Diagnostics = trimmed(h.Diagnostics, func(v DiagnosticsSample) time.Time { return v.At }, now)
	h.Poses = trimmed(h.Poses, func(v PoseSample) time.Time { return v.At }, now)
}

func (h *History) addTelemetry(s TelemetrySample) {
	h.Telemetry = append(h.Telemetry, s)
	h.prune(s.At)
}

func (h *History) addDiagnostics(s DiagnosticsSample) {
	h.Diagnostics = append(h.Diagnostics, s)
	h.prune(s.At)
}

func (h *History) addPose(at time.Time, p protocol.Pose) {
	s := PoseSample{At: at, OutputsOn: p.Flags&protocol.PoseOutputsOn != 0}
	for i, j := range p.Joints {
		s.ServoDeg[i] = math.NaN()
		if j.PulseUs != 0 {
			s.ServoDeg[i] = float64(j.ServoCentideg) / 100
		}
	}
	for k := range s.FastestDeg {
		s.FastestDeg[k] = math.NaN()
	}
	if n := len(h.Poses); n > 0 {
		prev := h.Poses[n-1]
		if dt := at.Sub(prev.At).Seconds(); dt > 0 {
			for i := range s.ServoDeg {
				d := math.Abs(s.ServoDeg[i]-prev.ServoDeg[i]) / dt
				if math.IsNaN(d) {
					continue
				}
				if k := i % JointKinds; math.IsNaN(s.FastestDeg[k]) || d > s.FastestDeg[k] {
					s.FastestDeg[k] = d
				}
			}
		}
	}
	h.Poses = append(h.Poses, s)
	h.prune(at)
}

// since returns the samples at or after t (the slices share storage with h;
// History values handed out are copies).
func (h History) since(t time.Time) History {
	out := History{Start: h.Start}
	out.Telemetry = append([]TelemetrySample(nil), h.Telemetry[firstAt(len(h.Telemetry), func(i int) time.Time { return h.Telemetry[i].At }, t):]...)
	out.Diagnostics = append([]DiagnosticsSample(nil), h.Diagnostics[firstAt(len(h.Diagnostics), func(i int) time.Time { return h.Diagnostics[i].At }, t):]...)
	out.Poses = append([]PoseSample(nil), h.Poses[firstAt(len(h.Poses), func(i int) time.Time { return h.Poses[i].At }, t):]...)
	return out
}

// firstAt is the index of the first of n time-ordered samples at or after t.
func firstAt(n int, at func(int) time.Time, t time.Time) int {
	lo, hi := 0, n
	for lo < hi {
		mid := (lo + hi) / 2
		if at(mid).Before(t) {
			lo = mid + 1
		} else {
			hi = mid
		}
	}
	return lo
}

// CountRate is the per-second rate of a wrapping 16-bit counter between two
// samples dt apart (NaN for no interval).
func CountRate(before, after uint16, dt time.Duration) float64 {
	if dt <= 0 {
		return math.NaN()
	}
	return float64(after-before) / dt.Seconds()
}

// HostRate is the per-second rate of a host link count between two samples.
func HostRate(before, after uint32, dt time.Duration) float64 {
	if dt <= 0 {
		return math.NaN()
	}
	return float64(after-before) / dt.Seconds()
}

// HistorySince returns a copy of the history from t on, and the history's
// version, which changes whenever a sample is added or the history restarts;
// a caller holding a copy of that version need not ask again.
func (s *Session) HistorySince(t time.Time) (History, uint64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.history.prune(s.now())
	return s.history.since(t), s.historyVersion
}

// HistoryVersion is the history's current version (see HistorySince).
func (s *Session) HistoryVersion() uint64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.historyVersion
}
