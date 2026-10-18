package e2e

import (
	"fmt"
	"math"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// Observing sends no controller: the robot names no source and never
// starts the body, and the app's mode controls stay disabled.
func TestObserveSendsNoController(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	time.Sleep(1500 * time.Millisecond) // past the supply settling time
	s := c.S.Snapshot()
	if s.Source != session.SourceNone || s.Telemetry.Flags&protocol.TelemetryUSBInControl != 0 {
		t.Fatalf("source %v, flags %#x", s.Source, s.Telemetry.Flags)
	}
	if s.ModesEnabled || s.Telemetry.OutputsOn {
		t.Fatalf("modes enabled %v, outputs %v", s.ModesEnabled, s.Telemetry.OutputsOn)
	}
	if st := r.Do("status"); !strings.Contains(st, "running=0") {
		t.Fatal(st)
	}
	if hasLog(s, "operator source: USB") {
		t.Fatal("the robot followed the observer")
	}
	if !hasLog(s, "supply ") {
		t.Fatal("no debug text in the log")
	}
}

// In control and in Calibrate, a joint change is confirmed by the robot's
// report, and the applied value follows at no more than 60°/s.
func TestCalibrationChangeIsConfirmedAndRateLimited(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	snap := c.Calibrate(t)
	const joint = 4
	before := snap.Joints[joint].Reported
	if err := c.S.Nudge(joint, calib.FieldOffset, 1, true); err != nil { // +5°
		t.Fatal(err)
	}
	want := before.OffsetCentideg + calib.CoarseOffsetCentideg
	snap = c.WaitFor(t, 3*time.Second, "the change confirmed", func(s session.Snapshot) bool {
		j := s.Joints[joint]
		return !j.Pending && j.Reported.OffsetCentideg == want && j.Result == calib.ResultAccepted
	})

	// The applied value, as the robot reports it after each tick.
	prefix := fmt.Sprintf("joint=%d ", joint)
	deadline := time.Now().Add(3 * time.Second)
	for {
		var times []float64
		var offsets []float64
		for _, l := range r.Lines() {
			if !strings.HasPrefix(l, "applied ") || !strings.Contains(l, prefix) {
				continue
			}
			var tm uint32
			var j int
			var off, p0, p180 float64
			var rev int
			if _, err := fmt.Sscanf(l, "applied t=%d joint=%d offset=%f pulse0=%f pulse180=%f reversed=%d", &tm, &j, &off, &p0, &p180, &rev); err != nil {
				t.Fatalf("%q: %v", l, err)
			}
			times = append(times, float64(tm))
			offsets = append(offsets, off)
		}
		if n := len(offsets); n > 0 && math.Abs(offsets[n-1]-float64(want)/100) < 0.01 {
			if n < 4 {
				t.Fatalf("a 5° change arrived in %d ticks: not rate-limited", n)
			}
			prev, prevT := float64(before.OffsetCentideg)/100, times[0]-20
			for i := range offsets {
				rate := math.Abs(offsets[i]-prev) / ((times[i] - prevT) / 1000)
				if rate > 60+0.5 {
					t.Fatalf("applied moved at %.1f°/s at t=%v", rate, times[i])
				}
				prev, prevT = offsets[i], times[i]
			}
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("applied offset never reached %v: %v", float64(want)/100, offsets)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

// The e-stop latches whether or not the app is in control, switches output
// off, and a fault clear does nothing while it is still signalled.
func TestEmergencyStopLatchesAndBlocksClear(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	c.Calibrate(t)
	c.WaitFor(t, 2*time.Second, "output on", func(s session.Snapshot) bool { return s.Telemetry.OutputsOn })

	c.S.ToggleEmergencyStop()
	c.WaitFor(t, time.Second, "e-stop latched, output off", func(s session.Snapshot) bool {
		return s.Telemetry.Faults&session.FaultEmergencyStop != 0 && !s.Telemetry.OutputsOn
	})
	if err := c.S.ClearFault(); err != nil {
		t.Fatal(err)
	}
	time.Sleep(500 * time.Millisecond)
	if s := c.S.Snapshot(); s.Telemetry.Faults&session.FaultEmergencyStop == 0 {
		t.Fatal("cleared while signalled")
	}
	c.S.ToggleEmergencyStop()
	time.Sleep(100 * time.Millisecond)
	if err := c.S.ClearFault(); err != nil {
		t.Fatal(err)
	}
	c.WaitFor(t, 2*time.Second, "fault cleared", func(s session.Snapshot) bool {
		return s.Telemetry.Faults == 0
	})
	// Clearing alone never restarts motion: output stays off until a new request.
	time.Sleep(300 * time.Millisecond)
	if s := c.S.Snapshot(); s.Telemetry.OutputsOn {
		t.Fatal("output on again without a new request")
	}
	if err := c.S.RequestMode(session.RequestCalibrate); err != nil {
		t.Fatal(err)
	}
	c.WaitFor(t, 10*time.Second, "restarted", func(s session.Snapshot) bool { return s.Telemetry.OutputsOn })
}

// The e-stop from the (simulated) bridge latches as well, while the app
// only observes.
func TestBridgeEmergencyStopWhileObserving(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	r.Do("estop on")
	c.WaitFor(t, time.Second, "e-stop latched", func(s session.Snapshot) bool {
		return s.Telemetry.Faults&session.FaultEmergencyStop != 0
	})
}

// Releasing control puts the robot to sleep.
func TestReleaseControlSleeps(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	c.Calibrate(t)
	c.S.ReleaseControl()
	c.WaitFor(t, 3*time.Second, "Sleep with no source", func(s session.Snapshot) bool {
		return s.Body == session.BodySleep && s.Source == session.SourceNone
	})
}

// A supply above the maximum voltage trips protection.
func TestOvervoltageTrips(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	c.Calibrate(t)
	r.Do("supply 7.5")
	s := c.WaitFor(t, time.Second, "overvoltage, output off", func(s session.Snapshot) bool {
		return s.Telemetry.Faults&session.FaultOvervoltage != 0 && !s.Telemetry.OutputsOn
	})
	if s.Telemetry.VoltageMv != 7500 {
		t.Fatalf("voltage %d mV", s.Telemetry.VoltageMv)
	}
	c.WaitFor(t, time.Second, "the fault in the log", func(s session.Snapshot) bool {
		return hasLog(s, "fault latched: overvoltage (")
	})
}

// The gamepad on the bridge while the app is in control is a conflict.
func TestConflictWithTheBridgeSleeps(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	c.Calibrate(t)
	r.Do("bridge gamepad")
	c.WaitFor(t, 3*time.Second, "conflict and Sleep", func(s session.Snapshot) bool {
		return s.Source == session.SourceConflict && s.Body == session.BodySleep
	})
}

// The simulated current rises above the idle draw while the legs move, and a
// heavy load trips protection.
func TestCurrentGrowsWithMotionAndLoadTrips(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	if err := c.S.TakeControl(); err != nil {
		t.Fatal(err)
	}
	if err := c.S.RequestMode(session.RequestCalibrate); err != nil {
		t.Fatal(err)
	}
	const idleMa = 18 * 120
	var peak uint16
	c.WaitFor(t, 15*time.Second, "Calibrate", func(s session.Snapshot) bool {
		peak = max(peak, s.Telemetry.CurrentMa)
		return s.Body == session.BodyCalibrate && s.Telemetry.OutputsOn
	})
	if peak <= idleMa {
		t.Fatalf("peak %d mA while moving, idle %d mA", peak, idleMa)
	}
	r.Do("load 5")
	c.WaitFor(t, time.Second, "overcurrent", func(s session.Snapshot) bool {
		return s.Telemetry.Faults&session.FaultOvercurrent != 0 && !s.Telemetry.OutputsOn
	})
}

// Unplugging the virtual cable loses the connection; replugging (a new
// terminal behind the same path) reconnects without user action.
func TestUnplugAndReplug(t *testing.T) {
	t.Parallel()
	r := StartRobot(t)
	c := Connect(t, r)
	r.Do("unplug")
	c.WaitFor(t, 2*time.Second, "connection lost", func(s session.Snapshot) bool {
		return s.Link.Status != link.Connected
	})
	r.Do("plug")
	c.WaitFor(t, 3*time.Second, "reconnected with telemetry", func(s session.Snapshot) bool {
		return s.Link.Status == link.Connected && s.Fresh && s.TelemetryAge < 200*time.Millisecond
	})
}
