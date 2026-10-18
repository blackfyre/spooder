// Package e2e drives the desktop calibrator's real link and session against
// the virtual robot (tools/virtual-robot), which runs the robot firmware's
// own control tick behind a pseudo-terminal. Build the robot first
// (`mise run virtual-robot`, or `mise run e2e` for both); without it the
// tests skip.
package e2e

import (
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/session"
	"github.com/blackfyre/spooder/desktop/internal/virtualrobot"
)

// Robot is a running virtual robot, failing the test on errors.
type Robot struct {
	t *testing.T
	*virtualrobot.Robot
}

// StartRobot runs the virtual robot with its USB terminal linked in a
// temporary directory, or skips the test if it is not built.
func StartRobot(t *testing.T, args ...string) *Robot {
	t.Helper()
	bin, ok := virtualrobot.Binary()
	if !ok {
		t.Skip("virtual robot not built (mise run virtual-robot):", bin)
	}
	vr, err := virtualrobot.Start(bin, filepath.Join(t.TempDir(), "usb"), args...)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(vr.Close)
	return &Robot{t: t, Robot: vr}
}

// Do sends a command and returns its answer.
func (r *Robot) Do(command string) string {
	r.t.Helper()
	answer, err := r.Robot.Do(command)
	if err != nil {
		r.t.Fatal(err)
	}
	return answer
}

// Calibrator is the desktop app without its window: the real session and
// link, connected to the robot's terminal through --port's path.
type Calibrator struct {
	S *session.Session
	L *link.Link
}

// Connect connects a calibrator to the robot and waits for telemetry.
func Connect(t *testing.T, r *Robot) *Calibrator {
	t.Helper()
	s := session.New(nil, nil)
	l := link.New(r.System(), s, s, link.Options{})
	l.Connect(link.PortInfo{Name: r.Link})
	t.Cleanup(func() { s.Shutdown(); l.Close() })
	c := &Calibrator{S: s, L: l}
	c.WaitFor(t, 5*time.Second, "connected with telemetry", func(s session.Snapshot) bool {
		return s.Link.Status == link.Connected && s.Fresh
	})
	return c
}

// WaitFor waits until the session's snapshot satisfies cond.
func (c *Calibrator) WaitFor(t *testing.T, timeout time.Duration, what string, cond func(session.Snapshot) bool) session.Snapshot {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for {
		s := c.S.Snapshot()
		if cond(s) {
			return s
		}
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s; last: link %v, body %v, faults %v, source %v, outputs %v",
				what, s.Link.Status, s.Body, s.Faults, s.Source, s.Telemetry.OutputsOn)
		}
		time.Sleep(10 * time.Millisecond)
	}
}

// Calibrate takes control and brings the robot into Calibrate with a report.
func (c *Calibrator) Calibrate(t *testing.T) session.Snapshot {
	t.Helper()
	if err := c.S.TakeControl(); err != nil {
		t.Fatal(err)
	}
	if err := c.S.RequestMode(session.RequestCalibrate); err != nil {
		t.Fatal(err)
	}
	return c.WaitFor(t, 15*time.Second, "Calibrate and editable", func(s session.Snapshot) bool {
		return s.Body == session.BodyCalibrate && s.Editable && s.Source == session.SourceUSB
	})
}

func hasLog(s session.Snapshot, prefix string) bool {
	for _, l := range s.Log {
		if strings.HasPrefix(l.Text, prefix) {
			return true
		}
	}
	return false
}
