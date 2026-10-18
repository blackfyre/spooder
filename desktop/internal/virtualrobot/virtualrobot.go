// Package virtualrobot runs the virtual robot (tools/virtual-robot) for
// tests: the robot firmware's own control tick behind a pseudo-terminal,
// controlled through its standard input and observed through its standard
// output.
package virtualrobot

import (
	"bufio"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
)

// Binary returns the virtual robot's path: SPOODER_VIRTUAL_ROBOT, or the
// mise build in this checkout (`mise run virtual-robot`). ok is false when
// it does not exist.
func Binary() (path string, ok bool) {
	path = os.Getenv("SPOODER_VIRTUAL_ROBOT")
	if path == "" {
		_, here, _, _ := runtime.Caller(0)
		path = filepath.Join(filepath.Dir(here), "..", "..", "..", "build", "virtual-robot", "virtual-robot")
	}
	_, err := os.Stat(path)
	return path, err == nil
}

// System lists only the robot's terminal (never the computer's own ports)
// and opens it as the app does.
func (r *Robot) System() link.System {
	return link.WithPort(noPorts{}, r.Link)
}

type noPorts struct{}

func (noPorts) List() ([]link.PortInfo, error)      { return nil, nil }
func (noPorts) Open(name string) (link.Port, error) { return link.Serial{}.Open(name) }

// Robot is a running virtual robot.
type Robot struct {
	Link string // the USB terminal's stable path

	cmd   *exec.Cmd
	stdin io.WriteCloser

	mu    sync.Mutex
	lines []string
	read  int // lines consumed by Do
}

// Start runs the binary in real time with its USB terminal linked at link
// and waits until it is ready. The link protocol's gap and staleness rules
// are wall-clock rules for a front-end, so tests run it in real time.
func Start(binary, link string, args ...string) (*Robot, error) {
	r := &Robot{Link: link}
	r.cmd = exec.Command(binary, append([]string{"--link", link}, args...)...)
	r.cmd.Stderr = os.Stderr
	var err error
	if r.stdin, err = r.cmd.StdinPipe(); err != nil {
		return nil, err
	}
	out, err := r.cmd.StdoutPipe()
	if err != nil {
		return nil, err
	}
	if err := r.cmd.Start(); err != nil {
		return nil, err
	}
	go func() {
		sc := bufio.NewScanner(out)
		for sc.Scan() {
			r.mu.Lock()
			r.lines = append(r.lines, sc.Text())
			r.mu.Unlock()
		}
	}()
	if _, err := r.WaitLine("ready", 5*time.Second); err != nil {
		r.Close()
		return nil, err
	}
	return r, nil
}

// Close ends the robot (end of its input; killed if it does not exit).
func (r *Robot) Close() {
	r.stdin.Close()
	done := make(chan struct{})
	go func() { r.cmd.Wait(); close(done) }()
	select {
	case <-done:
	case <-time.After(3 * time.Second):
		r.cmd.Process.Kill()
		<-done
	}
}

// Lines returns a copy of everything the robot has printed.
func (r *Robot) Lines() []string {
	r.mu.Lock()
	defer r.mu.Unlock()
	return append([]string(nil), r.lines...)
}

// WaitLine waits for a line starting with prefix and returns it.
func (r *Robot) WaitLine(prefix string, timeout time.Duration) (string, error) {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		for _, l := range r.Lines() {
			if strings.HasPrefix(l, prefix) {
				return l, nil
			}
		}
		time.Sleep(5 * time.Millisecond)
	}
	return "", fmt.Errorf("virtual robot never printed %q", prefix)
}

// Do sends a command and returns its answer (an ok or status line); an
// error answer is returned as an error.
func (r *Robot) Do(command string) (string, error) {
	if _, err := fmt.Fprintln(r.stdin, command); err != nil {
		return "", err
	}
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		r.mu.Lock()
		for ; r.read < len(r.lines); r.read++ {
			l := r.lines[r.read]
			if strings.HasPrefix(l, "ok") || strings.HasPrefix(l, "error") || strings.HasPrefix(l, "status") {
				r.read++
				r.mu.Unlock()
				if strings.HasPrefix(l, "error") {
					return "", errors.New(command + ": " + l)
				}
				return l, nil
			}
		}
		r.mu.Unlock()
		time.Sleep(5 * time.Millisecond)
	}
	return "", fmt.Errorf("no answer to %q", command)
}
