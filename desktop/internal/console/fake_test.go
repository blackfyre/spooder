package console

import (
	"bytes"
	"context"
	"errors"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/link"
)

// fakePort is an in-memory serial port: the test sends the robot's bytes,
// and every write reaching it is recorded.
type fakePort struct {
	mu      sync.Mutex
	in      chan []byte // wakes a waiting read
	queue   [][]byte
	timeout time.Duration
	closed  chan struct{}
	lost    chan struct{}
	once    sync.Once
	out     []byte
	writes  int
}

var errUnplugged = errors.New("unplugged")

func newFakePort() *fakePort {
	return &fakePort{in: make(chan []byte, 256), closed: make(chan struct{}), lost: make(chan struct{}), timeout: time.Second}
}

func (p *fakePort) send(b []byte) { p.sendChunks(b, len(b)) }

// sendChunks sends b in pieces of n bytes, one read each. The pieces are
// queued at once, so no read timeout falls between them.
func (p *fakePort) sendChunks(b []byte, n int) {
	var chunks [][]byte
	for len(b) > 0 {
		k := min(n, len(b))
		chunks = append(chunks, append([]byte(nil), b[:k]...))
		b = b[k:]
	}
	p.mu.Lock()
	p.queue = append(p.queue, chunks...)
	p.mu.Unlock()
	p.in <- nil // wake a waiting read
}

func (p *fakePort) unplug() { close(p.lost) }

func (p *fakePort) Read(b []byte) (int, error) {
	p.mu.Lock()
	timeout := p.timeout
	p.mu.Unlock()
	deadline := time.After(timeout)
	for {
		select {
		case <-p.closed:
			return 0, errors.New("port closed")
		case <-p.lost:
			return 0, errUnplugged
		default:
		}
		p.mu.Lock()
		if len(p.queue) > 0 {
			n := copy(b, p.queue[0])
			if p.queue[0] = p.queue[0][n:]; len(p.queue[0]) == 0 {
				p.queue = p.queue[1:]
			}
			p.mu.Unlock()
			return n, nil
		}
		p.mu.Unlock()
		select {
		case <-p.in:
		case <-p.closed:
		case <-p.lost:
		case <-deadline:
			return 0, nil
		}
	}
}

func (p *fakePort) Write(b []byte) (int, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.writes++
	p.out = append(p.out, b...)
	return len(b), nil
}

func (p *fakePort) Close() error {
	p.once.Do(func() { close(p.closed) })
	return nil
}

func (p *fakePort) SetReadTimeout(t time.Duration) error {
	p.mu.Lock()
	p.timeout = t
	p.mu.Unlock()
	return nil
}

func (p *fakePort) written() (int, []byte) {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.writes, append([]byte(nil), p.out...)
}

// fakeSystem lists one device while it is plugged in and hands out a new
// fakePort on every open.
type fakeSystem struct {
	mu      sync.Mutex
	device  link.PortInfo
	present bool
	ports   []*fakePort
	opened  chan *fakePort
}

func newFakeSystem(device link.PortInfo, present bool) *fakeSystem {
	return &fakeSystem{device: device, present: present, opened: make(chan *fakePort, 16)}
}

func (s *fakeSystem) List() ([]link.PortInfo, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	ports := []link.PortInfo{{Name: "/dev/ttyS0"}}
	if s.present {
		ports = append(ports, s.device)
	}
	return ports, nil
}

func (s *fakeSystem) Open(name string) (link.Port, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !s.present || name != s.device.Name {
		return nil, errors.New("no such port")
	}
	p := newFakePort()
	s.ports = append(s.ports, p)
	s.opened <- p
	return p, nil
}

func (s *fakeSystem) setPresent(present bool) {
	s.mu.Lock()
	s.present = present
	s.mu.Unlock()
}

func (s *fakeSystem) allPorts() []*fakePort {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]*fakePort(nil), s.ports...)
}

// output is a writer the test can read while the monitor writes.
type output struct {
	mu sync.Mutex
	b  bytes.Buffer
}

func (o *output) Write(p []byte) (int, error) {
	o.mu.Lock()
	defer o.mu.Unlock()
	return o.b.Write(p)
}

func (o *output) String() string {
	o.mu.Lock()
	defer o.mu.Unlock()
	return o.b.String()
}

// waitFor polls until the output holds s.
func (o *output) waitFor(t *testing.T, s string) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for !strings.Contains(o.String(), s) {
		if time.Now().After(deadline) {
			t.Fatalf("no %q in output:\n%s", s, o.String())
		}
		time.Sleep(2 * time.Millisecond)
	}
}

// clock is a deterministic clock: each reading is 100 ms after the last.
func clock() func() time.Time {
	var mu sync.Mutex
	t := time.Date(2026, 10, 18, 12, 0, 0, 0, time.Local)
	return func() time.Time {
		mu.Lock()
		defer mu.Unlock()
		t = t.Add(100 * time.Millisecond)
		return t
	}
}

var servo = link.PortInfo{Name: "/dev/ttyACM0", IsUSB: true, VID: "2E8A", PID: "000A", SerialNumber: "E66138935F2A6B24"}

var fast = link.Options{RetryInterval: 10 * time.Millisecond, ReadTimeout: 5 * time.Millisecond}

// monitor runs the monitor against sys until the returned stop is called;
// stop waits for Run to return.
type monitor struct {
	out  *output
	stop func()
}

func startMonitor(t *testing.T, c Config, sys link.System, keys *keyInput) *monitor {
	t.Helper()
	env := Env{System: sys, Exists: func(string) bool { return false }}
	if keys != nil {
		env.Keys = keys
	}
	return startMonitorEnv(t, c, env)
}

// startMonitorEnv fills in the output, a deterministic clock and fast
// timing.
func startMonitorEnv(t *testing.T, c Config, env Env) *monitor {
	t.Helper()
	if c.TelemetryEvery == 0 {
		c.TelemetryEvery = 1
	}
	ctx, cancel := context.WithCancel(context.Background())
	out := &output{}
	done := make(chan error, 1)
	env.Out, env.Now, env.Options = out, clock(), fast
	go func() { done <- Run(ctx, c, env) }()
	var once sync.Once
	m := &monitor{out: out, stop: func() {
		once.Do(func() {
			cancel()
			if err := <-done; err != nil {
				t.Errorf("Run: %v", err)
			}
		})
	}}
	t.Cleanup(m.stop)
	return m
}

// keyInput is standard input fed by the test.
type keyInput struct{ ch chan []byte }

func newKeyInput() *keyInput { return &keyInput{ch: make(chan []byte, 16)} }

func (k *keyInput) Read(b []byte) (int, error) {
	chunk, ok := <-k.ch
	if !ok {
		return 0, errors.New("EOF")
	}
	return copy(b, chunk), nil
}

func waitPort(t *testing.T, sys *fakeSystem) *fakePort {
	t.Helper()
	select {
	case p := <-sys.opened:
		return p
	case <-time.After(2 * time.Second):
		t.Fatal("port not opened")
	}
	return nil
}
