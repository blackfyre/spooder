package link

import (
	"errors"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// fakePort is an in-memory serial port. The test writes the robot's bytes
// with send; the link's writes are collected.
type fakePort struct {
	mu      sync.Mutex
	in      chan []byte
	timeout time.Duration
	closed  chan struct{}
	lost    chan struct{}
	once    sync.Once
	out     []byte
	rest    []byte
	// maxWrite limits the bytes each write takes (0: all); blockWrites
	// makes writes wait until the port is closed.
	maxWrite    int
	blockWrites bool
	writes  int
}

var errUnplugged = errors.New("unplugged")
var errClosed = errors.New("port closed")

func newFakePort() *fakePort {
	return &fakePort{in: make(chan []byte, 64), closed: make(chan struct{}), lost: make(chan struct{}), timeout: time.Second}
}

func (p *fakePort) send(b []byte) { p.in <- append([]byte(nil), b...) }

func (p *fakePort) unplug() { close(p.lost) }

func (p *fakePort) Read(b []byte) (int, error) {
	p.mu.Lock()
	timeout := p.timeout
	p.mu.Unlock()
	select {
	case <-p.closed:
		return 0, errClosed
	case <-p.lost:
		return 0, errUnplugged
	default:
	}
	p.mu.Lock()
	if len(p.rest) > 0 {
		n := copy(b, p.rest)
		p.rest = p.rest[n:]
		p.mu.Unlock()
		return n, nil
	}
	p.mu.Unlock()
	select {
	case chunk := <-p.in:
		n := copy(b, chunk)
		p.mu.Lock()
		p.rest = chunk[n:] // the rest comes with the next read
		p.mu.Unlock()
		return n, nil
	case <-p.closed:
		return 0, errClosed
	case <-p.lost:
		return 0, errUnplugged
	case <-time.After(timeout):
		return 0, nil
	}
}

func (p *fakePort) Write(b []byte) (int, error) {
	select {
	case <-p.lost:
		return 0, errUnplugged
	case <-p.closed:
		return 0, errClosed
	default:
	}
	p.mu.Lock()
	block, limit := p.blockWrites, p.maxWrite
	p.mu.Unlock()
	if block {
		// A stalled device: the write returns only when the port closes.
		<-p.closed
		return 0, errClosed
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	if limit > 0 && len(b) > limit {
		b = b[:limit] // a short write
	}
	p.out = append(p.out, b...)
	p.writes++
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

func (p *fakePort) isClosed() bool {
	select {
	case <-p.closed:
		return true
	default:
		return false
	}
}

// written decodes what the link wrote.
func (p *fakePort) written() []protocol.Frame {
	p.mu.Lock()
	data := append([]byte(nil), p.out...)
	p.mu.Unlock()
	var d protocol.Decoder
	var frames []protocol.Frame
	for _, b := range data {
		d.Push(b)
		for {
			f, ok := d.Next()
			if !ok {
				break
			}
			frames = append(frames, f)
		}
	}
	return frames
}

// fakeSystem lists one device while it is plugged in, and hands out a new
// fakePort on every open.
type fakeSystem struct {
	mu      sync.Mutex
	device  PortInfo
	present bool
	openErr error
	ports   []*fakePort
	opened  chan *fakePort
	// configure, when set, adjusts each port as it is opened.
	configure func(*fakePort)
}

func newFakeSystem(device PortInfo) *fakeSystem {
	return &fakeSystem{device: device, present: true, opened: make(chan *fakePort, 16)}
}

func (s *fakeSystem) List() ([]PortInfo, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !s.present {
		return []PortInfo{{Name: "/dev/ttyS0"}}, nil
	}
	return []PortInfo{{Name: "/dev/ttyS0"}, s.device}, nil
}

func (s *fakeSystem) Open(name string) (Port, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.openErr != nil {
		return nil, s.openErr
	}
	if !s.present || name != s.device.Name {
		return nil, errors.New("no such port")
	}
	p := newFakePort()
	if s.configure != nil {
		s.configure(p)
	}
	s.ports = append(s.ports, p)
	select {
	case s.opened <- p:
	default: // nobody is waiting for this many ports
	}
	return p, nil
}

func (s *fakeSystem) setPresent(present bool, name string) {
	s.mu.Lock()
	s.present = present
	if name != "" {
		s.device.Name = name
	}
	s.mu.Unlock()
}

// recorder is a Handler that keeps everything it receives.
type recorder struct {
	mu     sync.Mutex
	states []State
	frames []protocol.Frame
	lines  []Line
	stats  Stats
	events []string // "frame" or "line", in order
	notify chan struct{}
}

func newRecorder() *recorder { return &recorder{notify: make(chan struct{}, 1024)} }

func (r *recorder) LinkState(s State) {
	r.mu.Lock()
	r.states = append(r.states, s)
	r.mu.Unlock()
	r.notify <- struct{}{}
}

func (r *recorder) Frame(f protocol.Frame, s Stats) {
	r.mu.Lock()
	r.frames = append(r.frames, f)
	r.events = append(r.events, "frame")
	r.stats = s
	r.mu.Unlock()
	r.notify <- struct{}{}
}

func (r *recorder) Line(l Line, s Stats) {
	r.mu.Lock()
	r.lines = append(r.lines, l)
	r.events = append(r.events, "line")
	r.stats = s
	r.mu.Unlock()
	r.notify <- struct{}{}
}

// waitFor polls cond until it holds or the timeout passes.
func (r *recorder) waitFor(timeout time.Duration, cond func(r *recorder) bool) bool {
	deadline := time.After(timeout)
	for {
		r.mu.Lock()
		ok := cond(r)
		r.mu.Unlock()
		if ok {
			return true
		}
		select {
		case <-r.notify:
		case <-deadline:
			return false
		case <-time.After(5 * time.Millisecond):
		}
	}
}

// staticSource always sends the same command and an optional queue of sets.
type staticSource struct {
	mu   sync.Mutex
	cmd  protocol.Command
	sets []protocol.CalibrationSet
}

func (s *staticSource) Command() protocol.Command {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.cmd
}

func (s *staticSource) CalibrationSet(time.Time) (protocol.CalibrationSet, bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.sets) == 0 {
		return protocol.CalibrationSet{}, false
	}
	set := s.sets[0]
	s.sets = s.sets[1:]
	return set, true
}
