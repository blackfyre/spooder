package console

import (
	"errors"
	"os"
	"sync"
	"time"

	"go.bug.st/serial"

	"github.com/blackfyre/spooder/desktop/internal/link"
)

// OpenSerial opens a port like link.Serial, with DTR and RTS raised. A
// pseudo-terminal has no modem lines, so when setting them fails the port is
// opened again without them (design D3).
func OpenSerial(name string) (link.Port, error) {
	p, err := link.Serial{}.Open(name)
	var pe *serial.PortError
	if err != nil && errors.As(err, &pe) && pe.Code() == serial.InvalidSerialPort {
		return serial.Open(name, &serial.Mode{BaudRate: link.BaudRate})
	}
	return p, err
}

// SerialSystem is the computer's serial ports, opened with OpenSerial.
type SerialSystem struct{}

// List returns the serial ports with their USB details.
func (SerialSystem) List() ([]link.PortInfo, error) { return link.Serial{}.List() }

// Open opens a port with OpenSerial.
func (SerialSystem) Open(name string) (link.Port, error) { return OpenSerial(name) }

// system is the monitor's view of the ports: the base system's, plus a
// named path that USB enumeration does not list (a pseudo-terminal), and
// every port it opens is guarded.
type system struct {
	base   link.System
	path   string            // --port, or ""
	exists func(string) bool // whether a path is present
	gate   *gate
}

func pathExists(name string) bool {
	_, err := os.Stat(name)
	return err == nil
}

func (s *system) List() ([]link.PortInfo, error) {
	ports, err := s.base.List()
	if s.path == "" {
		return ports, err
	}
	for _, p := range ports {
		if p.Name == s.path {
			return ports, err
		}
	}
	if s.exists(s.path) {
		// Enumeration failing must not hide a port named outright.
		return append(ports, link.PortInfo{Name: s.path}), nil
	}
	return ports, err
}

func (s *system) Open(name string) (link.Port, error) {
	p, err := s.base.Open(name)
	if err != nil {
		return nil, err
	}
	return s.gate.attach(p), nil
}

// errReadOnly is returned by every write through the link: the monitor only
// observes (design D2).
var errReadOnly = errors.New("spooder-monitor never writes to the robot")

// guardedPort refuses writes. Only the gate's key forwarding writes, to the
// port underneath.
type guardedPort struct {
	link.Port
	gate *gate
}

func (p *guardedPort) Write([]byte) (int, error) { return 0, errReadOnly }

func (p *guardedPort) Close() error {
	p.gate.detach(p)
	return p.Port.Close()
}

// gate tracks the open port for key forwarding.
type gate struct {
	mu      sync.Mutex
	current *guardedPort
}

func (g *gate) attach(p link.Port) *guardedPort {
	gp := &guardedPort{Port: p, gate: g}
	g.mu.Lock()
	g.current = gp
	g.mu.Unlock()
	return gp
}

func (g *gate) detach(p *guardedPort) {
	g.mu.Lock()
	if g.current == p {
		g.current = nil
	}
	g.mu.Unlock()
}

// errNotConnected means a key arrived while no port was open.
var errNotConnected = errors.New("not connected")

// send writes key bytes to the open port. A write that blocks longer than
// the link's WriteTimeout closes the port, which the link then reports as
// a loss.
func (g *gate) send(b []byte) error {
	g.mu.Lock()
	p := g.current
	g.mu.Unlock()
	if p == nil {
		return errNotConnected
	}
	timer := time.AfterFunc(link.WriteTimeout, func() { p.Close() })
	defer timer.Stop()
	for len(b) > 0 {
		n, err := p.Port.Write(b)
		if err != nil {
			return err
		}
		if n == 0 {
			time.Sleep(time.Millisecond)
		}
		b = b[n:]
	}
	return nil
}

// Printable keeps the bytes that may be forwarded: printable ASCII only, so
// no forwarded byte can be a frame's start marker (0xA5).
func Printable(b []byte) []byte {
	out := b[:0:0]
	for _, c := range b {
		if c >= 0x20 && c <= 0x7E {
			out = append(out, c)
		}
	}
	return out
}
