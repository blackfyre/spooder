// Package link carries the Spooder link protocol over the robot's USB
// serial port: one reader splits the stream into frames and debug text
// lines, one writer sends a command frame every period and at most one
// calibration set after it, and a supervisor reconnects to the same device
// after an unplug (design D3).
package link

import (
	"errors"
	"sync"
	"syscall"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Timing (design D3). Command frames go out every CommandPeriod, like the
// bridge's; a lost device is looked for again every RetryInterval.
const (
	CommandPeriod = 20 * time.Millisecond
	RetryInterval = 500 * time.Millisecond
	// ReadTimeout is how long a read waits for data. A read that returns
	// nothing means the line has been quiet that long, longer than any
	// frame takes on USB, so the decoder may abandon an incomplete
	// candidate.
	ReadTimeout = 20 * time.Millisecond
)

// Status is the connection state.
type Status int

const (
	Disconnected Status = iota // no device chosen, or released by the user
	Waiting                    // a device is chosen but cannot be opened yet
	Connected
)

func (s Status) String() string {
	switch s {
	case Disconnected:
		return "disconnected"
	case Waiting:
		return "waiting for device"
	case Connected:
		return "connected"
	}
	return "unknown"
}

// State is reported whenever the connection changes.
type State struct {
	Status Status
	Port   PortInfo // the device chosen; Name is the port currently or last open
	Err    error    // why the last open failed or the connection was lost
}

// Stats are the host side's decoding counts for the current connection.
type Stats struct {
	Valid, Discarded uint32 // frames decoded and candidates rejected
	Stray            uint32 // 0xA5 bytes outside frames
}

// Handler receives what the link reads. Calls come from the link's
// goroutines, one at a time per connection, in stream order.
type Handler interface {
	LinkState(s State)
	Frame(f protocol.Frame, s Stats)
	Line(l Line, s Stats)
}

// Source supplies what the writer sends.
type Source interface {
	// Command is the command frame payload to send now.
	Command() protocol.Command
	// CalibrationSet is the calibration set to send after the command, if
	// any, at most one per period.
	CalibrationSet(now time.Time) (protocol.CalibrationSet, bool)
}

// Options adjust the timing, for tests.
type Options struct {
	CommandPeriod time.Duration
	RetryInterval time.Duration
	ReadTimeout   time.Duration
	Now           func() time.Time
}

func (o Options) withDefaults() Options {
	if o.CommandPeriod == 0 {
		o.CommandPeriod = CommandPeriod
	}
	if o.RetryInterval == 0 {
		o.RetryInterval = RetryInterval
	}
	if o.ReadTimeout == 0 {
		o.ReadTimeout = ReadTimeout
	}
	if o.Now == nil {
		o.Now = time.Now
	}
	return o
}

// Link owns at most one serial port at a time.
type Link struct {
	sys  System
	h    Handler
	src  Source
	opts Options

	mu   sync.Mutex
	stop chan struct{} // closed to end the current supervisor
	done chan struct{} // closed when the supervisor has ended
}

// New returns an idle link.
func New(sys System, h Handler, src Source, opts Options) *Link {
	return &Link{sys: sys, h: h, src: src, opts: opts.withDefaults()}
}

// Connect starts using the device: it is opened now or as soon as it
// appears, and again after every loss, until Disconnect or Close. A previous
// device is released first.
func (l *Link) Connect(device PortInfo) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.release()
	l.stop = make(chan struct{})
	l.done = make(chan struct{})
	go l.supervise(device, l.stop, l.done)
}

// Disconnect releases the device: the writer sends one last command frame
// without the controller flag, then the port is closed. It returns once the
// port is closed.
func (l *Link) Disconnect() {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.release()
}

// release stops the supervisor and waits for it; l.mu is held, so
// connecting and disconnecting are serialised.
func (l *Link) release() {
	if l.stop == nil {
		return
	}
	close(l.stop)
	<-l.done
	l.stop, l.done = nil, nil
}

// Close is Disconnect, for shutting down.
func (l *Link) Close() { l.Disconnect() }

// supervise opens the device, runs the connection, and retries after every
// failure or loss until stop is closed.
func (l *Link) supervise(device PortInfo, stop, done chan struct{}) {
	defer close(done)
	defer l.h.LinkState(State{Status: Disconnected, Port: device})
	for {
		port, info, err := l.open(device)
		if err == nil {
			device = info
			l.h.LinkState(State{Status: Connected, Port: device})
			err = l.run(port, stop)
			if err == nil {
				return // stopped by the user
			}
		}
		l.h.LinkState(State{Status: Waiting, Port: device, Err: err})
		select {
		case <-stop:
			return
		case <-time.After(l.opts.RetryInterval):
		}
	}
}

// errNotPresent means the device is not among the computer's ports.
var errNotPresent = errors.New("device not present")

// open finds the device among the current ports and opens it.
func (l *Link) open(device PortInfo) (Port, PortInfo, error) {
	ports, err := l.sys.List()
	if err != nil {
		return nil, device, err
	}
	found := false
	for _, p := range ports {
		if SameDevice(device, p) {
			device, found = p, true
			break
		}
	}
	if !found {
		return nil, device, errNotPresent
	}
	port, err := l.sys.Open(device.Name)
	if err != nil {
		return nil, device, err
	}
	if err := port.SetReadTimeout(l.opts.ReadTimeout); err != nil {
		port.Close()
		return nil, device, err
	}
	return port, device, nil
}

// ErrLost is reported when the port fails while connected.
var ErrLost = errors.New("connection lost")

// run serves one open port until stop is closed (nil) or the port fails.
func (l *Link) run(port Port, stop chan struct{}) error {
	readErr := make(chan error, 1)
	readDone := make(chan struct{})
	go func() {
		defer close(readDone)
		readErr <- l.read(port)
	}()

	// A write that blocks (a stalled device) must not hold up a disconnect:
	// once stop is closed, the writer gets CloseGrace to finish, then the
	// port is closed under it, which ends any pending write.
	writerDone := make(chan struct{})
	defer close(writerDone)
	go func() {
		select {
		case <-stop:
			select {
			case <-writerDone:
			case <-time.After(CloseGrace):
				port.Close()
			}
		case <-writerDone:
		}
	}()

	ticker := time.NewTicker(l.opts.CommandPeriod)
	defer ticker.Stop()
	var seq uint8
	write := func(frame []byte) error { return writeFrame(port, frame, l.opts.CommandPeriod) }
	var err error
loop:
	for {
		select {
		case <-stop:
			// The last frame never claims control, whatever the source says.
			c := l.src.Command()
			c.Flags &^= protocol.FlagControllerConnected
			_ = write(protocol.EncodeCommand(c, seq))
			break loop
		case e := <-readErr:
			err = e
			break loop
		case <-ticker.C:
			if e := write(protocol.EncodeCommand(l.src.Command(), seq)); e != nil {
				err = e
				break loop
			}
			seq++
			if set, ok := l.src.CalibrationSet(l.opts.Now()); ok {
				if e := write(protocol.EncodeCalibrationSet(set, seq)); e != nil {
					err = e
					break loop
				}
				seq++
			}
		}
	}
	port.Close()
	<-readDone
	if err != nil {
		return errors.Join(ErrLost, err)
	}
	return nil
}

// CloseGrace is how long a disconnect waits for a pending write.
const CloseGrace = 200 * time.Millisecond

// errWriteStalled means a frame could not be written completely in time.
var errWriteStalled = errors.New("write stalled")

// writeFrame writes a whole frame. A full transmit buffer before the first
// byte drops the frame, as the robot does with its own; a frame begun is
// completed within limit, or the connection is treated as failed, since a
// device that stops reading mid-frame is not working.
func writeFrame(port Port, frame []byte, limit time.Duration) error {
	deadline := time.Now().Add(limit)
	for written := 0; written < len(frame); {
		n, err := port.Write(frame[written:])
		written += max(n, 0)
		switch {
		case errors.Is(err, syscall.EAGAIN) || (err == nil && n == 0):
			if written == 0 {
				return nil // dropped whole; the robot's decoder resynchronises
			}
			if time.Now().After(deadline) {
				return errWriteStalled
			}
			time.Sleep(time.Millisecond)
		case err != nil:
			return err
		}
	}
	return nil
}

// read feeds the decoder until the port fails or is closed.
func (l *Link) read(port Port) error {
	var stats Stats
	text := textAccumulator{}
	text.emit = func(line Line) { l.h.Line(line, stats) }
	dec := protocol.Decoder{Discard: func(b byte) {
		if b == protocol.Start {
			stats.Stray++
		}
		text.add(b)
	}}
	next := func() {
		for {
			f, ok := dec.Next()
			if !ok {
				return
			}
			stats.Valid, stats.Discarded = dec.Valid(), dec.Discarded()
			l.h.Frame(f, stats)
		}
	}
	buf := make([]byte, 512)
	for {
		n, err := port.Read(buf)
		for _, b := range buf[:n] {
			dec.Push(b)
			next()
		}
		stats.Valid, stats.Discarded = dec.Valid(), dec.Discarded()
		if err != nil {
			// End of stream: abandon every incomplete candidate, so the
			// bytes behind each reach the text or a frame.
			for dec.Pending() > 0 {
				dec.Idle()
				next()
			}
			if text.pending() {
				text.flush()
			}
			return err
		}
		if n == 0 {
			// The line has been quiet for the read timeout.
			dec.Idle()
			next()
		}
	}
}
