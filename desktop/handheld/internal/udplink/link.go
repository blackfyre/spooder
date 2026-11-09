// Package udplink is the handheld's UDP link to the bridge's WiFi drive
// (openspec change add-handheld-controller, design D3): one connected
// socket per run, command frames every 20 ms after a 150 ms start-up wait,
// and the receiver that accepts only exactly one valid robot frame per
// datagram and keeps the link-quality counts.
package udplink

import (
	"context"
	"errors"
	"net"
	"sync"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

const (
	// DefaultAddr is the bridge's access point address and WiFi drive port.
	DefaultAddr = "192.168.4.1:52040"
	// Period is the command rate, 50 Hz.
	Period = 20 * time.Millisecond
	// StartWait is the quiet time after the socket is made: the bridge's
	// liveness timeout (150 ms), so a restart never continues the previous
	// run's session.
	StartWait = 150 * time.Millisecond
	// maxEvents bounds the queue of accepted frames between two takes.
	maxEvents = 256
)

// Outcome is what Run does with a step's command.
type Outcome int

const (
	// Send sends the command.
	Send Outcome = iota
	// Hold sends nothing this tick: no datagram, the sequence number
	// unchanged (the handheld off the bridge's network).
	Hold
	// Stop ends Run without sending.
	Stop
)

// Step returns the command for the tick at now and what to do with it.
type Step func(now time.Time) (cmd protocol.Command, out Outcome)

// Link is one run's socket, sender and receiver.
type Link struct {
	conn    *net.UDPConn
	created time.Time
	done    chan struct{}

	mu     sync.Mutex
	rx     *Receiver
	events []Event
	seq    uint8
}

// Dial makes the run's one socket, connected to addr, so the source port
// never changes and the kernel drops datagrams from any other address and
// port, and starts the receiver.
func Dial(addr string) (*Link, error) {
	raddr, err := net.ResolveUDPAddr("udp4", addr)
	if err != nil {
		return nil, err
	}
	conn, err := net.DialUDP("udp4", nil, raddr)
	if err != nil {
		return nil, err
	}
	now := time.Now()
	l := &Link{conn: conn, created: now, done: make(chan struct{}), rx: NewReceiver(now)}
	go l.receive()
	return l, nil
}

// LocalAddr is the socket's own address.
func (l *Link) LocalAddr() net.Addr { return l.conn.LocalAddr() }

// RemoteAddr is the bridge's address.
func (l *Link) RemoteAddr() net.Addr { return l.conn.RemoteAddr() }

// Close closes the socket and waits for the receiver to stop.
func (l *Link) Close() error {
	err := l.conn.Close()
	<-l.done
	return err
}

func (l *Link) receive() {
	defer close(l.done)
	// One byte larger than the largest robot frame, so a datagram with a
	// byte too many is seen as such.
	buf := make([]byte, protocol.MaxFrameSize+1)
	for {
		n, err := l.conn.Read(buf)
		if errors.Is(err, net.ErrClosed) {
			return
		}
		if err != nil {
			continue // an ICMP error from an earlier send: the bridge is not there yet
		}
		l.datagram(buf[:n], time.Now())
	}
}

func (l *Link) datagram(b []byte, now time.Time) {
	l.mu.Lock()
	defer l.mu.Unlock()
	ev, ok := l.rx.Datagram(b, now)
	if !ok {
		return
	}
	if len(l.events) == maxEvents {
		copy(l.events, l.events[1:])
		l.events = l.events[:maxEvents-1]
		l.rx.q.Dropped++
	}
	l.events = append(l.events, ev)
}

// TakeEvents returns the frames accepted since the last call, oldest
// first.
func (l *Link) TakeEvents() []Event {
	l.mu.Lock()
	defer l.mu.Unlock()
	ev := l.events
	l.events = nil
	return ev
}

// Status is the receiver's state at now, with the send counts.
func (l *Link) Status(now time.Time) Status {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.rx.Status(now)
}

// Rebaseline makes the next valid frame set the newest sequence number
// without counting anything lost or late: after a resume.
func (l *Link) Rebaseline() {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.rx.Rebaseline()
}

// Run waits out StartWait from the socket's creation, then sends step's
// command every Period, the sequence number advancing by one per frame,
// until ctx ends or step asks to stop; a held tick sends nothing and keeps
// the sequence number. A send error is counted and the next
// tick tries again. Ticks missed while step blocks (a suspend) are dropped,
// not sent late in a burst.
func (l *Link) Run(ctx context.Context, step Step) error {
	wait := time.NewTimer(time.Until(l.created.Add(StartWait)))
	defer wait.Stop()
	select {
	case <-ctx.Done():
		return nil
	case <-wait.C:
	}
	ticker := time.NewTicker(Period)
	defer ticker.Stop()
	for {
		began := time.Now()
		if l.send(step) {
			return nil
		}
		if time.Since(began) >= Period {
			// The step blocked past a tick (a suspend): start the ticks
			// afresh, so the tick it missed is not sent at once after it.
			ticker.Reset(Period)
		}
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
		}
	}
}

// send sends one tick's frame, unless step holds it; it reports whether
// step asked to stop.
func (l *Link) send(step Step) bool {
	cmd, out := step(time.Now())
	switch out {
	case Stop:
		return true
	case Hold:
		return false
	}
	l.mu.Lock()
	seq := l.seq
	l.seq++
	l.mu.Unlock()
	_, err := l.conn.Write(protocol.EncodeCommand(cmd, seq))
	l.mu.Lock()
	if err != nil {
		l.rx.q.SendErrors++
	} else {
		l.rx.q.Sent++
	}
	l.mu.Unlock()
	return false
}
