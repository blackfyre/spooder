package link

import (
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// Concurrent Connect and Disconnect calls never leave a second supervisor
// running: after Close, every port opened is closed.
func TestConcurrentConnectAndDisconnectAreSerialised(t *testing.T) {
	sys := newFakeSystem(servo)
	l := New(sys, newRecorder(), &staticSource{}, fast)
	var wg sync.WaitGroup
	for i := 0; i < 20; i++ {
		wg.Add(2)
		go func() { defer wg.Done(); l.Connect(servo) }()
		go func() { defer wg.Done(); l.Disconnect() }()
	}
	wg.Wait()
	l.Close()
	time.Sleep(3 * fast.RetryInterval) // a leaked supervisor would reopen
	sys.mu.Lock()
	defer sys.mu.Unlock()
	for i, p := range sys.ports {
		if !p.isClosed() {
			t.Fatalf("port %d of %d left open", i, len(sys.ports))
		}
	}
}

// Short writes are completed, so every frame arrives whole.
func TestShortWritesAreCompleted(t *testing.T) {
	sys := newFakeSystem(servo)
	sys.configure = func(p *fakePort) { p.maxWrite = 3 }
	src := &staticSource{cmd: protocol.Command{Mode: 2, Speed: 44}, sets: []protocol.CalibrationSet{{RequestID: 7, Joint: 1}}}
	l := New(sys, newRecorder(), src, fast)
	l.Connect(servo)
	p := <-sys.opened
	time.Sleep(10 * fast.CommandPeriod)
	l.Close()
	frames := p.written()
	if len(frames) < 4 || frames[1].Type != protocol.TypeCalibrationSet {
		t.Fatalf("%d frames", len(frames))
	}
	var d protocol.Decoder
	for _, b := range p.out {
		d.Push(b)
		for {
			if _, ok := d.Next(); !ok {
				break
			}
		}
	}
	if d.Discarded() != 0 {
		t.Fatalf("%d broken frames", d.Discarded())
	}
}

// A write that never returns does not hold up a disconnect.
func TestStalledWriteDoesNotBlockDisconnect(t *testing.T) {
	sys := newFakeSystem(servo)
	sys.configure = func(p *fakePort) { p.blockWrites = true }
	l := New(sys, newRecorder(), &staticSource{}, fast)
	l.Connect(servo)
	p := <-sys.opened
	time.Sleep(3 * fast.CommandPeriod) // the writer is now stuck
	began := time.Now()
	l.Disconnect()
	if d := time.Since(began); d > CloseGrace+500*time.Millisecond {
		t.Fatalf("disconnect took %v", d)
	}
	if !p.isClosed() {
		t.Fatal("port left open")
	}
}

// At the end of the stream every incomplete candidate is abandoned, so the
// bytes behind nested false starts still reach the text.
func TestEndOfStreamFlushesNestedFalseStarts(t *testing.T) {
	_, sys, rec, p := start(t, nil)
	_ = sys
	p.send([]byte{0xA5, 0x02, 0x04, 0x00, 0x80, 0xA5, 0x02, 0x04, 0x00, 0x80, 't', 'a', 'i', 'l'})
	time.Sleep(time.Millisecond)
	p.unplug()
	if !rec.waitFor(time.Second, func(r *recorder) bool {
		for _, l := range r.lines {
			if strings.HasSuffix(l.Text, "tail") {
				return true
			}
		}
		return false
	}) {
		rec.mu.Lock()
		defer rec.mu.Unlock()
		t.Fatalf("lines %+v", rec.lines)
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	var stray int
	for _, l := range rec.lines {
		stray += l.Stray
	}
	if stray != 2 {
		t.Fatalf("stray %d", stray)
	}
}

// A write that blocks during normal operation is treated as a loss after
// WriteTimeout, and the device is reopened.
func TestStalledWriteIsALoss(t *testing.T) {
	sys := newFakeSystem(servo)
	first := true
	sys.configure = func(p *fakePort) {
		p.blockWrites = first
		first = false
	}
	rec := newRecorder()
	l := New(sys, rec, &staticSource{}, fast)
	l.Connect(servo)
	defer l.Close()
	<-sys.opened
	select {
	case <-sys.opened:
	case <-time.After(WriteTimeout + time.Second):
		t.Fatal("not reopened after a stalled write")
	}
	rec.mu.Lock()
	defer rec.mu.Unlock()
	var waiting bool
	for _, s := range rec.states {
		waiting = waiting || s.Status == Waiting
	}
	if !waiting {
		t.Fatal("loss not reported")
	}
}
