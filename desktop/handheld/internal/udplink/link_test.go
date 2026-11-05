package udplink

import (
	"context"
	"net"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// listener stands in for the bridge on the loopback address.
type listener struct {
	conn *net.UDPConn
}

func newListener(t *testing.T) *listener {
	t.Helper()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { conn.Close() })
	return &listener{conn: conn}
}

func (b *listener) addr() string { return b.conn.LocalAddr().String() }

type arrival struct {
	at    time.Time
	from  *net.UDPAddr
	bytes []byte
}

// collect reads datagrams until the deadline.
func (b *listener) collect(t *testing.T, until time.Time) []arrival {
	t.Helper()
	var got []arrival
	buf := make([]byte, 2048)
	for {
		if err := b.conn.SetReadDeadline(until); err != nil {
			t.Fatal(err)
		}
		n, from, err := b.conn.ReadFromUDP(buf)
		if err != nil {
			return got
		}
		got = append(got, arrival{at: time.Now(), from: from, bytes: append([]byte(nil), buf[:n]...)})
	}
}

func TestStartWaitRateSequenceAndSourcePort(t *testing.T) {
	b := newListener(t)
	t0 := time.Now()
	l, err := Dial(b.addr())
	if err != nil {
		t.Fatal(err)
	}
	defer l.Close()
	ctx, cancel := context.WithCancel(context.Background())
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		_ = l.Run(ctx, func(time.Time) (protocol.Command, bool) {
			return protocol.Command{Mode: 2, Speed: 44, Flags: protocol.FlagControllerConnected}, false
		})
	}()
	got := b.collect(t, t0.Add(StartWait+1300*time.Millisecond))
	cancel()
	wg.Wait()

	if len(got) == 0 {
		t.Fatal("no datagram")
	}
	if early := got[0].at.Sub(t0); early < StartWait {
		t.Fatalf("first datagram %v after start, want none in the first %v", early, StartWait)
	}
	window := got[0].at.Add(time.Second)
	n := 0
	for i, a := range got {
		if a.from.Port != got[0].from.Port || !a.from.IP.Equal(got[0].from.IP) {
			t.Fatalf("datagram %d from %v, the first from %v", i, a.from, got[0].from)
		}
		f, ok := classify(a.bytes)
		if !ok || f.Type != protocol.TypeCommand {
			t.Fatalf("datagram %d is not exactly one command frame: % x", i, a.bytes)
		}
		if f.Seq != uint8(i) {
			t.Fatalf("datagram %d has sequence %d, want %d", i, f.Seq, uint8(i))
		}
		if a.at.Before(window) {
			n++
		}
	}
	if n < 45 || n > 55 {
		t.Fatalf("%d datagrams in the second after the first, want 45 to 55", n)
	}
	// The sender may have sent one more after the listener stopped reading.
	if st := l.Status(time.Now()); st.Sent < len(got) || st.Sent > len(got)+1 || st.SendErrors != 0 {
		t.Fatalf("sent %d (errors %d), the listener got %d", st.Sent, st.SendErrors, len(got))
	}
}

// waitFor polls the link's status until ok holds or a second has passed.
func waitFor(t *testing.T, l *Link, what string, ok func(Status) bool) Status {
	t.Helper()
	deadline := time.Now().Add(time.Second)
	for {
		st := l.Status(time.Now())
		if ok(st) {
			return st
		}
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s: %+v", what, st.Quality)
		}
		time.Sleep(2 * time.Millisecond)
	}
}

func TestReceivedFramesValidation(t *testing.T) {
	b := newListener(t)
	l, err := Dial(b.addr())
	if err != nil {
		t.Fatal(err)
	}
	defer l.Close()
	to := l.LocalAddr().(*net.UDPAddr)
	send := func(conn *net.UDPConn, p []byte) {
		t.Helper()
		if _, err := conn.WriteToUDP(p, to); err != nil {
			t.Fatal(err)
		}
	}

	tel := protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: 2, VoltageMv: 7400}, 5)
	badCRC := append([]byte(nil), tel...)
	badCRC[len(badCRC)-1] ^= 0xFF
	invalid := [][]byte{
		badCRC,
		append(append([]byte(nil), tel...), 0x00),
		append(append([]byte(nil), tel...), protocol.EncodeTelemetry(protocol.Telemetry{}, 6)...),
		protocol.EncodeCommand(protocol.Command{}, 7),
		protocol.EncodeCalibrationSet(protocol.CalibrationSet{}, 8),
	}
	for i, p := range invalid {
		send(b.conn, p)
		want := i + 1
		waitFor(t, l, "the invalid count", func(s Status) bool { return s.Invalid == want })
	}

	// A valid robot frame from another socket never reaches the receiver.
	other, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	defer other.Close()
	send(other, protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: 3, VoltageMv: 1}, 40))
	// Then a valid one from the bridge, which shows the path works and
	// that it arrived after the foreign one.
	send(b.conn, protocol.EncodeTelemetry(protocol.Telemetry{BodyMode: 1, VoltageMv: 7000}, 9))
	st := waitFor(t, l, "the bridge's telemetry", func(s Status) bool {
		return s.Shown[protocol.TypeTelemetry].Have
	})
	shown := st.Shown[protocol.TypeTelemetry].Frame
	if shown.Seq != 9 || shown.Telemetry.VoltageMv != 7000 {
		t.Fatalf("shown telemetry %+v, want the bridge's sequence 9", shown)
	}
	if st.Invalid != len(invalid) || st.Lost != 0 || st.Late != 0 {
		t.Fatalf("counts %+v after the foreign frame", st.Quality)
	}
	for _, ft := range []protocol.FrameType{protocol.TypePose, protocol.TypeDiagnostics, protocol.TypeCalibrationReport} {
		if st.Shown[ft].Have {
			t.Fatalf("type %d shown from an invalid datagram", ft)
		}
	}
	ev := l.TakeEvents()
	if len(ev) != 1 || ev[0].Frame.Seq != 9 || !ev[0].Rebaseline {
		t.Fatalf("events %+v, want only the bridge's telemetry, rebaselining", ev)
	}
}

func TestBlockedStepDoesNotBurst(t *testing.T) {
	b := newListener(t)
	l, err := Dial(b.addr())
	if err != nil {
		t.Fatal(err)
	}
	defer l.Close()
	ctx, cancel := context.WithCancel(context.Background())
	var wg sync.WaitGroup
	wg.Add(1)
	calls := 0
	go func() {
		defer wg.Done()
		_ = l.Run(ctx, func(time.Time) (protocol.Command, bool) {
			calls++
			if calls == 3 {
				time.Sleep(110 * time.Millisecond) // a suspend
			}
			return protocol.Command{}, false
		})
	}()
	got := b.collect(t, time.Now().Add(StartWait+400*time.Millisecond))
	cancel()
	wg.Wait()
	if len(got) < 6 {
		t.Fatalf("%d datagrams, want at least 6", len(got))
	}
	// Datagram 2 is the blocked step's; 3 must follow a whole period
	// later, not at once (allowing for scheduling jitter).
	if gap := got[3].at.Sub(got[2].at); gap < Period/2 {
		t.Fatalf("the frame after the blocked step followed it by %v, want about %v", gap, Period)
	}
	if gap := got[2].at.Sub(got[1].at); gap < 100*time.Millisecond {
		t.Fatalf("the blocked step's frame came %v after the one before, want after the block", gap)
	}
}
