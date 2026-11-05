package wifiprobe

import (
	"bufio"
	"bytes"
	"context"
	"encoding/hex"
	"io"
	"net"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// golden reads one frame of test/golden/protocol_v3.txt by name.
func golden(t *testing.T, name string) []byte {
	t.Helper()
	f, err := os.Open(filepath.Join("..", "..", "..", "test", "golden", "protocol_v3.txt"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for sc.Scan() {
		parts := strings.Split(sc.Text(), " ")
		if len(parts) >= 2 && parts[0] == name {
			b, err := hex.DecodeString(parts[1])
			if err != nil {
				t.Fatal(err)
			}
			return b
		}
	}
	t.Fatalf("no golden frame %q", name)
	return nil
}

// arrival is one datagram the fake bridge received, at a time since the
// probe started.
type arrival struct {
	at    time.Duration
	bytes []byte
}

// bridge is a UDP listener on the loopback address standing in for the
// bridge. reply, if set, is sent back to the sender after its first
// datagram, one datagram per element.
type bridge struct {
	conn     *net.UDPConn
	begin    time.Time
	reply    [][]byte
	mu       sync.Mutex
	arrivals []arrival
	done     chan struct{}
}

func newBridge(t *testing.T, reply [][]byte) *bridge {
	t.Helper()
	conn, err := net.ListenUDP("udp4", &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1)})
	if err != nil {
		t.Fatal(err)
	}
	b := &bridge{conn: conn, reply: reply, done: make(chan struct{})}
	t.Cleanup(func() { conn.Close(); <-b.done })
	return b
}

func (b *bridge) serve() {
	defer close(b.done)
	buf := make([]byte, 2048)
	replied := false
	for {
		n, from, err := b.conn.ReadFromUDP(buf)
		if err != nil {
			return
		}
		b.mu.Lock()
		b.arrivals = append(b.arrivals, arrival{time.Since(b.begin), append([]byte(nil), buf[:n]...)})
		b.mu.Unlock()
		if !replied {
			replied = true
			for _, r := range b.reply {
				b.conn.WriteToUDP(r, from)
			}
		}
	}
}

// run runs the probe against the bridge and returns its result and output.
func (b *bridge) run(t *testing.T, cfg Config) (Result, string) {
	t.Helper()
	cfg.Addr = b.conn.LocalAddr().String()
	var out bytes.Buffer
	b.begin = time.Now()
	go b.serve()
	res, err := Run(context.Background(), cfg, &out)
	if err != nil {
		t.Fatal(err)
	}
	return res, out.String()
}

func (b *bridge) commands(t *testing.T) []struct {
	at time.Duration
	f  protocol.Frame
} {
	t.Helper()
	b.mu.Lock()
	defer b.mu.Unlock()
	var out []struct {
		at time.Duration
		f  protocol.Frame
	}
	for _, a := range b.arrivals {
		f, ok := classify(a.bytes)
		if !ok || f.Type != protocol.TypeCommand || len(a.bytes) != protocol.CommandFrameSize || a.bytes[1] != 3 {
			t.Fatalf("datagram %x is not one version 3 command frame", a.bytes)
		}
		out = append(out, struct {
			at time.Duration
			f  protocol.Frame
		}{a.at, f})
	}
	return out
}

// near fails unless at lies within [want - early, want + late].
func near(t *testing.T, what string, at, want time.Duration) {
	t.Helper()
	if at < want-5*time.Millisecond || at > want+3*Period {
		t.Errorf("%s at %v, want about %v", what, at, want)
	}
}

func TestProbeSendsCommandFrames(t *testing.T) {
	b := newBridge(t, nil)
	cfg := Config{
		Mode: "stand", ModeAfter: 300 * time.Millisecond,
		EStop: true, EStopClearAfter: 400 * time.Millisecond,
		ClearAfter: 500 * time.Millisecond,
		Duration:   StartWait + time.Second,
	}
	res, _ := b.run(t, cfg)
	frames := b.commands(t)

	if n := len(frames); n < 42 || n > 52 {
		t.Errorf("%d frames in one second, want about 50", n)
	}
	if res.Sent != len(frames) || res.Flood != 0 {
		t.Errorf("counted %d sent and %d flood, the bridge received %d", res.Sent, res.Flood, len(frames))
	}
	if len(frames) == 0 {
		t.FailNow()
	}
	near(t, "first frame", frames[0].at, StartWait)

	modeChanges, clearChanges, estopClears := 0, 0, 0
	for i, fr := range frames {
		c := fr.f.Command
		if c.Flags&protocol.FlagControllerConnected == 0 {
			t.Errorf("frame %d reports no controller", i)
		}
		if i == 0 {
			if c.ModeRequestCounter != 0 || c.ClearCounter != 0 || c.Mode != uint8(session.RequestSleep) ||
				c.Flags&protocol.FlagEmergencyStop == 0 {
				t.Errorf("first frame %+v, want counters 0, Sleep and the e-stop", c)
			}
			continue
		}
		prev := frames[i-1].f
		if fr.f.Seq != prev.Seq+1 {
			t.Errorf("frame %d has sequence %d after %d", i, fr.f.Seq, prev.Seq)
		}
		if c.ModeRequestCounter != prev.Command.ModeRequestCounter {
			modeChanges++
			near(t, "mode request", fr.at, cfg.ModeAfter)
			if c.Mode != uint8(session.RequestStand) {
				t.Errorf("mode request asks for %d, want Stand", c.Mode)
			}
		} else if c.Mode != prev.Command.Mode {
			t.Errorf("frame %d changes the mode without a request", i)
		}
		if c.ClearCounter != prev.Command.ClearCounter {
			clearChanges++
			near(t, "fault clear", fr.at, cfg.ClearAfter)
		}
		was, is := prev.Command.Flags&protocol.FlagEmergencyStop != 0, c.Flags&protocol.FlagEmergencyStop != 0
		if was && !is {
			estopClears++
			near(t, "e-stop clear", fr.at, cfg.EStopClearAfter)
		} else if !was && is {
			t.Errorf("frame %d sets the e-stop again", i)
		}
	}
	if modeChanges != 1 || clearChanges != 1 || estopClears != 1 {
		t.Errorf("%d mode requests, %d fault clears, %d e-stop clears; want one each", modeChanges, clearChanges, estopClears)
	}
}

func TestProbeFloodAddsCopies(t *testing.T) {
	b := newBridge(t, nil)
	res, _ := b.run(t, Config{Flood: 500, Duration: StartWait + 500*time.Millisecond})
	frames := b.commands(t)
	perSeq := map[uint8]int{}
	for _, fr := range frames {
		perSeq[fr.f.Seq]++
	}
	// 500 per second is 10 copies every 20 ms period.
	for seq, n := range perSeq {
		if n != 11 {
			t.Errorf("sequence %d arrived %d times, want the frame and 10 copies", seq, n)
		}
	}
	if res.Flood != 10*res.Sent || res.Sent < 20 {
		t.Errorf("sent %d and %d copies, want about 25 and ten times as many", res.Sent, res.Flood)
	}
}

func TestProbeFloodKeepsCadence(t *testing.T) {
	b := newBridge(t, nil)
	cfg := Config{Flood: 10_000_000, EStop: true, EStopClearAfter: 300 * time.Millisecond, Duration: StartWait + 500*time.Millisecond}
	start := time.Now()
	res, _ := b.run(t, cfg)
	if elapsed := time.Since(start); elapsed > cfg.Duration+100*time.Millisecond {
		t.Errorf("ran for %v, want about %v", elapsed, cfg.Duration)
	}
	if res.Sent < 22 || res.Sent > 26 {
		t.Errorf("sent %d frames in 500 ms under a flood, want about 25", res.Sent)
	}
	if res.FloodShort == 0 {
		t.Errorf("sent all %d copies of a flood beyond the budget", res.Flood)
	}
}

func TestProbePauses(t *testing.T) {
	b := newBridge(t, nil)
	cfg := Config{PauseAfter: 300 * time.Millisecond, PauseFor: 300 * time.Millisecond, Duration: 900 * time.Millisecond}
	b.run(t, cfg)
	frames := b.commands(t)
	gaps := 0
	for i := 1; i < len(frames); i++ {
		gap := frames[i].at - frames[i-1].at
		if gap > 100*time.Millisecond {
			gaps++
			if gap < cfg.PauseFor-Period || gap > cfg.PauseFor+3*Period {
				t.Errorf("pause of %v, want about %v", gap, cfg.PauseFor)
			}
			near(t, "pause end", frames[i].at, cfg.PauseAfter+cfg.PauseFor)
		}
	}
	if gaps != 1 {
		t.Errorf("%d pauses, want one", gaps)
	}
}

func TestProbeSummaryCountsRelayedFrames(t *testing.T) {
	telemetry := golden(t, "telemetry_conflict")
	extra := append(append([]byte(nil), telemetry...), 0)
	b := newBridge(t, [][]byte{telemetry, golden(t, "command_walk"), extra})
	res, out := b.run(t, Config{Duration: StartWait + 200*time.Millisecond, summaryEvery: 100 * time.Millisecond})
	if res.Received[protocol.TypeTelemetry] != 1 || res.Errors != 2 || res.Commands != 1 {
		t.Errorf("received %v, %d errors, %d command frames; want one telemetry frame and two errors, one a command frame",
			res.Received, res.Errors, res.Commands)
	}
	f, _ := classify(telemetry)
	if !res.HaveTelemetry || res.Telemetry != f.Telemetry {
		t.Errorf("latest telemetry %+v, want the golden frame's", res.Telemetry)
	}
	if !strings.Contains(out, "telemetry 1") || !strings.Contains(out, "errors 2 (command 1)") {
		t.Errorf("summaries do not show the counts:\n%s", out)
	}
}

func TestClassify(t *testing.T) {
	telemetry := golden(t, "telemetry_extremes")
	two := append(append([]byte(nil), telemetry...), telemetry...)
	for _, c := range []struct {
		name string
		b    []byte
		ok   bool
	}{
		{"telemetry", telemetry, true},
		{"pose", golden(t, "pose_extremes"), true},
		{"diagnostics", golden(t, "diagnostics_extremes"), true},
		{"calibration report", golden(t, "calibration_report_defaults"), true},
		{"one extra byte", append(append([]byte(nil), telemetry...), 0xA5), false},
		{"a leading byte", append([]byte{0}, telemetry...), false},
		{"two frames", two, false},
		{"truncated", telemetry[:len(telemetry)-1], false},
		{"bad CRC", golden(t, "reject_bad_crc_pose"), false},
		{"version 2", golden(t, "reject_version_2_command"), false},
		{"empty", nil, false},
	} {
		if _, ok := classify(c.b); ok != c.ok {
			t.Errorf("%s: classify = %v, want %v", c.name, ok, c.ok)
		}
	}
}

func TestParseFlags(t *testing.T) {
	c, err := ParseFlags(nil, io.Discard)
	if err != nil || c.Addr != DefaultAddr || c.Mode != "" || c.Flood != 0 {
		t.Errorf("defaults %+v, %v", c, err)
	}
	if _, err := ParseFlags([]string{"--mode", "dance"}, io.Discard); err == nil {
		t.Error("an unknown mode is accepted")
	}
	if _, err := ParseFlags([]string{"--flood", "-1"}, io.Discard); err == nil {
		t.Error("a negative flood is accepted")
	}
	c, err = ParseFlags([]string{"--mode", "walk", "--estop", "--estop-clear-after", "2s", "--clear-after", "3s"}, io.Discard)
	if err != nil || c.Mode != "walk" || !c.EStop || c.EStopClearAfter != 2*time.Second || c.ClearAfter != 3*time.Second {
		t.Errorf("parsed %+v, %v", c, err)
	}
}
