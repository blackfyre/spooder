package wlan

import (
	"context"
	"errors"
	"net"
	"os"
	"strings"
	"testing"
)

// fakeRunner records commands and answers them from a table keyed by the
// command line; after, when set, runs after each command.
type fakeRunner struct {
	calls   []string
	answers map[string]string
	fail    map[string]bool
	after   func(cmd string)
}

func (f *fakeRunner) Run(_ context.Context, name string, args ...string) ([]byte, error) {
	cmd := strings.Join(append([]string{name}, args...), " ")
	f.calls = append(f.calls, cmd)
	if f.after != nil {
		f.after(cmd)
	}
	if f.fail[cmd] {
		return nil, errors.New("exit status 1")
	}
	return []byte(f.answers[cmd]), nil
}

func (f *fakeRunner) count(cmd string) int {
	n := 0
	for _, c := range f.calls {
		if c == cmd {
			n++
		}
	}
	return n
}

// fakeAddrs is wlan0 holding ip, which renew may change.
type fakeAddrs struct{ ip string }

func (a *fakeAddrs) addrs(iface string) ([]net.IP, error) {
	if iface != Interface {
		return nil, errors.New("no such interface")
	}
	if a.ip == "" {
		return nil, nil
	}
	return []net.IP{net.ParseIP("fe80::1"), net.ParseIP(a.ip)}, nil
}

const renew = "udhcpc -i wlan0 -n -q -t 5"

const link = "iw dev wlan0 link"

// onNetwork answers iw's link as wlan0 on ssid.
func onNetwork(ssid string) map[string]string {
	return map[string]string{link: "Connected to 02:00:00:00:00:00 (on wlan0)\n\tSSID: " + ssid + "\n\tfreq: 2412\n"}
}

func TestJoined(t *testing.T) {
	run := &fakeRunner{answers: onNetwork("spooder-drive-1a2b")}
	a := &fakeAddrs{ip: "192.168.4.2"}
	ssid, err := CheckNetwork(context.Background(), run, a.addrs)
	if err != nil || ssid != "spooder-drive-1a2b" {
		t.Fatalf("got %q, %v; want spooder-drive-1a2b", ssid, err)
	}
	if run.count(renew) != 0 {
		t.Fatalf("ran %v, want no renewal", run.calls)
	}
}

func TestStaleLeaseRenewed(t *testing.T) {
	a := &fakeAddrs{ip: "192.168.0.23"}
	run := &fakeRunner{answers: onNetwork("spooder-drive-1a2b"), after: func(cmd string) {
		if cmd == renew {
			a.ip = "192.168.4.2"
		}
	}}
	if _, err := CheckNetwork(context.Background(), run, a.addrs); err != nil {
		t.Fatal(err)
	}
	if run.count(renew) != 1 {
		t.Fatalf("ran %v, want one %q", run.calls, renew)
	}
}

func TestNotJoinedNamesTheNetwork(t *testing.T) {
	a := &fakeAddrs{ip: "192.168.0.23"}
	run := &fakeRunner{answers: onNetwork("spooder-drive-1a2b")}
	_, err := CheckNetwork(context.Background(), run, a.addrs)
	var nj *NotJoinedError
	if !errors.As(err, &nj) {
		t.Fatalf("got %v, want a NotJoinedError", err)
	}
	if run.count(renew) != 1 {
		t.Fatalf("ran %v, want one renewal", run.calls)
	}
	msg := err.Error()
	for _, want := range []string{"spooder-drive-XXXX", "NextUI's WiFi settings", "bridge's link page", `"spooder-drive-1a2b"`} {
		if !strings.Contains(msg, want) {
			t.Fatalf("join message %q lacks %q", msg, want)
		}
	}
}

// A network that is not a bridge's WiFi drive fails by its name, however
// wlan0's address reads, and without a renewal (handheld-network-check, D1):
// the setup network hands out 192.168.4.x too.
func TestOtherNetworkRefusedByName(t *testing.T) {
	for _, ssid := range []string{"spooder-1a2b", "Home WiFi", "spooder-drive-1A2B", "spooder-drive-1a2b3", "spooder-drive-1a2", "xspooder-drive-1a2b"} {
		a := &fakeAddrs{ip: "192.168.4.2"}
		run := &fakeRunner{answers: onNetwork(ssid)}
		_, err := CheckNetwork(context.Background(), run, a.addrs)
		var nj *NotJoinedError
		if !errors.As(err, &nj) || nj.Network != ssid {
			t.Fatalf("%q: got %v, want the join message naming it", ssid, err)
		}
		if run.count(renew) != 0 {
			t.Fatalf("%q: ran %v, want no renewal", ssid, run.calls)
		}
		if !strings.Contains(err.Error(), `"`+ssid+`"`) {
			t.Fatalf("%q: join message %q does not name it", ssid, err)
		}
	}
}

func TestNotJoinedOnNoNetwork(t *testing.T) {
	for _, run := range []*fakeRunner{
		{answers: map[string]string{link: "Not connected.\n"}},
		{fail: map[string]bool{link: true}},
	} {
		a := &fakeAddrs{ip: "192.168.4.2"}
		_, err := CheckNetwork(context.Background(), run, a.addrs)
		if err == nil || !strings.Contains(err.Error(), "on no network") {
			t.Fatalf("got %v, want the join message naming no network", err)
		}
		if run.count(renew) != 0 {
			t.Fatalf("ran %v, want no renewal", run.calls)
		}
	}
}

func TestPowerSaveOnIsRestored(t *testing.T) {
	run := &fakeRunner{answers: map[string]string{"iw dev wlan0 get power_save": "Power save: on\n"}}
	ctx := context.Background()
	p, err := ReadPowerSave(ctx, run)
	if err != nil {
		t.Fatal(err)
	}
	if err := p.Off(ctx); err != nil {
		t.Fatal(err)
	}
	if err := p.Restore(ctx); err != nil {
		t.Fatal(err)
	}
	want := []string{"iw dev wlan0 get power_save", "iw dev wlan0 set power_save off", "iw dev wlan0 set power_save on"}
	if strings.Join(run.calls, "|") != strings.Join(want, "|") {
		t.Fatalf("ran %v, want %v", run.calls, want)
	}
}

func TestPowerSaveOffStaysOff(t *testing.T) {
	run := &fakeRunner{answers: map[string]string{"iw dev wlan0 get power_save": "Power save: off\n"}}
	ctx := context.Background()
	p, err := ReadPowerSave(ctx, run)
	if err != nil {
		t.Fatal(err)
	}
	_ = p.Off(ctx)
	_ = p.Restore(ctx)
	if run.count("iw dev wlan0 set power_save on") != 0 {
		t.Fatalf("ran %v: power save turned on", run.calls)
	}
}

func TestPowerSaveUnreadable(t *testing.T) {
	for name, run := range map[string]*fakeRunner{
		"fails":   {fail: map[string]bool{"iw dev wlan0 get power_save": true}},
		"garbled": {answers: map[string]string{"iw dev wlan0 get power_save": "command failed: No such device (-19)\n"}},
	} {
		if _, err := ReadPowerSave(context.Background(), run); err == nil {
			t.Fatalf("%s: read succeeded", name)
		}
	}
}

const wireless = `Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   70.  -40.  -256        0      0      0      0      0        0
`

const iwLink = `Connected to 02:00:00:00:00:00 (on wlan0)
	SSID: spooder-drive-1a2b
	freq: 2412
	signal: -47 dBm
	rx bitrate: 72.2 MBit/s
`

func TestRead(t *testing.T) {
	read := func(content string, err error) func(string) ([]byte, error) {
		return func(path string) ([]byte, error) {
			if path != WirelessPath {
				t.Fatalf("read %s", path)
			}
			return []byte(content), err
		}
	}
	ctx := context.Background()

	// The kernel's statistics give the level; iw still gives the network.
	run := &fakeRunner{answers: map[string]string{link: iwLink}}
	want := Sample{NetworkKnown: true, Network: "spooder-drive-1a2b", SignalKnown: true, SignalDBm: -40}
	if got := Read(ctx, run, read(wireless, nil)); got != want {
		t.Fatalf("got %+v, want %+v", got, want)
	}
	if run.count(link) != 1 {
		t.Fatalf("ran %v, want one iw", run.calls)
	}
	// No file (the TrimUI) or no wlan0 line: the level from the same iw.
	noLine := strings.Join(strings.Split(wireless, "\n")[:2], "\n")
	want.SignalDBm = -47
	for _, c := range []struct {
		content string
		err     error
	}{{"", os.ErrNotExist}, {noLine, nil}} {
		if got := Read(ctx, run, read(c.content, c.err)); got != want {
			t.Fatalf("from iw: got %+v, want %+v", got, want)
		}
	}
	// Not connected: no network, no level, both known to be so.
	notConnected := &fakeRunner{answers: map[string]string{link: "Not connected.\n"}}
	if got := Read(ctx, notConnected, read("", os.ErrNotExist)); got != (Sample{NetworkKnown: true}) {
		t.Fatalf("not connected: got %+v", got)
	}
	// iw failing: the network unknown, the statistics' level still read.
	failing := &fakeRunner{fail: map[string]bool{link: true}}
	if got := Read(ctx, failing, read("", os.ErrNotExist)); got != (Sample{}) {
		t.Fatalf("iw failing: got %+v", got)
	}
	if got := Read(ctx, failing, read(wireless, nil)); got != (Sample{SignalKnown: true, SignalDBm: -40}) {
		t.Fatalf("iw failing with statistics: got %+v", got)
	}
}
