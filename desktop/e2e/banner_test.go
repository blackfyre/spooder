package e2e

import (
	"bytes"
	"os"
	"os/exec"
	"strings"
	"syscall"
	"testing"
	"time"
)

const bannerPrefix = "spooder robot: build "

// expectedIdentity is the build identity of the checkout the virtual robot
// was just built from (`mise run e2e` builds it first), after the banner's
// rules: bytes outside printable ASCII as `?`, cut at 40 characters, and
// `unknown` when git fails.
func expectedIdentity(t *testing.T) string {
	t.Helper()
	out, err := exec.Command("git", "describe", "--always", "--dirty", "--abbrev=12").Output()
	id := strings.TrimRight(string(out), "\r\n")
	if err != nil || id == "" {
		return "unknown"
	}
	b := []byte(id)
	for i, c := range b {
		if c < 0x20 || c > 0x7e {
			b[i] = '?'
		}
	}
	if len(b) > 40 {
		b = b[:40]
	}
	return string(b)
}

// readBanner opens the robot's USB terminal once, as a console would, and
// returns the identity of the first banner line it receives.
func readBanner(t *testing.T, r *Robot) string {
	t.Helper()
	f, err := os.OpenFile(r.Link, os.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var got []byte
	buf := make([]byte, 512)
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		f.SetReadDeadline(time.Now().Add(50 * time.Millisecond))
		n, _ := f.Read(buf)
		got = append(got, buf[:n]...)
		if i := bytes.Index(got, []byte(bannerPrefix)); i >= 0 {
			rest := got[i+len(bannerPrefix):]
			if j := bytes.IndexByte(rest, '\n'); j >= 0 {
				return string(rest[:j])
			}
		}
	}
	if len(got) > 200 {
		got = got[:200]
	}
	t.Fatalf("no %q line on the USB terminal; read %q", bannerPrefix, got)
	return ""
}

// Spec "Virtual robot" (build identity): opening the USB terminal shows the
// build banner with the checkout's identity, and so does each new open after
// the cable is replugged.
func TestBuildBannerOnEachConnect(t *testing.T) {
	t.Parallel()
	want := expectedIdentity(t)
	r := StartRobot(t)
	if got := readBanner(t, r); got != want {
		t.Fatalf("banner identity %q, want %q", got, want)
	}
	// The connection is sampled once per control tick: give the robot a few
	// ticks to see the terminal closed before it is opened again.
	r.Do("unplug")
	r.Do("plug")
	time.Sleep(100 * time.Millisecond)
	if got := readBanner(t, r); got != want {
		t.Fatalf("after replugging: banner identity %q, want %q", got, want)
	}
}
