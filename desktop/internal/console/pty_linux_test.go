package console

import (
	"os"
	"strconv"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"

	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// openPty returns a pseudo-terminal's master and the path of its slave.
func openPty(t *testing.T) (*os.File, string) {
	t.Helper()
	m, err := os.OpenFile("/dev/ptmx", os.O_RDWR, 0)
	if err != nil {
		t.Skipf("no pseudo-terminals: %v", err)
	}
	t.Cleanup(func() { m.Close() })
	n, err := unix.IoctlGetInt(int(m.Fd()), unix.TIOCGPTN)
	if err != nil {
		t.Fatal(err)
	}
	if err := unix.IoctlSetPointerInt(int(m.Fd()), unix.TIOCSPTLCK, 0); err != nil {
		t.Fatal(err)
	}
	return m, "/dev/pts/" + strconv.Itoa(n)
}

// --port accepts a pseudo-terminal, which has no modem lines; text and
// frames written to the other side arrive, and only typed keys come back.
func TestPseudoTerminalPort(t *testing.T) {
	master, path := openPty(t)
	p, err := (link.Serial{}).Open(path)
	if err != nil {
		t.Fatalf("open %s: %v", path, err)
	}
	p.Close()
	keys := newKeyInput()
	m := startMonitorEnv(t, Config{Port: path, Frames: true, TelemetryEvery: 1, Keys: true}, Env{System: link.Serial{}, Keys: keys})
	m.out.waitFor(t, "-- connected "+path)
	frame := protocol.EncodeTelemetry(calibrating, 5)
	if _, err := master.Write(append(append([]byte("hello from the pty\r\n"), frame...), "bye\r\n"...)); err != nil {
		t.Fatal(err)
	}
	m.out.waitFor(t, "bye")
	keys.ch <- []byte{'s', protocol.Start}
	buf := make([]byte, 16)
	if err := master.SetReadDeadline(time.Now().Add(2 * time.Second)); err != nil {
		t.Fatal(err)
	}
	n, err := master.Read(buf)
	if err != nil || string(buf[:n]) != "s" {
		t.Fatalf("master read %q %v", buf[:n], err)
	}
	m.stop()
	out := m.out.String()
	for _, s := range []string{"  hello from the pty\n", "  > telemetry #5  Calibrate", "  bye\n"} {
		if !strings.Contains(out, s) {
			t.Errorf("no %q in:\n%s", s, out)
		}
	}
}

// KeyMode clears canonical input and echo, keeps signals, and restores.
func TestKeyModeOnATerminal(t *testing.T) {
	_, path := openPty(t)
	slave, err := os.OpenFile(path, os.O_RDWR|unix.O_NOCTTY, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer slave.Close()
	fd := int(slave.Fd())
	before, err := unix.IoctlGetTermios(fd, ioctlGetTermios)
	if err != nil {
		t.Fatal(err)
	}
	restore, err := KeyMode(fd)
	if err != nil {
		t.Fatal(err)
	}
	during, _ := unix.IoctlGetTermios(fd, ioctlGetTermios)
	if during.Lflag&(unix.ICANON|unix.ECHO) != 0 || during.Lflag&unix.ISIG == 0 || during.Cc[unix.VMIN] != 1 {
		t.Errorf("lflag %#x vmin %d", during.Lflag, during.Cc[unix.VMIN])
	}
	restore()
	after, _ := unix.IoctlGetTermios(fd, ioctlGetTermios)
	if after.Lflag != before.Lflag {
		t.Errorf("not restored: %#x, was %#x", after.Lflag, before.Lflag)
	}
	// Not a terminal: an error, so the keys are read as they come.
	f, err := os.CreateTemp(t.TempDir(), "stdin")
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	if _, err := KeyMode(int(f.Fd())); err == nil {
		t.Error("key mode on a plain file")
	}
}
