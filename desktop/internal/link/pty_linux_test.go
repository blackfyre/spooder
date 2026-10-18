package link

import (
	"os"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// openPty returns a pseudo-terminal's master and the path of a symbolic
// link to its slave, as the virtual robot keeps.
func openPty(t *testing.T) (*os.File, string) {
	t.Helper()
	m, err := os.OpenFile("/dev/ptmx", os.O_RDWR|unix.O_NOCTTY, 0)
	if err != nil {
		t.Skip("no pseudo-terminals:", err)
	}
	t.Cleanup(func() { m.Close() })
	if err := unix.IoctlSetPointerInt(int(m.Fd()), unix.TIOCSPTLCK, 0); err != nil {
		t.Fatal(err)
	}
	n, err := unix.IoctlGetInt(int(m.Fd()), unix.TIOCGPTN)
	if err != nil {
		t.Fatal(err)
	}
	link := filepath.Join(t.TempDir(), "usb")
	if err := os.Symlink("/dev/pts/"+strconv.Itoa(n), link); err != nil {
		t.Fatal(err)
	}
	return m, link
}

func TestSerialOpensAPseudoTerminal(t *testing.T) {
	m, path := openPty(t)
	p, err := Serial{}.Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer p.Close()
	if _, err := m.Write([]byte("hello\n")); err != nil {
		t.Fatal(err)
	}
	if err := p.SetReadTimeout(500 * time.Millisecond); err != nil {
		t.Fatal(err)
	}
	buf := make([]byte, 16)
	n, err := p.Read(buf)
	if err != nil || string(buf[:n]) != "hello\n" {
		t.Fatalf("read %q %v (raw mode expected)", buf[:n], err)
	}
}
