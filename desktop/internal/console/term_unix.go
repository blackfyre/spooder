//go:build linux || darwin

package console

import "golang.org/x/sys/unix"

// KeyMode makes a terminal deliver keys one at a time without echo, keeping
// signals so Ctrl+C still interrupts (design D5). It returns the function
// that restores the terminal, or an error when fd is not a terminal.
func KeyMode(fd int) (restore func(), err error) {
	t, err := unix.IoctlGetTermios(fd, ioctlGetTermios)
	if err != nil {
		return nil, err
	}
	old := *t
	t.Lflag &^= unix.ICANON | unix.ECHO
	t.Lflag |= unix.ISIG
	t.Cc[unix.VMIN] = 1
	t.Cc[unix.VTIME] = 0
	if err := unix.IoctlSetTermios(fd, ioctlSetTermios, t); err != nil {
		return nil, err
	}
	return func() { _ = unix.IoctlSetTermios(fd, ioctlSetTermios, &old) }, nil
}
