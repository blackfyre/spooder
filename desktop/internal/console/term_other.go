//go:build !linux && !darwin

package console

import "errors"

// KeyMode is not supported here: keys are forwarded as standard input
// delivers them (design D5).
func KeyMode(fd int) (restore func(), err error) {
	return nil, errors.New("key mode is not supported on this system")
}
