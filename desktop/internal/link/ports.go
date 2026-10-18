package link

import (
	"errors"
	"io"
	"io/fs"
	"os"
	"strings"
	"time"

	"go.bug.st/serial"
	"go.bug.st/serial/enumerator"
)

// The Servo 2040's USB ID: the RP2040's pico-sdk stdio USB device.
const (
	Servo2040VID = "2e8a"
	Servo2040PID = "000a"
)

// BaudRate is nominal on USB CDC. It must never be 1200: a 1200 baud
// "touch" reboots an RP2040 into its bootloader.
const BaudRate = 115200

// PortInfo describes one serial port.
type PortInfo struct {
	Name         string
	IsUSB        bool
	VID, PID     string
	SerialNumber string
	Product      string
}

// IsServo2040 reports whether the port is an RP2040 stdio USB device.
func (p PortInfo) IsServo2040() bool {
	return p.IsUSB && strings.EqualFold(p.VID, Servo2040VID) && strings.EqualFold(p.PID, Servo2040PID)
}

// Label is a short description for a port picker.
func (p PortInfo) Label() string {
	s := p.Name
	if p.IsUSB {
		s += " (" + strings.ToLower(p.VID) + ":" + strings.ToLower(p.PID)
		if p.IsServo2040() {
			s += " Servo 2040"
		} else if p.Product != "" {
			s += " " + p.Product
		}
		s += ")"
	}
	return s
}

// SameDevice reports whether b is the device a was: the same USB serial
// number when a has one, otherwise the same port name.
func SameDevice(a, b PortInfo) bool {
	if a.SerialNumber != "" {
		return b.SerialNumber == a.SerialNumber && strings.EqualFold(a.VID, b.VID) && strings.EqualFold(a.PID, b.PID)
	}
	return a.Name == b.Name
}

// Preselect returns the index of the first Servo 2040 in ports, or -1.
func Preselect(ports []PortInfo) int {
	for i, p := range ports {
		if p.IsServo2040() {
			return i
		}
	}
	return -1
}

// Port is an open serial port. Read returns 0 bytes and no error when the
// read timeout passes without data.
type Port interface {
	io.ReadWriteCloser
	SetReadTimeout(t time.Duration) error
}

// System lists and opens serial ports.
type System interface {
	List() ([]PortInfo, error)
	Open(name string) (Port, error)
}

// Serial is the computer's serial ports, through go.bug.st/serial.
type Serial struct{}

// List returns the serial ports with their USB details.
func (Serial) List() ([]PortInfo, error) {
	details, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, err
	}
	out := make([]PortInfo, 0, len(details))
	for _, d := range details {
		out = append(out, PortInfo{Name: d.Name, IsUSB: d.IsUSB, VID: d.VID, PID: d.PID, SerialNumber: d.SerialNumber, Product: d.Product})
	}
	return out, nil
}

// Open opens a port at BaudRate with DTR raised, which the robot's USB stack
// takes as a host being connected. A port without modem control lines (a
// pseudo-terminal, such as the virtual robot's) is opened without them.
func (Serial) Open(name string) (Port, error) {
	p, err := serial.Open(name, &serial.Mode{
		BaudRate:          BaudRate,
		InitialStatusBits: &serial.ModemOutputBits{DTR: true, RTS: true},
	})
	if err != nil && isNoModemLines(err) {
		return serial.Open(name, &serial.Mode{BaudRate: BaudRate})
	}
	return p, err
}

// isNoModemLines reports whether opening failed only for want of modem
// control lines (go.bug.st/serial's TIOCMGET or TIOCMSET refused).
func isNoModemLines(err error) bool {
	var pe *serial.PortError
	return errors.As(err, &pe) && pe.Code() == serial.InvalidSerialPort && strings.Contains(err.Error(), "modem bits")
}

// WithPort returns sys with the port at path added: listed whenever the path
// exists, even if the system's enumeration does not report it (a
// pseudo-terminal, or a symbolic link to one, as the virtual robot keeps).
// Its PortInfo has only a Name, so reconnection follows the path. An empty
// path returns sys unchanged.
func WithPort(sys System, path string) System {
	if path == "" {
		return sys
	}
	return explicitPort{System: sys, path: path}
}

type explicitPort struct {
	System
	path string
}

func (e explicitPort) List() ([]PortInfo, error) {
	ports, err := e.System.List()
	if err != nil {
		ports = nil // the explicit port does not depend on the enumeration
	}
	for _, p := range ports {
		if p.Name == e.path {
			return ports, nil
		}
	}
	if _, err := os.Stat(e.path); err == nil {
		ports = append(ports, PortInfo{Name: e.path})
	}
	return ports, nil
}

// IsPermission reports whether opening a port failed for lack of access
// (on Linux, membership of the dialout group).
func IsPermission(err error) bool {
	var pe *serial.PortError
	if errors.As(err, &pe) && pe.Code() == serial.PermissionDenied {
		return true
	}
	return errors.Is(err, fs.ErrPermission)
}

// IsBusy reports whether the port is already open elsewhere.
func IsBusy(err error) bool {
	var pe *serial.PortError
	return errors.As(err, &pe) && pe.Code() == serial.PortBusy
}
