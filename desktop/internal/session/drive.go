package session

import "github.com/blackfyre/spooder/desktop/internal/protocol"

// Drive is the set of held drive keys. The command frame maps it as the
// bridge's car layout maps the left stick: walk forward and back, steer
// (turning on the spot without walking, mirrored when reversing, as a car's
// steering), and step sideways.
type Drive struct {
	Forward, Back         bool
	SteerLeft, SteerRight bool
	StepLeft, StepRight   bool
}

// pressedSince reports whether d holds a key that before did not.
func (d Drive) pressedSince(before Drive) bool {
	return (d.Forward && !before.Forward) || (d.Back && !before.Back) ||
		(d.SteerLeft && !before.SteerLeft) || (d.SteerRight && !before.SteerRight) ||
		(d.StepLeft && !before.StepLeft) || (d.StepRight && !before.StepRight)
}

// Walking settings: the bridge's starting values and dial step.
const (
	dialStep      = 10
	dialMax       = 100
	strideDynamic = 0
	strideFixed   = 1
	gaitCount     = 6 // gait::GAIT_COUNT
)

func axis(minus, plus bool) int {
	v := 0
	if plus {
		v++
	}
	if minus {
		v--
	}
	return v
}

// driveInput maps the held keys to translate and rotate.
func driveInput(d Drive) (translateX, translateY, rotate int8) {
	walk := axis(d.Back, d.Forward)
	steer := axis(d.SteerLeft, d.SteerRight)
	// Positive rotate turns anticlockwise, so steering right is negative.
	r := -steer
	if walk < 0 {
		r = -r // reversing mirrors the steering
	}
	return int8(100 * axis(d.StepLeft, d.StepRight)), int8(100 * walk), int8(100 * r)
}

// SetDrive replaces the held drive keys. Only in control; a key newly
// pressed while Stand is requested requests Walk once (a key still held
// from before Stand, or a release, never does). In Sleep and Calibrate the
// keys have no effect.
func (s *Session) SetDrive(d Drive) error {
	return s.update(func() error {
		if !s.control || !s.connected() {
			s.drive = Drive{}
			return ErrNotInControl
		}
		pressed := d.pressedSince(s.drive)
		s.drive = d
		if pressed && s.mode == RequestStand {
			s.mode = RequestWalk
			s.modeCounter++
		}
		return nil
	})
}

func stepDial(v uint8, delta int) uint8 {
	n := int(v) + delta
	if n < 0 {
		n = 0
	}
	if n > dialMax {
		n = dialMax
	}
	return uint8(n)
}

// setting changes a walking setting; only in control.
func (s *Session) setting(f func()) error {
	return s.update(func() error {
		if !s.control || !s.connected() {
			return ErrNotInControl
		}
		f()
		return nil
	})
}

// StepSpeed moves the speed dial by steps of 10, within 0-100.
func (s *Session) StepSpeed(steps int) error {
	return s.setting(func() { s.speed = stepDial(s.speed, steps*dialStep) })
}

// StepHeight moves the body height by steps of 10, within 0-100.
func (s *Session) StepHeight(steps int) error {
	return s.setting(func() { s.height = stepDial(s.height, steps*dialStep) })
}

// NextGait selects the next gait, after Hop Tri again.
func (s *Session) NextGait() error {
	return s.setting(func() { s.gait = (s.gait + 1) % gaitCount })
}

// ToggleStride switches between the dynamic and the fixed stride.
func (s *Session) ToggleStride() error {
	return s.setting(func() {
		if s.stride == strideFixed {
			s.stride = strideDynamic
		} else {
			s.stride = strideFixed
		}
	})
}

// drives fills the walking fields of c; s.mu is held and the app is in
// control.
func (s *Session) drives(c *protocol.Command) {
	c.Height, c.Speed, c.Gait, c.Stride = s.height, s.speed, s.gait, s.stride
	if s.mode == RequestWalk {
		c.TranslateX, c.TranslateY, c.Rotate = driveInput(s.drive)
	}
}

// StrideName names a stride mode value.
func StrideName(v uint8) string {
	if v == strideFixed {
		return "fixed"
	}
	return "dynamic"
}
