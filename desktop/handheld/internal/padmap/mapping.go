// Package padmap is the handheld's input mapping: a line-for-line port of
// core/gamepad_mapping (mapping.go), kept in step with the C++ by
// test/golden/gamepad_mapping.txt, and the handheld layer (handheld.go) that
// turns the TrimUI's controls into the gamepad's PadState before the port
// sees them (design D2 of add-handheld-controller).
package padmap

import (
	"math"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// PadState.Buttons bits (Bluepad32 names), as gamepad_mapping::BUTTON_*.
const (
	ButtonA      uint16 = 1 << 0
	ButtonB      uint16 = 1 << 1
	ButtonX      uint16 = 1 << 2
	ButtonY      uint16 = 1 << 3
	ButtonL1     uint16 = 1 << 4
	ButtonR1     uint16 = 1 << 5
	ButtonL2     uint16 = 1 << 6 // Bluepad32 BUTTON_TRIGGER_L
	ButtonR2     uint16 = 1 << 7 // Bluepad32 BUTTON_TRIGGER_R
	ButtonThumbL uint16 = 1 << 8
	ButtonThumbR uint16 = 1 << 9
)

// PadState.Dpad bits.
const (
	DpadUp    uint8 = 1 << 0
	DpadDown  uint8 = 1 << 1
	DpadRight uint8 = 1 << 2
	DpadLeft  uint8 = 1 << 3
)

// PadState.Misc bits.
const (
	MiscHome uint8 = 1 << 0
	MiscBack uint8 = 1 << 1
)

const (
	AxisMax          = 512  // Bluepad32 axes span -511..512
	DeadZone         = 0.08 // radial, fraction of full scale (float32 in use)
	ComboHoldMs      = 1000
	DialStep         = 10
	ReverseThreshold = 10
	// GaitCount is gait::GAIT_COUNT (core/movement/gait.h).
	GaitCount = 6
	// StartGait is START_GAIT (core/gamepad_mapping): Wave.
	StartGait = 1
	// StartSpeed is START_SPEED (core/movement/movement_command.h).
	StartSpeed = 44
)

// RequestedMode is RequestedMode in core/movement/movement_command.h.
type RequestedMode uint8

const (
	ModeStand RequestedMode = iota
	ModeWalk
	ModeSleep
	ModeCalibrate
)

// ModeState is gamepad_mapping::ModeState: the mapping's mode request, the
// reachable combinations of a base mode (Stand or Walk) with the Sleep and
// Calibrate toggles. Sleep keeps the mode it returns to; leaving Calibrate
// returns to Stand.
type ModeState uint8

const (
	ModeStateStand ModeState = iota
	ModeStateWalk
	ModeStateSleepFromStand
	ModeStateSleepFromWalk
	ModeStateCalibrate
)

// ModeEvent is gamepad_mapping::ModeEvent, plus the handheld's sleep request
// (ModeEventSleepRequest), which only the Go port has.
type ModeEvent uint8

const (
	ModeEventConnect ModeEvent = iota
	ModeEventHome
	ModeEventSleep
	ModeEventCalibrateHold
	// ModeEventSleepRequest is Mapper.RequestSleep: Sleep returning to
	// Stand, from any state.
	ModeEventSleepRequest
)

// StrideMode is StrideMode in core/movement/movement_command.h.
type StrideMode uint8

const (
	StrideDynamic StrideMode = iota
	StrideFixed
)

// StickMode is gamepad_mapping::StickMode.
type StickMode uint8

const (
	StickSteer StickMode = iota
	StickCrab
)

// PadState is one pad sample, as gamepad_mapping::PadState.
type PadState struct {
	Connected      bool
	AxisX, AxisY   int32 // left stick; Bluepad32 Y is negative for up
	AxisRX, AxisRY int32 // right stick
	Buttons        uint16
	Dpad           uint8
	Misc           uint8
}

// Desired is the desired movement state, as gamepad_mapping::Desired.
type Desired struct {
	Mode                   RequestedMode
	TranslateX, TranslateY int8 // -100..100, + Y = forward
	Rotate                 int8 // -100..100
	Height, Speed          uint8
	Gait                   uint8
	Stride                 StrideMode
	StickMode              StickMode
	ControllerConnected    bool
	ClearCounter           uint8
	ModeRequestCounter     uint8
	Gesture                uint8
	GestureRequestCounter  uint8
}

// Mapper holds the toggles, dials, counters and input history across
// samples, as gamepad_mapping::Mapper. NewMapper gives its initial values.
type Mapper struct {
	height                uint8
	speed                 uint8
	gait                  uint8
	stride                StrideMode
	stickMode             StickMode
	clearCounter          uint8
	modeRequestCounter    uint8
	gesture               uint8
	gestureRequestCounter uint8

	// Changed only through applyModeEvent.
	modeState ModeState

	wasConnected     bool
	prevButtons      uint16
	prevDpad         uint8
	prevMisc         uint8
	calibrateHeld    bool
	calibrateFired   bool
	calibrateSinceMs uint32
	clearHeld        bool
	clearFired       bool
	clearSinceMs     uint32
}

// NewMapper returns a Mapper with the C++ member initialisers.
func NewMapper() Mapper {
	return Mapper{speed: StartSpeed, gait: StartGait, stride: StrideDynamic, stickMode: StickSteer, modeState: ModeStateStand}
}

// sqrtf is C's sqrtf: the float64 square root of a float32 rounds to the
// correctly rounded float32 result.
func sqrtf(x float32) float32 { return float32(math.Sqrt(float64(x))) }

// roundf is C's roundf (half away from zero), exact for float32 inputs.
func roundf(x float32) float32 { return float32(math.Round(float64(x))) }

func toPercent(v float32) int8 {
	p := roundf(v * 100)
	if p > 100 {
		p = 100
	}
	if p < -100 {
		p = -100
	}
	return int8(p)
}

// StickToVector converts a stick pair (Bluepad32 units, Y already pointing
// up) to -100..100 per axis with the radial dead zone and magnitude clamp,
// as gamepad_mapping::stickToVector. Every float operation is float32, as in
// the C++; the explicit float32 conversions of the products keep the
// compiler from fusing them into a multiply-add (Go fuses on arm64, the
// device), which would round differently from the C++.
func StickToVector(x, y int32) (outX, outY int8) {
	dz := float32(DeadZone)
	fx := float32(x) / AxisMax
	fy := float32(y) / AxisMax
	m := sqrtf(float32(fx*fx) + float32(fy*fy))
	if m <= dz {
		return 0, 0
	}
	// Rescaled so the output rises from zero at the dead-zone edge, then
	// clamped to full scale.
	scaled := (m - dz) / (1 - dz)
	if scaled > 1 {
		scaled = 1
	}
	k := scaled / m
	ix := int(toPercent(fx * k))
	iy := int(toPercent(fy * k))
	// Rounding each axis can push a full diagonal just past 100; pull the
	// larger component back until the magnitude is within range.
	for ix*ix+iy*iy > 100*100 {
		big := &iy
		if abs(ix) >= abs(iy) {
			big = &ix
		}
		if *big > 0 {
			*big--
		} else {
			*big++
		}
	}
	return int8(ix), int8(iy)
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func stepDial(value uint8, delta int) uint8 {
	v := int(value) + delta
	if v < 0 {
		v = 0
	}
	if v > 100 {
		v = 100
	}
	return uint8(v)
}

// comboFired tracks a two-button hold; it returns true once per hold when it
// reaches the hold time.
func comboFired(down bool, held, fired *bool, sinceMs *uint32, nowMs uint32) bool {
	if !down {
		*held = false
		*fired = false
		return false
	}
	if !*held {
		*held = true
		*sinceMs = nowMs
	}
	if !*fired && nowMs-*sinceMs >= ComboHoldMs {
		*fired = true
		return true
	}
	return false
}

// next is gamepad_mapping::next, the mapping's one transition function,
// with the sleep request added.
func next(s ModeState, e ModeEvent) ModeState {
	switch e {
	case ModeEventConnect:
		return ModeStateStand
	case ModeEventHome:
		// Toggles Stand and Walk, also the mode Sleep returns to; leaves
		// Calibrate for Stand.
		switch s {
		case ModeStateStand:
			return ModeStateWalk
		case ModeStateWalk, ModeStateCalibrate:
			return ModeStateStand
		case ModeStateSleepFromStand:
			return ModeStateSleepFromWalk
		case ModeStateSleepFromWalk:
			return ModeStateSleepFromStand
		}
	case ModeEventSleep:
		// Toggles Sleep; Sleep from Calibrate returns to Stand.
		switch s {
		case ModeStateStand, ModeStateCalibrate:
			return ModeStateSleepFromStand
		case ModeStateWalk:
			return ModeStateSleepFromWalk
		case ModeStateSleepFromStand:
			return ModeStateStand
		case ModeStateSleepFromWalk:
			return ModeStateWalk
		}
	case ModeEventCalibrateHold:
		// Toggles Calibrate, also from Sleep; leaving it returns to Stand.
		if s == ModeStateCalibrate {
			return ModeStateStand
		}
		return ModeStateCalibrate
	case ModeEventSleepRequest:
		return ModeStateSleepFromStand
	}
	return s
}

// requestedMode is gamepad_mapping::requestedMode.
func requestedMode(s ModeState) RequestedMode {
	switch s {
	case ModeStateStand:
		return ModeStand
	case ModeStateWalk:
		return ModeWalk
	case ModeStateCalibrate:
		return ModeCalibrate
	}
	return ModeSleep
}

// stepsCounter reports whether event e in state s advances the mode-request
// counter: every event but Connect, and the sleep request only when Sleep is
// not already requested.
func stepsCounter(s ModeState, e ModeEvent) bool {
	switch e {
	case ModeEventConnect:
		return false
	case ModeEventSleepRequest:
		return requestedMode(s) != ModeSleep
	}
	return true
}

// applyModeEvent is gamepad_mapping::applyModeEvent.
func (m *Mapper) applyModeEvent(e ModeEvent) {
	if stepsCounter(m.modeState, e) {
		m.modeRequestCounter++
	}
	m.modeState = next(m.modeState, e)
}

// RequestSleep requests Sleep, advancing the mode-request counter once
// unless Sleep is already requested. The mode Sleep returns to is Stand, as
// at a bridge session's start, so Y then leaves Sleep for Stand even when the
// robot was walking before. Not in the C++ mapping (design D3 of
// explicit-state-core).
func (m *Mapper) RequestSleep() { m.applyModeEvent(ModeEventSleepRequest) }

var gestureButtons = []struct {
	button  uint16
	gesture uint8
}{
	{ButtonA, protocol.GestureSayHi},
	{ButtonB, protocol.GestureWiggle},
	{ButtonX, protocol.GestureDance},
	{ButtonY, protocol.GestureAttack},
	{ButtonR2, protocol.GestureBall},
}

// Update advances the mapping by one pad sample taken at nowMs, as
// gamepad_mapping::update.
func (m *Mapper) Update(pad PadState, nowMs uint32) Desired {
	var d Desired

	if !pad.Connected {
		m.wasConnected = false
	} else {
		if !m.wasConnected {
			// A fresh connection starts in Stand; buttons already held do
			// not count as presses.
			m.wasConnected = true
			m.applyModeEvent(ModeEventConnect)
			m.prevButtons = pad.Buttons
			m.prevDpad = pad.Dpad
			m.prevMisc = pad.Misc
			m.calibrateHeld, m.clearHeld = false, false
			m.calibrateFired, m.clearFired = false, false
		}

		pressed := pad.Buttons &^ m.prevButtons
		dpadPressed := pad.Dpad &^ m.prevDpad
		miscPressed := pad.Misc &^ m.prevMisc
		m.prevButtons = pad.Buttons
		m.prevDpad = pad.Dpad
		m.prevMisc = pad.Misc

		// Calibrate: miscBack + miscHome held. While miscBack is held,
		// miscHome is part of the combination and does not toggle
		// Stand/Walk.
		backHeld := pad.Misc&MiscBack != 0
		calibrateDown := backHeld && pad.Misc&MiscHome != 0
		if comboFired(calibrateDown, &m.calibrateHeld, &m.calibrateFired, &m.calibrateSinceMs, nowMs) {
			m.applyModeEvent(ModeEventCalibrateHold)
		} else if miscPressed&MiscHome != 0 && !backHeld {
			m.applyModeEvent(ModeEventHome)
		}

		// The gesture layer: while L2 is held, a, b, x, y and R2 request
		// gestures instead of their own functions.
		if pad.Buttons&ButtonL2 != 0 {
			for _, g := range gestureButtons {
				if pressed&g.button != 0 {
					m.gesture = g.gesture
					m.gestureRequestCounter++
				}
			}
			pressed &^= ButtonA | ButtonB | ButtonX | ButtonY | ButtonR2
		}

		// After the Calibrate hold in the same sample, so a y press as the
		// hold completes enters Calibrate and leaves it for Sleep.
		if pressed&ButtonY != 0 {
			m.applyModeEvent(ModeEventSleep)
		}

		if pressed&ButtonR1 != 0 {
			m.gait = uint8((int(m.gait) + 1) % GaitCount)
		}
		if pressed&ButtonL1 != 0 {
			m.gait = uint8((int(m.gait) + GaitCount - 1) % GaitCount)
		}
		if dpadPressed&DpadUp != 0 {
			m.height = stepDial(m.height, DialStep)
		}
		if dpadPressed&DpadDown != 0 {
			m.height = stepDial(m.height, -DialStep)
		}
		if dpadPressed&DpadRight != 0 {
			m.speed = stepDial(m.speed, DialStep)
		}
		if dpadPressed&DpadLeft != 0 {
			m.speed = stepDial(m.speed, -DialStep)
		}
		if pressed&ButtonX != 0 {
			if m.stride == StrideDynamic {
				m.stride = StrideFixed
			} else {
				m.stride = StrideDynamic
			}
		}
		if pressed&ButtonB != 0 {
			if m.stickMode == StickSteer {
				m.stickMode = StickCrab
			} else {
				m.stickMode = StickSteer
			}
		}

		clearDown := pad.Buttons&ButtonThumbL != 0 && pad.Buttons&ButtonThumbR != 0
		if comboFired(clearDown, &m.clearHeld, &m.clearFired, &m.clearSinceMs, nowMs) {
			m.clearCounter++
		}

		leftX, leftY := StickToVector(pad.AxisX, -pad.AxisY)
		rightX, _ := StickToVector(pad.AxisRX, 0)
		// Positive rotate turns the body anticlockwise, so a stick pushed
		// right (positive) turns it clockwise with a negative rotate.
		rotate := -int(rightX)
		if m.stickMode == StickSteer {
			// Walk on Y, steer on X. Reversing mirrors the steering, as a
			// car's; a slightly backward stick, within the threshold,
			// neither reverses nor mirrors.
			reversing := leftY < -ReverseThreshold
			d.TranslateX = 0
			if leftY < 0 && !reversing {
				d.TranslateY = 0
			} else {
				d.TranslateY = leftY
			}
			if reversing {
				rotate += int(leftX)
			} else {
				rotate -= int(leftX)
			}
		} else {
			d.TranslateX = leftX
			d.TranslateY = leftY
		}
		d.Rotate = int8(max(-100, min(100, rotate)))
	}

	d.ControllerConnected = pad.Connected
	if pad.Connected {
		d.Mode = requestedMode(m.modeState)
	} else {
		d.Mode = ModeSleep
	}
	d.Height = m.height
	d.Speed = m.speed
	d.Gait = m.gait
	d.Stride = m.stride
	d.StickMode = m.stickMode
	d.ClearCounter = m.clearCounter
	d.ModeRequestCounter = m.modeRequestCounter
	d.Gesture = m.gesture
	d.GestureRequestCounter = m.gestureRequestCounter
	return d
}

// ToCommand is the command frame payload for d, as gamepad_mapping::toCommand.
func ToCommand(d Desired, emergencyStop bool) protocol.Command {
	c := protocol.Command{
		Mode:                  uint8(d.Mode),
		TranslateX:            d.TranslateX,
		TranslateY:            d.TranslateY,
		Rotate:                d.Rotate,
		Height:                d.Height,
		Speed:                 d.Speed,
		Gait:                  d.Gait,
		Stride:                uint8(d.Stride),
		ClearCounter:          d.ClearCounter,
		ModeRequestCounter:    d.ModeRequestCounter,
		Gesture:               d.Gesture,
		GestureRequestCounter: d.GestureRequestCounter,
	}
	if d.ControllerConnected {
		c.Flags |= protocol.FlagControllerConnected
	}
	if emergencyStop {
		c.Flags |= protocol.FlagEmergencyStop
	}
	return c
}
