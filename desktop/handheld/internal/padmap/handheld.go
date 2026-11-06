package padmap

import "github.com/blackfyre/spooder/desktop/internal/protocol"

// Buttons are the TrimUI's digital controls that reach the mapping. Menu and
// Power have no bit: they go to the leave state machine (design D5), never
// here. L2 and R2 are analogue (Controls.TriggerL, TriggerR).
type Buttons uint16

const (
	HandA Buttons = 1 << iota
	HandB
	HandX
	HandY
	HandL1
	HandR1
	HandUp
	HandDown
	HandRight
	HandLeft
	HandStart
	HandSelect
	// HandL2 and HandR2 are the triggers beyond half travel, set by the
	// layer itself from the analogue values, so that they are masked like
	// the buttons.
	HandL2
	HandR2
)

// Controls is one sample of the TrimUI's controls in SDL units.
type Controls struct {
	LeftX, LeftY   int16 // SDL: -32768..32767, Y negative for up
	RightX, RightY int16
	// TriggerL and TriggerR are SDL's trigger axes, 0..32767.
	TriggerL, TriggerR int16
	Buttons            Buttons
}

// TriggerThreshold is half of a trigger's travel; beyond it the trigger
// counts as pressed (Bluepad32's trigger button).
const TriggerThreshold = 16384

// ScaleAxis maps an SDL stick axis (-32768..32767) to Bluepad32 units
// (-511..512), each half scaled on its own so both ends map exactly.
func ScaleAxis(v int16) int32 {
	if v >= 0 {
		return int32(v) * AxisMax / 32767
	}
	return int32(v) * (AxisMax - 1) / 32768
}

// Handheld is the handheld layer in front of the ported mapping (design
// D2): it turns Controls into the gamepad's PadState and keeps the
// handheld's own emergency stop.
//
// It keeps the PadState from showing the mapping a press the user did not
// make: a Start, X, Y or B pressed with Select held is masked until that
// button is released, whatever Select does meanwhile; so is every control held
// when driving begins or when the neutral pad (a leave) ends.
type Handheld struct {
	mapper     Mapper
	estop      bool
	driveView  bool // Select + X: the drive screen's driving-view page
	soundOff   bool // Select + B: the pak's sound feedback off
	neutral    bool
	maskHeld   bool    // mask every control held at the next Step
	prev       Buttons // the controls held at the last Step
	masked     Buttons // held controls not passed on until released
	clearArmed bool    // Select + Y, armed by a Y press with Select held
}

// NewHandheld returns a layer with a fresh mapper.
func NewHandheld() *Handheld {
	return &Handheld{mapper: NewMapper()}
}

// Begin starts driving: it connects the mapper and requests Sleep, as the
// bridge holds it at a session's start, so that Y leaves Sleep for Stand
// and Start does not ask for Walk from Sleep. Controls held at the next
// Step are not taken as presses.
func (h *Handheld) Begin(nowMs uint32) {
	h.mapper.Update(PadState{Connected: true}, nowMs)
	h.RequestSleep()
	h.maskHeld = true
}

// RequestSleep sets the mapper's Sleep toggle, advancing the mode-request
// counter once unless Sleep is already requested. For the start of a run, a
// return from suspend and a leave (design D5). The mode Sleep returns to is
// reset to Stand, as at a bridge session's start, so Y then leaves Sleep for
// Stand even when the robot was walking before.
func (h *Handheld) RequestSleep() {
	m := &h.mapper
	m.baseMode = ModeStand
	if m.requestedMode() == ModeSleep {
		return
	}
	m.calibrating = false
	m.sleeping = true
	m.modeRequestCounter++
}

// SetNeutral makes the mapping see a connected pad at rest (during a leave)
// or, when turned off, the controls again, with those held at that moment
// masked until released.
func (h *Handheld) SetNeutral(on bool) {
	if h.neutral && !on {
		h.maskHeld = true
	}
	h.neutral = on
}

// EmergencyStop is the handheld's e-stop toggle (Select + Start).
func (h *Handheld) EmergencyStop() bool { return h.estop }

// DriveView is the drive screen's page toggle (Select + X): the driving view in
// place of the panels. It never reaches the mapping.
func (h *Handheld) DriveView() bool { return h.driveView }

// SoundOn is the pak's sound feedback toggle (Select + B, design D4 of
// handheld-observations). It never reaches the mapping.
func (h *Handheld) SoundOn() bool { return !h.soundOff }

// SetSoundOn sets the sound toggle (the setting kept from an earlier run).
func (h *Handheld) SetSoundOn(on bool) { h.soundOff = !on }

// Held is the controls held at the last Step, the triggers beyond half
// travel included.
func (h *Handheld) Held() Buttons { return h.prev }

// ClearProgress is how far the Select + Y hold is towards its fault clear
// at nowMs: 0 when no hold is armed, rising to 1 when the clear is sent.
func (h *Handheld) ClearProgress(nowMs uint32) float64 {
	m := &h.mapper
	if !h.clearArmed || !m.clearHeld {
		return 0
	}
	if m.clearFired {
		return 1
	}
	return min(1, float64(nowMs-m.clearSinceMs)/ComboHoldMs)
}

// Command is the command frame payload for d with the handheld's e-stop.
func (h *Handheld) Command(d Desired) protocol.Command { return ToCommand(d, h.estop) }

// Step advances the layer and the mapping by one sample taken at nowMs.
func (h *Handheld) Step(c Controls, nowMs uint32) Desired {
	held := c.Buttons &^ (HandL2 | HandR2)
	if c.TriggerL > TriggerThreshold {
		held |= HandL2
	}
	if c.TriggerR > TriggerThreshold {
		held |= HandR2
	}
	if h.maskHeld {
		h.maskHeld = false
		h.prev = held
		h.masked |= held &^ HandSelect
		h.clearArmed = false
	}
	pressed := held &^ h.prev
	h.prev = held
	h.masked &= held // a released control is no longer masked
	selectHeld := held&HandSelect != 0

	if selectHeld && pressed&HandStart != 0 {
		h.estop = !h.estop
		h.masked |= HandStart
	}
	if selectHeld && pressed&HandX != 0 {
		h.driveView = !h.driveView
		h.masked |= HandX
	}
	if selectHeld && pressed&HandB != 0 {
		h.soundOff = !h.soundOff
		h.masked |= HandB
	}
	if selectHeld && pressed&HandY != 0 {
		h.masked |= HandY
		h.clearArmed = true
	}
	if !selectHeld || held&HandY == 0 {
		h.clearArmed = false
	}

	pad := PadState{Connected: true}
	if !h.neutral {
		pass := held &^ h.masked
		pad.AxisX, pad.AxisY = ScaleAxis(c.LeftX), ScaleAxis(c.LeftY)
		pad.AxisRX, pad.AxisRY = ScaleAxis(c.RightX), ScaleAxis(c.RightY)
		for _, b := range buttonMap {
			if pass&b.hand != 0 {
				pad.Buttons |= b.pad
			}
		}
		for _, b := range dpadMap {
			if pass&b.hand != 0 {
				pad.Dpad |= b.pad
			}
		}
		if pass&HandStart != 0 {
			pad.Misc |= MiscHome
		}
		if h.clearArmed {
			pad.Buttons |= ButtonThumbL | ButtonThumbR
		}
	}
	return h.mapper.Update(pad, nowMs)
}

var buttonMap = []struct {
	hand Buttons
	pad  uint16
}{
	{HandA, ButtonA}, {HandB, ButtonB}, {HandX, ButtonX}, {HandY, ButtonY},
	{HandL1, ButtonL1}, {HandR1, ButtonR1}, {HandL2, ButtonL2}, {HandR2, ButtonR2},
}

var dpadMap = []struct {
	hand Buttons
	pad  uint8
}{
	{HandUp, DpadUp}, {HandDown, DpadDown}, {HandRight, DpadRight}, {HandLeft, DpadLeft},
}
