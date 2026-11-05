package main

import (
	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/padmap"
)

// buttonTable is the SDL controller table (design D2): SDL's game-controller
// buttons to the TrimUI's controls. SDL names the face buttons by position
// (Xbox layout); the TrimUI's labels are Nintendo's, so SDL's A (bottom) is
// the TrimUI's B and SDL's X (left) its Y, as gabagool's default mapping has
// them. The input-mapping requirement maps the controls by their labels.
// Confirmed on the device (task 7.1).
var buttonTable = []struct {
	sdl  sdl.GameControllerButton
	hand padmap.Buttons
}{
	{sdl.CONTROLLER_BUTTON_B, padmap.HandA},
	{sdl.CONTROLLER_BUTTON_A, padmap.HandB},
	{sdl.CONTROLLER_BUTTON_Y, padmap.HandX},
	{sdl.CONTROLLER_BUTTON_X, padmap.HandY},
	{sdl.CONTROLLER_BUTTON_LEFTSHOULDER, padmap.HandL1},
	{sdl.CONTROLLER_BUTTON_RIGHTSHOULDER, padmap.HandR1},
	{sdl.CONTROLLER_BUTTON_DPAD_UP, padmap.HandUp},
	{sdl.CONTROLLER_BUTTON_DPAD_DOWN, padmap.HandDown},
	{sdl.CONTROLLER_BUTTON_DPAD_LEFT, padmap.HandLeft},
	{sdl.CONTROLLER_BUTTON_DPAD_RIGHT, padmap.HandRight},
	{sdl.CONTROLLER_BUTTON_START, padmap.HandStart},
	{sdl.CONTROLLER_BUTTON_BACK, padmap.HandSelect},
}

// menuButton is the TrimUI's Menu button (gabagool's default mapping). It
// never reaches the mapping: a press starts a leave (design D5).
const menuButton = sdl.CONTROLLER_BUTTON_GUIDE

// pad is the first SDL game controller, reopened when one is added or
// removed. Without one the controls read at rest.
type pad struct {
	gc *sdl.GameController
}

func newPad() *pad {
	p := &pad{}
	p.open()
	return p
}

func (p *pad) open() {
	p.close()
	for i := 0; i < sdl.NumJoysticks(); i++ {
		if sdl.IsGameController(i) {
			if gc := sdl.GameControllerOpen(i); gc != nil {
				p.gc = gc
				return
			}
		}
	}
}

func (p *pad) close() {
	if p.gc != nil {
		p.gc.Close()
		p.gc = nil
	}
}

// event handles one SDL event and tells whether it asks to leave: Menu
// pressed, the window closed, or Escape on a desktop keyboard.
func (p *pad) event(ev sdl.Event) bool {
	switch e := ev.(type) {
	case *sdl.QuitEvent:
		return true
	case *sdl.ControllerDeviceEvent:
		if e.Type == sdl.CONTROLLERDEVICEADDED || e.Type == sdl.CONTROLLERDEVICEREMOVED {
			p.open()
		}
	case *sdl.ControllerButtonEvent:
		return e.Type == sdl.CONTROLLERBUTTONDOWN && sdl.GameControllerButton(e.Button) == menuButton
	case *sdl.KeyboardEvent:
		return e.Type == sdl.KEYDOWN && e.Repeat == 0 && e.Keysym.Sym == sdl.K_ESCAPE
	}
	return false
}

// controls reads the controller's state now. SDL keeps it current as the
// events are pumped, gabagool's event filter (which drops unmapped axis
// events from the queue) notwithstanding.
func (p *pad) controls() padmap.Controls {
	if p.gc == nil || !p.gc.Attached() {
		return padmap.Controls{}
	}
	c := padmap.Controls{
		LeftX:    p.gc.Axis(sdl.CONTROLLER_AXIS_LEFTX),
		LeftY:    p.gc.Axis(sdl.CONTROLLER_AXIS_LEFTY),
		RightX:   p.gc.Axis(sdl.CONTROLLER_AXIS_RIGHTX),
		RightY:   p.gc.Axis(sdl.CONTROLLER_AXIS_RIGHTY),
		TriggerL: p.gc.Axis(sdl.CONTROLLER_AXIS_TRIGGERLEFT),
		TriggerR: p.gc.Axis(sdl.CONTROLLER_AXIS_TRIGGERRIGHT),
	}
	for _, b := range buttonTable {
		if p.gc.Button(b.sdl) != 0 {
			c.Buttons |= b.hand
		}
	}
	return c
}
