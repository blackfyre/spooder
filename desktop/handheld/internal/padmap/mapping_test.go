package padmap

import "testing"

// Every cell of the mode transition table (design D2 and D3 of
// explicit-state-core), with the counter step of each.
func TestModeTransitionTable(t *testing.T) {
	const (
		stand  = ModeStateStand
		walk   = ModeStateWalk
		sStand = ModeStateSleepFromStand
		sWalk  = ModeStateSleepFromWalk
		cal    = ModeStateCalibrate
	)
	type cell struct {
		next ModeState
		step uint8
	}
	events := []ModeEvent{ModeEventConnect, ModeEventHome, ModeEventSleep, ModeEventCalibrateHold, ModeEventSleepRequest}
	table := map[ModeState][]cell{
		stand:  {{stand, 0}, {walk, 1}, {sStand, 1}, {cal, 1}, {sStand, 1}},
		walk:   {{stand, 0}, {stand, 1}, {sWalk, 1}, {cal, 1}, {sStand, 1}},
		sStand: {{stand, 0}, {sWalk, 1}, {stand, 1}, {cal, 1}, {sStand, 0}},
		sWalk:  {{stand, 0}, {sStand, 1}, {walk, 1}, {cal, 1}, {sStand, 0}},
		cal:    {{stand, 0}, {stand, 1}, {sStand, 1}, {stand, 1}, {sStand, 1}},
	}
	if len(table) != 5 {
		t.Fatalf("table has %d states, want 5", len(table))
	}
	for s, row := range table {
		for i, want := range row {
			m := NewMapper()
			m.modeState = s
			m.modeRequestCounter = 255 // the step wraps, as the counter does
			m.applyModeEvent(events[i])
			if m.modeState != want.next || m.modeRequestCounter != 255+want.step {
				t.Errorf("state %d event %d: state %d counter %d, want state %d counter %d", s, events[i],
					m.modeState, m.modeRequestCounter, want.next, uint8(255+want.step))
			}
		}
	}
}

func TestRequestedModeOfEachState(t *testing.T) {
	for s, want := range map[ModeState]RequestedMode{
		ModeStateStand:          ModeStand,
		ModeStateWalk:           ModeWalk,
		ModeStateSleepFromStand: ModeSleep,
		ModeStateSleepFromWalk:  ModeSleep,
		ModeStateCalibrate:      ModeCalibrate,
	} {
		if got := requestedMode(s); got != want {
			t.Errorf("state %d: mode %d, want %d", s, got, want)
		}
	}
}

// Scenario "Calibrate hold completes with y pressed" (gamepad-control).
func TestCalibrateHoldCompletesWithYPressed(t *testing.T) {
	m := NewMapper()
	now := uint32(1000)
	pad := PadState{Connected: true}
	d := m.Update(pad, now)
	before := d.ModeRequestCounter
	if d.Mode != ModeStand {
		t.Fatalf("start: mode %d, want Stand", d.Mode)
	}
	step := func() {
		now += 20
		d = m.Update(pad, now)
	}
	pad.Misc = MiscHome | MiscBack
	step() // the hold starts
	for i := 1; i < ComboHoldMs/20; i++ {
		step()
	}
	if d.Mode != ModeStand {
		t.Fatalf("before 1 s: mode %d, want Stand", d.Mode)
	}
	pad.Buttons = ButtonY
	step() // the hold reaches 1 s as y is pressed
	if d.Mode != ModeSleep || d.ModeRequestCounter != before+2 {
		t.Fatalf("hold with y: mode %d counter %d, want Sleep, %d", d.Mode, d.ModeRequestCounter, before+2)
	}
	pad.Misc, pad.Buttons = 0, 0
	step()
	pad.Buttons = ButtonY
	step()
	pad.Buttons = 0
	step()
	if d.Mode != ModeStand || d.ModeRequestCounter != before+3 {
		t.Fatalf("further y: mode %d counter %d, want Stand, %d", d.Mode, d.ModeRequestCounter, before+3)
	}
}
