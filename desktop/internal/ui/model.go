package ui

import (
	"fmt"
	"strings"
	"time"

	"gioui.org/io/key"

	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

// selection is the joint editor's keyboard cursor: a joint and one of its
// fields (design D5: arrow keys select a joint and field).
type selection struct {
	joint int
	field calib.Field
}

// move applies an arrow key: left and right step through the joints in
// output order (wrapping), up and down through the fields of the joint.
func (s selection) move(name key.Name) selection {
	switch name {
	case key.NameLeftArrow:
		s.joint = (s.joint + calib.Joints - 1) % calib.Joints
	case key.NameRightArrow:
		s.joint = (s.joint + 1) % calib.Joints
	case key.NameUpArrow:
		s.field = (s.field + calib.FieldCount - 1) % calib.FieldCount
	case key.NameDownArrow:
		s.field = (s.field + 1) % calib.FieldCount
	}
	return s
}

// keyLatch turns a key's presses into one action per physical press: the
// system's key repeat sends further presses while the key is held, which
// must not toggle the e-stop back. The latch opens on the key's release,
// and on losing the window focus, when a release may never arrive.
type keyLatch struct {
	down bool
}

// press records a key event and reports whether it is a new press.
func (k *keyLatch) press(state key.State) bool {
	if state == key.Release {
		k.down = false
		return false
	}
	if k.down {
		return false // key repeat
	}
	k.down = true
	return true
}

// reset forgets a held key, when the window loses the focus.
func (k *keyLatch) reset() { k.down = false }

// nudgeKey maps a key to a nudge (design D5): + and - nudge fine, with
// Shift coarse; R toggles the direction. The + key is matched by its
// unshifted name "=" (US layouts) or "+" (layouts where + is unshifted),
// and coarse is Shift on the same physical key, so Shift+= arrives as "+"
// with Shift, and Shift+- as "_".
func nudgeKey(name key.Name, mods key.Modifiers) (field *calib.Field, steps int, coarse bool, ok bool) {
	shift := mods.Contain(key.ModShift)
	switch name {
	case "=", "+":
		return nil, 1, shift, true
	case "*": // Shift and + on layouts where + is unshifted
		return nil, 1, true, true
	case "-":
		return nil, -1, shift, true
	case "_":
		return nil, -1, true, true
	case "R":
		f := calib.FieldDirection
		return &f, 1, false, true
	}
	return nil, 0, false, false
}

// Value formatting. Every number is the robot's report; units are fixed.

func degreesText(centideg int16) string {
	v := int(centideg)
	sign := ""
	if v < 0 {
		sign, v = "-", -v
	}
	return fmt.Sprintf("%s%d.%02d°", sign, v/100, v%100)
}

func pulseText(us uint16) string { return fmt.Sprintf("%d µs", us) }

func directionText(reversed bool) string {
	if reversed {
		return "reversed"
	}
	return "normal"
}

// coxaCentreCentideg is the stored coxa offset that puts the IK's 0° (the foot
// straight out from the mount) at the servo's mid-travel. Coxa offsets are
// shown from it; the table, the link and the export keep the stored value.
const coxaCentreCentideg = 9000

func isCoxa(joint int) bool { return joint%3 == 0 }

// signedDegreesText is degreesText with a plus sign on positive values.
func signedDegreesText(centideg int) string {
	if centideg > 0 {
		return "+" + degreesText(int16(centideg))
	}
	return degreesText(int16(centideg))
}

// fieldText is a field's value for joint joint (0–17, output order).
func fieldText(joint int, j calib.Joint, f calib.Field) string {
	switch f {
	case calib.FieldOffset:
		if isCoxa(joint) {
			return signedDegreesText(int(j.OffsetCentideg) - coxaCentreCentideg)
		}
		return degreesText(j.OffsetCentideg)
	case calib.FieldPulse0:
		return pulseText(j.Pulse0Us)
	case calib.FieldPulse180:
		return pulseText(j.Pulse180Us)
	}
	return directionText(j.Reversed)
}

var fieldLabels = [calib.FieldCount]string{"offset", "0° pulse", "180° pulse", "direction"}

// fieldLabel is a field's label for joint joint.
func fieldLabel(joint int, f calib.Field) string {
	if f == calib.FieldOffset && isCoxa(joint) {
		return "offset from centre"
	}
	return fieldLabels[f]
}

// fieldDiffers reports whether a field of two joint values differs.
func fieldDiffers(a, b calib.Joint, f calib.Field) bool {
	return fieldText(1, a, f) != fieldText(1, b, f)
}

func voltsText(mv uint16) string { return fmt.Sprintf("%d.%02d V", mv/1000, (mv%1000)/10) }
func ampsText(ma uint16) string  { return fmt.Sprintf("%d.%02d A", ma/1000, (ma%1000)/10) }
func onOffText(on bool) string {
	if on {
		return "on"
	}
	return "off"
}

// ageText is how old a value is, for the "not current" marker.
func ageText(d time.Duration) string {
	switch {
	case d < time.Second:
		return fmt.Sprintf("%d ms", d.Milliseconds())
	case d < time.Minute:
		return fmt.Sprintf("%.1f s", d.Seconds())
	}
	return d.Truncate(time.Second).String()
}

// connectionText describes the link state for the header.
func connectionText(s link.State) string {
	switch s.Status {
	case link.Connected:
		return "connected " + s.Port.Name
	case link.Waiting:
		if s.Err != nil && link.IsPermission(s.Err) {
			return "no access to " + s.Port.Name + ": add yourself to the dialout group, then log in again"
		}
		if s.Err != nil && link.IsBusy(s.Err) {
			return s.Port.Name + " is in use by another program"
		}
		return "waiting for " + s.Port.Label()
	}
	return "disconnected"
}

// sourceText describes who controls the robot, from telemetry.
func sourceText(snap session.Snapshot) string {
	switch snap.Source {
	case session.SourceConflict:
		return "conflict: bridge and USB both claim control"
	case session.SourceUSB:
		if snap.Control {
			return "USB (this app)"
		}
		return "USB"
	case session.SourceBridge:
		return "bridge"
	}
	return "none"
}

// faultsText lists the latched faults by name.
func faultsText(faults []string) string {
	if len(faults) == 0 {
		return "none"
	}
	return strings.Join(faults, ", ")
}

// resultText names a joint's last confirmed result, or "".
func resultText(r calib.Result) string {
	switch r {
	case calib.ResultClamped:
		return "clamped"
	case calib.ResultRejected:
		return "rejected"
	}
	return ""
}

// editHint explains why the editor is not live.
func editHint(snap session.Snapshot) string {
	switch {
	case snap.Link.Status != link.Connected:
		return "Connect to the robot to edit its calibration."
	case !snap.Control:
		return "Observing. Take control and request Calibrate to edit."
	case !snap.Fresh:
		return "Telemetry is not current."
	case snap.Body != session.BodyCalibrate:
		return "The robot is in " + snap.Body.String() + ". Request Calibrate to edit."
	case !snap.HaveReport:
		return "Waiting for the robot's calibration report."
	}
	return ""
}

// bridgeText is the bridge link as the robot reports it: its state (down,
// or up with or without a gamepad) and its frame counts, shown in every
// state, as a stalled or failing link is when they matter.
func bridgeText(t protocol.Telemetry) (state, counts string) {
	state = "down"
	if t.Flags&protocol.TelemetryBridgeLinkUp != 0 {
		state = "up · no pad"
		if t.Flags&protocol.TelemetryBridgeGamepad != 0 {
			state = "up · pad"
		}
	}
	return state, fmt.Sprintf("%d ok · %d bad", t.BridgeValidFrames, t.BridgeDiscardedFrames)
}

func bridgeState(t protocol.Telemetry) string  { s, _ := bridgeText(t); return s }
func bridgeCounts(t protocol.Telemetry) string { _, c := bridgeText(t); return c }
