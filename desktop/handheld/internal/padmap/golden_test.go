package padmap

import (
	"bufio"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// goldenPath is test/golden/gamepad_mapping.txt, which
// test/test_gamepad_mapping_golden runs the C++ mapping over.
// SPOODER_GAMEPAD_GOLDEN names another copy, as for the C++ test.
func goldenPath() string {
	if p := os.Getenv("SPOODER_GAMEPAD_GOLDEN"); p != "" {
		return p
	}
	return filepath.Join("..", "..", "..", "..", "test", "golden", "gamepad_mapping.txt")
}

var (
	goldenButtons = map[string]uint16{"A": ButtonA, "B": ButtonB, "X": ButtonX, "Y": ButtonY, "L1": ButtonL1,
		"R1": ButtonR1, "L2": ButtonL2, "R2": ButtonR2, "THUMB_L": ButtonThumbL, "THUMB_R": ButtonThumbR}
	goldenDpad   = map[string]uint8{"UP": DpadUp, "DOWN": DpadDown, "RIGHT": DpadRight, "LEFT": DpadLeft}
	goldenMisc   = map[string]uint8{"HOME": MiscHome, "BACK": MiscBack}
	goldenModes  = map[string]RequestedMode{"Stand": ModeStand, "Walk": ModeWalk, "Sleep": ModeSleep, "Calibrate": ModeCalibrate}
	goldenStride = map[string]StrideMode{"Dynamic": StrideDynamic, "Fixed": StrideFixed}
	goldenStick  = map[string]StickMode{"Steer": StickSteer, "Crab": StickCrab}
)

func parseBits[T uint8 | uint16](names map[string]T, s string) (T, error) {
	var out T
	if s == "-" {
		return 0, nil
	}
	for _, n := range strings.Split(s, "+") {
		b, ok := names[n]
		if !ok || out&b != 0 {
			return 0, fmt.Errorf("bad name %q", n)
		}
		out |= b
	}
	return out, nil
}

func lookup[T any](names map[string]T, s string) (T, error) {
	v, ok := names[s]
	if !ok {
		return v, fmt.Errorf("bad name %q", s)
	}
	return v, nil
}

func atoi(s string, lo, hi int64) (int64, error) {
	v, err := strconv.ParseInt(s, 10, 64)
	if err != nil || v < lo || v > hi {
		return 0, fmt.Errorf("bad number %q", s)
	}
	return v, nil
}

type goldenStep struct {
	line int
	pad  PadState
	now  uint32
	want Desired
}

type goldenSequence struct {
	name  string
	steps []goldenStep
}

func parseStep(f []string) (s goldenStep, err error) {
	if len(f) != 24 || f[9] != "=" {
		return s, fmt.Errorf("want 24 fields with '=' tenth, got %d", len(f))
	}
	var n [6]int64
	lims := [6][2]int64{{0, 0xFFFFFFFF}, {0, 1}, {-512, 512}, {-512, 512}, {-512, 512}, {-512, 512}}
	for i := range n {
		if n[i], err = atoi(f[i], lims[i][0], lims[i][1]); err != nil {
			return s, err
		}
	}
	s.now = uint32(n[0])
	s.pad = PadState{Connected: n[1] == 1, AxisX: int32(n[2]), AxisY: int32(n[3]), AxisRX: int32(n[4]), AxisRY: int32(n[5])}
	if s.pad.Buttons, err = parseBits(goldenButtons, f[6]); err != nil {
		return s, err
	}
	if s.pad.Dpad, err = parseBits(goldenDpad, f[7]); err != nil {
		return s, err
	}
	if s.pad.Misc, err = parseBits(goldenMisc, f[8]); err != nil {
		return s, err
	}
	w := &s.want
	if w.Mode, err = lookup(goldenModes, f[10]); err != nil {
		return s, err
	}
	if w.Stride, err = lookup(goldenStride, f[17]); err != nil {
		return s, err
	}
	if w.StickMode, err = lookup(goldenStick, f[18]); err != nil {
		return s, err
	}
	var v [11]int64
	idx := [11]int{11, 12, 13, 14, 15, 16, 19, 20, 21, 22, 23}
	lo := [11]int64{-100, -100, -100, 0, 0, 0, 0, 0, 0, 0, 0}
	hi := [11]int64{100, 100, 100, 100, 100, 255, 1, 255, 255, 255, 255}
	for i := range v {
		if v[i], err = atoi(f[idx[i]], lo[i], hi[i]); err != nil {
			return s, err
		}
	}
	w.TranslateX, w.TranslateY, w.Rotate = int8(v[0]), int8(v[1]), int8(v[2])
	w.Height, w.Speed, w.Gait = uint8(v[3]), uint8(v[4]), uint8(v[5])
	w.ControllerConnected = v[6] == 1
	w.ClearCounter, w.ModeRequestCounter = uint8(v[7]), uint8(v[8])
	w.Gesture, w.GestureRequestCounter = uint8(v[9]), uint8(v[10])
	return s, nil
}

func readGolden(t *testing.T) []goldenSequence {
	t.Helper()
	f, err := os.Open(goldenPath())
	if err != nil {
		t.Fatalf("golden steps: %v", err)
	}
	defer f.Close()
	var seqs []goldenSequence
	sc := bufio.NewScanner(f)
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		if name, ok := strings.CutPrefix(text, "reset "); ok && name != "" {
			seqs = append(seqs, goldenSequence{name: name})
			continue
		}
		if len(seqs) == 0 {
			t.Fatalf("line %d: step before a reset", line)
		}
		s, err := parseStep(strings.Split(text, " "))
		if err != nil {
			t.Fatalf("line %d: %v", line, err)
		}
		s.line = line
		seqs[len(seqs)-1].steps = append(seqs[len(seqs)-1].steps, s)
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	return seqs
}

func TestGoldenSteps(t *testing.T) {
	seqs := readGolden(t)
	steps := 0
	for _, seq := range seqs {
		m := NewMapper()
		for _, s := range seq.steps {
			got := m.Update(s.pad, s.now)
			if got != s.want {
				t.Errorf("%s, line %d: got %+v, want %+v", seq.name, s.line, got, s.want)
			}
			steps++
		}
	}
	// A truncated or emptied file must not pass.
	if len(seqs) < 8 || steps < 100 {
		t.Fatalf("only %d sequences and %d steps", len(seqs), steps)
	}
}

func TestToCommand(t *testing.T) {
	d := Desired{Mode: ModeWalk, TranslateX: -3, TranslateY: 4, Rotate: -5, Height: 6, Speed: 7, Gait: 2,
		Stride: StrideFixed, StickMode: StickCrab, ControllerConnected: true, ClearCounter: 8,
		ModeRequestCounter: 9, Gesture: 3, GestureRequestCounter: 10}
	c := ToCommand(d, true)
	if c.Mode != 1 || c.TranslateX != -3 || c.TranslateY != 4 || c.Rotate != -5 || c.Height != 6 || c.Speed != 7 ||
		c.Gait != 2 || c.Stride != 1 || c.Flags != 3 || c.ClearCounter != 8 || c.ModeRequestCounter != 9 ||
		c.Gesture != 3 || c.GestureRequestCounter != 10 {
		t.Errorf("ToCommand(%+v, true) = %+v", d, c)
	}
	d.ControllerConnected = false
	if c := ToCommand(d, false); c.Flags != 0 {
		t.Errorf("flags %d, want 0", c.Flags)
	}
}
