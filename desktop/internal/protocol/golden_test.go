package protocol

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// goldenPath is test/golden/protocol_v3.txt, written by the C++ encoder
// (test/test_protocol_golden).
var goldenPath = filepath.Join("..", "..", "..", "test", "golden", "protocol_v3.txt")

type vector struct {
	name   string
	frame  []byte
	fields []string
}

func readVectors(t testing.TB) []vector {
	t.Helper()
	f, err := os.Open(goldenPath)
	if err != nil {
		t.Fatalf("golden vectors: %v", err)
	}
	defer f.Close()
	var out []vector
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 64*1024), 1024*1024)
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		parts := strings.Split(text, " ")
		if len(parts) < 3 {
			t.Fatalf("line %d: %q", line, text)
		}
		frame, err := hex.DecodeString(parts[1])
		if err != nil {
			t.Fatalf("line %d: %v", line, err)
		}
		out = append(out, vector{name: parts[0], frame: frame, fields: parts[2:]})
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	return out
}

// decodeAll feeds bytes one at a time, calling Next after each, and Idle at
// the end, like the link's reader.
func decodeAll(d *Decoder, data []byte) []Frame {
	var frames []Frame
	for _, b := range data {
		d.Push(b)
		for {
			f, ok := d.Next()
			if !ok {
				break
			}
			frames = append(frames, f)
		}
	}
	d.Idle()
	for {
		f, ok := d.Next()
		if !ok {
			break
		}
		frames = append(frames, f)
	}
	return frames
}

func TestGoldenVectors(t *testing.T) {
	vectors := readVectors(t)
	types := map[FrameType]bool{}
	rejected := 0
	for _, v := range vectors {
		var d Decoder
		frames := decodeAll(&d, v.frame)
		if len(v.fields) == 1 && v.fields[0] == "rejected" {
			rejected++
			if len(frames) != 0 || d.Discarded() == 0 {
				t.Errorf("%s: decoded %d frames, %d discarded; want it rejected", v.name, len(frames), d.Discarded())
			}
			continue
		}
		if len(frames) != 1 || d.Discarded() != 0 {
			t.Errorf("%s: decoded %d frames, %d discarded", v.name, len(frames), d.Discarded())
			continue
		}
		got := Fields(frames[0])
		if strings.Join(got, " ") != strings.Join(v.fields, " ") {
			t.Errorf("%s fields:\n got %s\nwant %s", v.name, strings.Join(got, " "), strings.Join(v.fields, " "))
		}
		if enc := Encode(frames[0]); !bytes.Equal(enc, v.frame) {
			t.Errorf("%s re-encoded:\n got %x\nwant %x", v.name, enc, v.frame)
		}
		types[frames[0].Type] = true
	}
	for _, ft := range []FrameType{TypeCommand, TypeTelemetry, TypeCalibrationSet, TypeCalibrationReport, TypePose} {
		if !types[ft] {
			t.Errorf("no golden vector of type %s", ft.Name())
		}
	}
	if rejected == 0 {
		t.Error("no rejected vectors")
	}
}

// The whole golden file as one stream decodes every valid frame in order.
func TestGoldenVectorsBackToBack(t *testing.T) {
	var stream []byte
	var want []string
	for _, v := range readVectors(t) {
		stream = append(stream, v.frame...)
		if v.fields[0] != "rejected" {
			want = append(want, v.name)
		}
	}
	var d Decoder
	frames := decodeAll(&d, stream)
	if len(frames) != len(want) {
		t.Fatalf("decoded %d frames, want %d", len(frames), len(want))
	}
}

// The bench simulation vector (simulation flag, fault bit 6): the decoder
// does not interpret the flags or fault bits, so a receiver that does not
// know them reads every other field as before.
func TestGoldenBenchSimulation(t *testing.T) {
	var found *vector
	vectors := readVectors(t)
	for i := range vectors {
		if vectors[i].name == "telemetry_bench_simulation" {
			found = &vectors[i]
		}
	}
	if found == nil {
		t.Fatal("no telemetry_bench_simulation vector")
	}
	var d Decoder
	frames := decodeAll(&d, found.frame)
	if len(frames) != 1 || d.Discarded() != 0 {
		t.Fatalf("decoded %d frames, %d discarded", len(frames), d.Discarded())
	}
	f := frames[0]
	if f.Type != TypeTelemetry || f.Seq != 40 {
		t.Fatalf("type %s seq %d, want telemetry seq 40", f.Type.Name(), f.Seq)
	}
	got := f.Telemetry
	// Every field other than the flags and faults, then those as copied.
	want := Telemetry{
		BodyMode: 2, Gait: 1, OutputsOn: true, VoltageMv: 6000, CurrentMa: 2160,
		ValidFrames: 812, DiscardedFrames: 3, BridgeValidFrames: 1207, BridgeDiscardedFrames: 1,
		Flags: got.Flags, Faults: got.Faults,
	}
	if got != want {
		t.Errorf("telemetry %+v, want %+v", got, want)
	}
	if got.Flags != TelemetrySimulated || got.Faults != 1<<6 {
		t.Errorf("flags %#x faults %#x, want %#x and 0x40", got.Flags, got.Faults, TelemetrySimulated)
	}
}
