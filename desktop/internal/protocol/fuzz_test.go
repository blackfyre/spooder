package protocol

import (
	"bytes"
	"testing"
)

// Fuzz tests of the decoder (openspec link-fuzzing, "Fuzzed frame
// decoder"). The seed corpus runs in every `go test`; `mise run fuzz` adds
// random inputs.

// canonicalSpan reports whether a CRC-valid frame's bytes are the ones
// Encode gives for its decoded fields: reserved and unused payload bytes
// zero, boolean fields 0 or 1. Only these positions may differ between an
// accepted frame and its re-encoding (besides the CRC that covers them).
func canonicalSpan(span []byte) bool {
	p := span[HeaderSize : len(span)-CRCSize]
	switch FrameType(span[2]) {
	case TypeCommand:
		return p[13] == 0 // reserved
	case TypePose:
		return p[1] == 0 // reserved
	case TypeTelemetry:
		if p[2] > 1 { // outputsOn
			return false
		}
		for _, b := range p[27:] { // unused
			if b != 0 {
				return false
			}
		}
	}
	return true
}

// withPayload returns a copy of frame with its payload changed by edit and
// the CRC recomputed, so the decoder accepts it.
func withPayload(frame []byte, edit func(p []byte)) []byte {
	out := append([]byte(nil), frame...)
	edit(out[HeaderSize : len(out)-CRCSize])
	return finish(out)
}

func samplePose() Pose {
	var p Pose
	p.Flags = PoseOutputsOn
	for i := range p.Joints {
		p.Joints[i] = PoseJoint{CommandedCentideg: int16(i*1500 - 12000), ServoCentideg: int16(i*1100 - 900), PulseUs: uint16(500 + 100*i)}
	}
	return p
}

// nonCanonicalCase is a CRC-valid frame that is not canonical: want is
// what it decodes to, and Encode(want) its canonical bytes.
type nonCanonicalCase struct {
	name  string
	frame []byte
	want  Frame
}

func nonCanonicalCases() []nonCanonicalCase {
	command := Frame{Type: TypeCommand, Seq: 42, Command: sampleCommand()}
	pose := Frame{Type: TypePose, Seq: 77, Pose: samplePose()}
	telemetry := Frame{Type: TypeTelemetry, Seq: 9, Telemetry: sampleTelemetry()}
	return []nonCanonicalCase{
		{"command_reserved_13", withPayload(Encode(command), func(p []byte) { p[13] = 0x5A }), command},
		{"pose_reserved_1", withPayload(Encode(pose), func(p []byte) { p[1] = 0x80 }), pose},
		{"telemetry_unused_27_31", withPayload(Encode(telemetry), func(p []byte) {
			copy(p[27:], []byte{0x01, 0xA5, 0x7F, 0x80, 0xFF})
		}), telemetry},
		{"telemetry_outputs_on_2", withPayload(Encode(telemetry), func(p []byte) { p[2] = 2 }), telemetry},
	}
}

// decoderSeeds are the golden frames and the extra seeds of design D2.
func decoderSeeds(tb testing.TB) [][]byte {
	var seeds [][]byte
	var accepted [][]byte
	for _, v := range readVectors(tb) {
		seeds = append(seeds, v.frame)
		if v.fields[0] != "rejected" {
			accepted = append(accepted, v.frame)
		}
	}
	a, b := accepted[0], accepted[len(accepted)-1]
	cat := func(parts ...[]byte) []byte { return bytes.Join(parts, nil) }
	seeds = append(seeds,
		cat(a, b), // two frames back to back
		cat([]byte("ok\r\n\x00\xa5\x03"), a, []byte("\xa5\xff tail\n")), // garbage before and after
		a[:len(a)-3],  // truncated frame
		[]byte{Start}, // lone start marker
	)
	for _, c := range nonCanonicalCases() {
		seeds = append(seeds, c.frame)
	}
	return seeds
}

// decodeAlone decodes data with a fresh decoder and returns its frames and
// the number of bytes it discarded.
func decodeAlone(data []byte) ([]Frame, int) {
	discarded := 0
	d := Decoder{Discard: func(byte) { discarded++ }}
	frames := decodeAll(&d, data)
	return frames, discarded
}

// checkDecoder runs the properties of design D3 on one input.
func checkDecoder(t *testing.T, data []byte) {
	cur := 0 // next input byte not yet accounted for
	var d Decoder
	d.Discard = func(b byte) {
		if cur >= len(data) || data[cur] != b {
			t.Fatalf("discarded %#02x as input byte %d of %d, out of order", b, cur, len(data))
		}
		cur++
	}
	pending := func(op string) {
		if n := d.Pending(); n < 0 || n > MaxFrameSize {
			t.Fatalf("after %s: %d bytes pending, more than one frame (%d)", op, n, MaxFrameSize)
		}
	}
	frame := func(f Frame) {
		n := PayloadSize(f.Type)
		if n < 0 {
			t.Fatalf("decoded a frame of unknown type %d", f.Type)
		}
		size := HeaderSize + n + CRCSize
		if cur+size > len(data) {
			t.Fatalf("%s frame at input byte %d runs past the input (%d bytes)", f.Type.Name(), cur, len(data))
		}
		span := data[cur : cur+size]
		cur += size
		checkFrame(t, f, span)
	}
	next := func() {
		for {
			f, ok := d.Next()
			pending("Next")
			if !ok {
				return
			}
			frame(f)
		}
	}
	for _, b := range data {
		d.Push(b)
		pending("Push")
		next()
	}
	// End of stream, as the link's reader does it.
	for i := 0; d.Pending() > 0; i++ {
		if i > MaxFrameSize {
			t.Fatalf("draining does not finish: %d bytes pending", d.Pending())
		}
		d.Idle()
		pending("Idle")
		next()
	}
	if cur != len(data) {
		t.Fatalf("%d of %d input bytes accounted for", cur, len(data))
	}
}

// checkFrame checks one accepted frame against its own input bytes.
func checkFrame(t *testing.T, f Frame, span []byte) {
	if frames, discarded := decodeAlone(span); len(frames) != 1 || frames[0] != f || discarded != 0 {
		t.Fatalf("%s span % x decoded alone: %d frames, %d bytes discarded, want the same frame", f.Type.Name(), span, len(frames), discarded)
	}
	enc := Encode(f)
	if frames, discarded := decodeAlone(enc); len(frames) != 1 || frames[0] != f || discarded != 0 {
		t.Fatalf("%s re-encoding % x decoded: %d frames, %d bytes discarded, want the same frame", f.Type.Name(), enc, len(frames), discarded)
	}
	if !canonicalSpan(enc) {
		t.Fatalf("%s re-encoding % x is not canonical", f.Type.Name(), enc)
	}
	if canonicalSpan(span) && !bytes.Equal(enc, span) {
		t.Fatalf("%s re-encoded:\n got % x\nwant % x", f.Type.Name(), enc, span)
	}
}

func FuzzDecoder(f *testing.F) {
	for _, s := range decoderSeeds(f) {
		f.Add(s)
	}
	f.Fuzz(checkDecoder)
}

// The non-canonical seeds are accepted, decode to the expected fields and
// re-encode to the canonical bytes.
func TestNonCanonicalFramesReencodeCanonically(t *testing.T) {
	for _, c := range nonCanonicalCases() {
		if canonicalSpan(c.frame) {
			t.Errorf("%s: seed % x is canonical", c.name, c.frame)
			continue
		}
		var d Decoder
		frames := decodeAll(&d, c.frame)
		if len(frames) != 1 || d.Discarded() != 0 {
			t.Errorf("%s: decoded %d frames, %d discarded; want it accepted", c.name, len(frames), d.Discarded())
			continue
		}
		if frames[0] != c.want {
			t.Errorf("%s fields:\n got %v\nwant %v", c.name, Fields(frames[0]), Fields(c.want))
		}
		want := Encode(c.want)
		if enc := Encode(frames[0]); !bytes.Equal(enc, want) {
			t.Errorf("%s re-encoded:\n got % x\nwant % x", c.name, enc, want)
		}
		checkDecoder(t, c.frame)
	}
}

// Push and Next work in the decoder's fixed buffer: a stream of golden
// frames (accepted and rejected) allocates nothing.
func TestDecoderPushNextAllocNothing(t *testing.T) {
	var stream []byte
	for _, v := range readVectors(t) {
		stream = append(stream, v.frame...)
	}
	var d Decoder
	allocs := testing.AllocsPerRun(100, func() {
		for _, b := range stream {
			d.Push(b)
			for {
				if _, ok := d.Next(); !ok {
					break
				}
			}
		}
	})
	if allocs != 0 {
		t.Fatalf("%v allocations per golden stream, want 0", allocs)
	}
	if d.Valid() == 0 {
		t.Fatal("no frame decoded")
	}
}
