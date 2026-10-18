package protocol

import (
	"bytes"
	"math/rand"
	"testing"
)

func sampleCommand() Command {
	return Command{Mode: 1, TranslateX: -100, TranslateY: 73, Rotate: -5, Height: 40, Speed: 100, Gait: 5, Stride: 1,
		Flags: FlagControllerConnected | FlagEmergencyStop, ClearCounter: 200, ModeRequestCounter: 255}
}

func sampleTelemetry() Telemetry {
	return Telemetry{BodyMode: 2, Gait: 3, OutputsOn: true, Faults: 0x8011, VoltageMv: 6020, CurrentMa: 65535,
		ValidFrames: 1234, DiscardedFrames: 7, SensorsAvailable: SensorToF | SensorAttitude, ToFMm: 150, ToFStatus: 11,
		PitchCentideg: -1000, RollCentideg: 32767, Flags: TelemetrySourceConflict | TelemetryBridgeInControl}
}

func sampleSet() CalibrationSet {
	return CalibrationSet{RequestID: 201, Joint: 17, OffsetCentideg: -2450, Pulse0Us: 2600, Pulse180Us: 400, Flags: CalibrationReversed}
}

func sampleReport() CalibrationReport {
	r := CalibrationReport{LastRequestID: 255, Result: uint8(ResultClamped)}
	for i := range r.Joints {
		r.Joints[i] = CalibrationJoint{OffsetCentideg: int16(i*1000 - 9000), Pulse0Us: uint16(400 + i), Pulse180Us: uint16(2600 - i)}
		if i%2 == 1 {
			r.Joints[i].Flags = CalibrationReversed
		}
	}
	return r
}

// feed pushes bytes and calls Next after each, without Idle.
func feed(d *Decoder, data []byte) []Frame {
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
	return frames
}

func TestCRCCheckValue(t *testing.T) {
	if got := CRC16([]byte("123456789")); got != 0x29B1 {
		t.Fatalf("crc %04x", got)
	}
}

func TestFrameSizes(t *testing.T) {
	for _, tc := range []struct {
		got, want int
	}{
		{len(EncodeCommand(sampleCommand(), 0)), 19},
		{len(EncodeTelemetry(sampleTelemetry(), 0)), 39},
		{len(EncodeCalibrationSet(sampleSet(), 0)), 16},
		{len(EncodeCalibrationReport(sampleReport(), 0)), 135},
		{CalibrationReportPayloadSize, 128},
		{MaxFrameSize, 135},
	} {
		if tc.got != tc.want {
			t.Errorf("size %d, want %d", tc.got, tc.want)
		}
	}
}

func TestRoundTrips(t *testing.T) {
	for _, f := range []Frame{
		{Type: TypeCommand, Seq: 42, Command: sampleCommand()},
		{Type: TypeTelemetry, Seq: 9, Telemetry: sampleTelemetry()},
		{Type: TypeCalibrationSet, Seq: 250, CalibrationSet: sampleSet()},
		{Type: TypeCalibrationReport, Seq: 3, CalibrationReport: sampleReport()},
	} {
		var d Decoder
		frames := feed(&d, Encode(f))
		if len(frames) != 1 || frames[0] != f {
			t.Errorf("%s: got %+v", f.Type.Name(), frames)
		}
		if d.Valid() != 1 || d.Discarded() != 0 {
			t.Errorf("%s: valid %d discarded %d", f.Type.Name(), d.Valid(), d.Discarded())
		}
	}
	// The telemetry flags are byte 21 of the payload.
	if b := EncodeTelemetry(sampleTelemetry(), 0); b[HeaderSize+21] != sampleTelemetry().Flags {
		t.Fatal("telemetry flags not at payload byte 21")
	}
}

func TestEverySingleBitCorruptionIsRejected(t *testing.T) {
	for _, good := range [][]byte{
		EncodeCommand(sampleCommand(), 1),
		EncodeCalibrationSet(sampleSet(), 1),
		EncodeCalibrationReport(sampleReport(), 1),
	} {
		for i := range good {
			for bit := 0; bit < 8; bit++ {
				bad := bytes.Clone(good)
				bad[i] ^= 1 << bit
				var d Decoder
				if n := len(feed(&d, bad)); n != 0 {
					t.Fatalf("byte %d bit %d: corrupted frame accepted", i, bit)
				}
				// The next valid frame is still accepted.
				frames := feed(&d, good)
				if len(frames) != 1 || frames[0].Seq != 1 {
					t.Fatalf("byte %d bit %d: valid frame after corruption: %d frames", i, bit, len(frames))
				}
			}
		}
	}
}

func TestCorruptedFrameIsCounted(t *testing.T) {
	b := EncodeCommand(sampleCommand(), 1)
	b[8] ^= 0x10
	var d Decoder
	feed(&d, b)
	if d.Valid() != 0 || d.Discarded() < 1 {
		t.Fatalf("valid %d discarded %d", d.Valid(), d.Discarded())
	}
}

func TestResynchronisesOverGarbage(t *testing.T) {
	garbage := []byte{0x00, 0xA5, 0xA5, 0x01, 0x02, 0xA5, 0x01, 0x01, 0x07, 0x0C, 0x13, 0xFF, 0xA5}
	var d Decoder
	frames := feed(&d, EncodeCommand(sampleCommand(), 10))
	frames = append(frames, feed(&d, garbage)...)
	frames = append(frames, feed(&d, EncodeTelemetry(sampleTelemetry(), 11))...)
	if len(frames) != 2 || frames[1].Seq != 11 || frames[1].Type != TypeTelemetry {
		t.Fatalf("frames %+v", frames)
	}
}

func TestFrameHiddenInARejectedCandidateIsFound(t *testing.T) {
	falseHeader := []byte{Start, Version, byte(TypeTelemetry), 0, TelemetryPayloadSize}
	var d Decoder
	frames := feed(&d, falseHeader)
	frames = append(frames, feed(&d, EncodeCommand(sampleCommand(), 77))...)
	frames = append(frames, feed(&d, make([]byte, TelemetryFrameSize))...)
	if len(frames) != 1 || frames[0].Seq != 77 {
		t.Fatalf("frames %+v", frames)
	}
}

func TestFrameBehindIncompleteFalseStartIsReturnedWhenIdle(t *testing.T) {
	falseHeader := []byte{Start, Version, byte(TypeTelemetry), 0, TelemetryPayloadSize}
	var d Decoder
	frames := feed(&d, falseHeader)
	frames = append(frames, feed(&d, EncodeCommand(sampleCommand(), 78))...)
	if len(frames) != 0 {
		t.Fatal("frame decoded before idle")
	}
	d.Idle()
	f, ok := d.Next()
	if !ok || f.Seq != 78 || d.Discarded() != 1 {
		t.Fatalf("after idle: %v %+v discarded %d", ok, f, d.Discarded())
	}
	if _, ok := d.Next(); ok {
		t.Fatal("second frame")
	}
}

func TestCommandShapedBytesInsideTelemetryDoNotSplitIt(t *testing.T) {
	inner := EncodeCommand(Command{Mode: 1, Speed: 44, Flags: FlagControllerConnected}, 130)
	outer := make([]byte, TelemetryFrameSize)
	copy(outer, []byte{Start, Version, byte(TypeTelemetry), 9, TelemetryPayloadSize})
	copy(outer[HeaderSize+5:], inner)
	put16(outer[TelemetryFrameSize-2:], CRC16(outer[1:HeaderSize+TelemetryPayloadSize]))
	var d Decoder
	frames := feed(&d, outer)
	if len(frames) != 1 || frames[0].Type != TypeTelemetry || frames[0].Seq != 9 || d.Discarded() != 0 {
		t.Fatalf("frames %+v discarded %d", frames, d.Discarded())
	}
}

func TestIdleKeepsACompleteBufferUntouched(t *testing.T) {
	var d Decoder
	feed(&d, EncodeCommand(sampleCommand(), 3))
	d.Idle()
	if d.Discarded() != 0 {
		t.Fatal("idle discarded a decoded frame")
	}
}

func withHeaderByte(at int, v byte) []byte {
	b := EncodeCommand(sampleCommand(), 1)
	b[at] = v
	put16(b[len(b)-2:], CRC16(b[1:HeaderSize+CommandPayloadSize]))
	return b
}

func TestBadHeadersAreRejected(t *testing.T) {
	inconsistent := EncodeCommand(sampleCommand(), 1)
	inconsistent[4] = CommandPayloadSize - 1
	for name, b := range map[string][]byte{
		"version 1":           withHeaderByte(1, 1),
		"version 3":           withHeaderByte(1, 3),
		"unknown type":        withHeaderByte(2, 5),
		"inconsistent length": inconsistent,
	} {
		var d Decoder
		if n := len(feed(&d, b)); n != 0 || d.Discarded() != 1 {
			t.Errorf("%s: %d frames, %d discarded", name, n, d.Discarded())
		}
	}
}

func TestFrameSplitAcrossPushesWithoutNext(t *testing.T) {
	var d Decoder
	for _, b := range EncodeCommand(sampleCommand(), 5) {
		d.Push(b)
	}
	if f, ok := d.Next(); !ok || f.Seq != 5 {
		t.Fatal("frame not decoded")
	}
	if _, ok := d.Next(); ok {
		t.Fatal("second frame")
	}
}

func TestTelemetryAndReportInOneBurst(t *testing.T) {
	burst := append(EncodeTelemetry(sampleTelemetry(), 1), EncodeCalibrationReport(sampleReport(), 2)...)
	var d Decoder
	frames := feed(&d, burst)
	if len(frames) != 2 || frames[0].Type != TypeTelemetry || frames[1].Type != TypeCalibrationReport || d.Discarded() != 0 {
		t.Fatalf("frames %d discarded %d", len(frames), d.Discarded())
	}
}

// Discard receives exactly the bytes that are not part of a decoded frame,
// in order, so a host can recover interleaved text.
func TestDiscardReturnsEveryByteOutsideFrames(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	for round := 0; round < 200; round++ {
		var stream, wantText []byte
		var wantFrames []Frame
		for part := 0; part < 8; part++ {
			if rng.Intn(2) == 0 {
				text := []byte("debug line\n")
				if rng.Intn(4) == 0 {
					text = append([]byte{Start}, text...) // a stray start marker
				}
				stream = append(stream, text...)
				wantText = append(wantText, text...)
			} else {
				f := Frame{Type: TypeCommand, Seq: uint8(part), Command: sampleCommand()}
				stream = append(stream, Encode(f)...)
				wantFrames = append(wantFrames, f)
			}
		}
		var text []byte
		d := Decoder{Discard: func(b byte) { text = append(text, b) }}
		frames := decodeAll(&d, stream)
		if !bytes.Equal(text, wantText) {
			t.Fatalf("round %d: text %q, want %q", round, text, wantText)
		}
		if len(frames) != len(wantFrames) {
			t.Fatalf("round %d: %d frames, want %d", round, len(frames), len(wantFrames))
		}
	}
}
