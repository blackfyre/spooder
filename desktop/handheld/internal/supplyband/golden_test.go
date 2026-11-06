package supplyband

import (
	"bufio"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// goldenPath is test/golden/supply_bands.txt, which test/test_supply_band
// runs the C++ rule over. SPOODER_SUPPLY_BANDS_GOLDEN names another copy,
// as for the C++ test.
func goldenPath() string {
	if p := os.Getenv("SPOODER_SUPPLY_BANDS_GOLDEN"); p != "" {
		return p
	}
	return filepath.Join("..", "..", "..", "..", "test", "golden", "supply_bands.txt")
}

func parseU16(s string) (uint16, bool) {
	v, err := strconv.ParseUint(s, 10, 16)
	return uint16(v), err == nil
}

// Every case in the fixture gets its band.
func TestGoldenCases(t *testing.T) {
	f, err := os.Open(goldenPath())
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	var l Limits
	haveLimits, cases := false, 0
	sc := bufio.NewScanner(f)
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		fs := strings.Split(text, " ")
		if len(fs) == 6 && fs[0] == "limits" {
			var v [5]uint16
			for i := range v {
				var ok bool
				if v[i], ok = parseU16(fs[i+1]); !ok {
					t.Fatalf("line %d: malformed", line)
				}
			}
			l, haveLimits = Limits{v[0], v[1], v[2], v[3], v[4]}, true
			continue
		}
		reading, ok := uint16(0), false
		if len(fs) == 3 {
			reading, ok = parseU16(fs[1])
		}
		if !ok || !haveLimits || (fs[0] != "voltage" && fs[0] != "current") {
			t.Fatalf("line %d: malformed", line)
		}
		got := Voltage(l, reading)
		if fs[0] == "current" {
			got = Current(l, reading)
		}
		cases++
		if got.String() != fs[2] {
			t.Errorf("line %d: %s %d: band %s, want %s", line, fs[0], reading, got, fs[2])
		}
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	if cases == 0 {
		t.Fatal("no cases in the fixture")
	}
}

// The limits are a diagnostics frame's, field by field.
func TestFromDiagnostics(t *testing.T) {
	d := protocol.Diagnostics{BrownoutMv: 4800, MinMv: 5200, MaxMv: 6800, SustainedCurrentMa: 6000, TripCurrentMa: 9000}
	if got, want := FromDiagnostics(d), (Limits{4800, 5200, 6800, 6000, 9000}); got != want {
		t.Errorf("FromDiagnostics = %+v, want %+v", got, want)
	}
}
