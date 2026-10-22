package legview

import (
	"bufio"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// gravityGoldenPath is test/golden/centre_of_gravity.txt, written from the
// C++ model by test/test_centre_of_gravity.
var gravityGoldenPath = filepath.Join("..", "..", "..", "test", "golden", "centre_of_gravity.txt")

// Every golden pose, with and without the battery, gives the C++ model's
// centre, mass and margin.
func TestCentreOfGravityMatchesGolden(t *testing.T) {
	f, err := os.Open(gravityGoldenPath)
	if err != nil {
		t.Fatalf("golden centre of gravity: %v", err)
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	count := 0
	for line := 1; sc.Scan(); line++ {
		text := sc.Text()
		if text == "" || strings.HasPrefix(text, "#") {
			continue
		}
		parts := strings.Split(text, " ")
		if len(parts) != 2+Legs*JointsPerLeg+5 {
			t.Fatalf("line %d: %q", line, text)
		}
		num := func(i int) float64 {
			v, err := strconv.ParseFloat(parts[i], 64)
			if err != nil {
				t.Fatalf("line %d: %v", line, err)
			}
			return v
		}
		var p Pose
		for leg := range p.Legs {
			i := 2 + leg*JointsPerLeg
			p.Legs[leg] = Leg{Set: true, Angles: Angles{num(i), num(i + 1), num(i + 2)}}
		}
		legs := Body3(p)
		g, ok := CentreOfGravity(legs, parts[1] == "1")
		if !ok {
			t.Fatalf("line %d: no estimate", line)
		}
		n := 2 + Legs*JointsPerLeg
		want := Point3{num(n), num(n + 1), num(n + 2)}
		if !near(g.Centre.X, want.X, 0.01) || !near(g.Centre.Y, want.Y, 0.01) || !near(g.Centre.Z, want.Z, 0.01) {
			t.Errorf("%s battery %s: centre %+v, want %+v", parts[0], parts[1], g.Centre, want)
		}
		if !near(g.MassG, num(n+3), 0.1) {
			t.Errorf("%s: mass %.1f, want %s", parts[0], g.MassG, parts[n+3])
		}
		hull := SupportHull(legs)
		if parts[n+4] == "none" {
			if hull != nil {
				t.Errorf("%s: a support polygon, want none", parts[0])
			}
		} else if hull == nil {
			t.Errorf("%s: no support polygon", parts[0])
		} else if m := SupportMargin(hull, Vec2{g.Centre.X, g.Centre.Y}); !near(m, num(n+4), 0.01) {
			t.Errorf("%s: margin %.3f, want %s", parts[0], m, parts[n+4])
		}
		count++
	}
	if err := sc.Err(); err != nil {
		t.Fatal(err)
	}
	if count == 0 {
		t.Fatal("no golden poses")
	}
}

// A leg never set gives no estimate.
func TestCentreOfGravityNeedsEveryLeg(t *testing.T) {
	var p Pose
	for leg := range p.Legs {
		p.Legs[leg] = Leg{Set: leg != 3, Angles: Angles{0, 17, 95}}
	}
	if _, ok := CentreOfGravity(Body3(p), false); ok {
		t.Fatal("an estimate with a leg never set")
	}
}

// Spec "Too few feet down": two feet low, the rest higher.
func TestSupportHullNeedsThreeFeet(t *testing.T) {
	var legs [Legs]Leg3
	for leg := range legs {
		z := -50.0
		if leg < 2 {
			z = -100
		}
		legs[leg] = Leg3{Set: true, Points: [4]Point3{{}, {}, {}, {X: float64(leg) * 30, Y: float64(leg * leg), Z: z}}}
	}
	if h := SupportHull(legs); h != nil {
		t.Fatalf("hull %v from two feet down", h)
	}
}
