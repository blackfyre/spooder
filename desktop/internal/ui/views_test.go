package ui

import (
	"image"
	"image/png"
	"math"
	"os"
	"path/filepath"
	"sync/atomic"
	"testing"
	"time"

	"gioui.org/f32"
	"gioui.org/gpu/headless"
	"gioui.org/io/input"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/unit"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/legview"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
)

func testViews(t *testing.T, dark bool) (*viewsWindow, *session.Session) {
	t.Helper()
	w, _, _ := testWindow(t, dark)
	env := w.env
	env.Selection = new(atomic.Int32)
	v, err := newViewsWindow(env)
	if err != nil {
		t.Fatal(err)
	}
	return v, env.Session
}

func viewsFrame(v *viewsWindow, size image.Point) {
	var ops op.Ops
	router := new(input.Router)
	gtx := layout.Context{Ops: &ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
		Constraints: layout.Exact(size), Now: nextFrameTime(), Source: router.Source()}
	v.frame(gtx)
	router.Frame(&ops)
}

// Spec "No pose" and a pose drawn: the window follows the session alone.
func TestViewsWindowFollowsThePose(t *testing.T) {
	v, s := testViews(t, false)
	viewsFrame(v, image.Pt(1100, 900))
	if st := legsFor(v.d.snap, selection{}); st.Current {
		t.Fatal("current pose before connecting")
	}
	s.LinkState(link.State{Status: link.Connected, Port: servo})
	s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(walkingAngles, true)}, link.Stats{})
	v.env.Selection.Store(7)
	viewsFrame(v, image.Pt(1100, 900))
	st := legsFor(v.d.snap, selection{joint: v.selectedJoint()})
	if !st.Current || st.Leg != 2 {
		t.Fatalf("pose %v, selected leg %d", st.Current, st.Leg)
	}
	clock = clock.Add(2 * time.Second) // spec "No pose": stale after 1 s
	viewsFrame(v, image.Pt(1100, 900))
	if legsFor(v.d.snap, selection{}).Current {
		t.Fatal("a stale pose is drawn")
	}
}

// Spec "Shared scale": the top, front and left views share one scale, and
// each still holds its whole extent in its cell.
func TestViewsShareOneScale(t *testing.T) {
	for _, cell := range []image.Point{{540, 440}, {300, 250}, {900, 300}} {
		s := orthoScale(cell)
		for _, view := range []legview.View{legview.ViewTop, legview.ViewFront, legview.ViewLeft} {
			b := legview.ViewBounds(view)
			vp := fitAt(b, cell, s)
			a, c := vp.pt(legview.Vec2{X: 0, Y: 0}), vp.pt(legview.Vec2{X: 100, Y: 0})
			if math.Abs(float64(c.X-a.X)-100*s) > 1e-3 {
				t.Fatalf("cell %v view %d: 100 mm is %v px, want %v", cell, view, c.X-a.X, 100*s)
			}
			lo, hi := vp.pt(legview.Vec2{X: b[0], Y: b[3]}), vp.pt(legview.Vec2{X: b[2], Y: b[1]})
			if lo.X < -0.5 || lo.Y < -0.5 || hi.X > float32(cell.X)+0.5 || hi.Y > float32(cell.Y)+0.5 {
				t.Fatalf("cell %v view %d: extent %v..%v outside", cell, view, lo, hi)
			}
		}
	}
}

// The top bar's button opens the window.
func TestViewsButton(t *testing.T) {
	w, router, _ := testWindow(t, false)
	opened := 0
	w.env.OpenViews = func() { opened++ }
	frame(w, router)
	click(w, router, &w.views)
	if opened != 1 {
		t.Fatalf("opened %d times", opened)
	}
}

// Spec "Opened twice": while the window is open, opening it again brings
// nothing new up; once closed, it opens again.
func TestViewsOpenOnce(t *testing.T) {
	w, _, _ := testWindow(t, false)
	runs := make(chan chan struct{}, 4)
	run := func(*viewsWindow) {
		done := make(chan struct{})
		runs <- done
		<-done // open until closed
	}
	openViews(w.env, run)
	first := <-runs
	openViews(w.env, run)
	select {
	case <-runs:
		t.Fatal("a second window opened")
	case <-time.After(50 * time.Millisecond):
	}
	close(first) // closed
	for i := 0; i < 100 && viewsOpen.Load(); i++ {
		time.Sleep(5 * time.Millisecond)
	}
	openViews(w.env, run)
	select {
	case again := <-runs:
		close(again)
	case <-time.After(time.Second):
		t.Fatal("not opened again after closing")
	}
}

// TestViewsScreenshots renders the four-view window off screen, like
// TestScreenshots: SPOODER_SCREENSHOTS=<dir> go test -run ViewsScreenshots.
func TestViewsScreenshots(t *testing.T) {
	dir := os.Getenv("SPOODER_SCREENSHOTS")
	if dir == "" {
		t.Skip("set SPOODER_SCREENSHOTS to a directory")
	}
	for _, dark := range []bool{false, true} {
		v, s := testViews(t, dark)
		s.LinkState(link.State{Status: link.Connected, Port: servo})
		shoot := func(name string, size image.Point) {
			hw, err := headless.NewWindow(size.X, size.Y)
			if err != nil {
				t.Skip("no headless GPU:", err)
			}
			defer hw.Release()
			var ops op.Ops
			router := new(input.Router)
			gtx := layout.Context{Ops: &ops, Metric: unit.Metric{PxPerDp: 1, PxPerSp: 1},
				Constraints: layout.Exact(size), Now: nextFrameTime(), Source: router.Source()}
			v.frame(gtx)
			if err := hw.Frame(&ops); err != nil {
				t.Fatal(err)
			}
			img := image.NewRGBA(image.Rectangle{Max: size})
			if err := hw.Screenshot(img); err != nil {
				t.Fatal(err)
			}
			theme := "light"
			if dark {
				theme = "dark"
			}
			f, err := os.Create(filepath.Join(dir, theme+"-views-"+name+".png"))
			if err != nil {
				t.Fatal(err)
			}
			defer f.Close()
			if err := png.Encode(f, img); err != nil {
				t.Fatal(err)
			}
		}
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(calibrationAngles(), true)}, link.Stats{})
		shoot("calibration", image.Pt(1100, 900))
		v.env.Selection.Store(13) // leg 5 femur
		s.Frame(protocol.Frame{Type: protocol.TypePose, Pose: testPose(walkingAngles, true)}, link.Stats{})
		shoot("walking", image.Pt(1100, 900))
		shoot("walking-min", image.Pt(640, 520))
	}
}

// A full appearance channel never blocks the watcher: the newest wins.
func TestOfferLookNeverBlocks(t *testing.T) {
	ch := make(chan appearance.Settings, 2)
	done := make(chan struct{})
	go func() {
		for i := 0; i < 10; i++ {
			offerLook(ch, appearance.Settings{Dark: i%2 == 1})
		}
		offerLook(ch, appearance.Settings{Dark: true})
		close(done)
	}()
	select {
	case <-done:
	case <-time.After(time.Second):
		t.Fatal("offerLook blocked on a full channel")
	}
	var last appearance.Settings
	for len(ch) > 0 {
		last = <-ch
	}
	if !last.Dark {
		t.Fatal("the newest setting was dropped")
	}
}

// Every leg keeps its number: in the calibration pose the left view puts
// mirrored legs at one place, and they share a label.
func TestEveryLegNumbered(t *testing.T) {
	legs := legview.Body3(legview.FromFrame(testPose(calibrationAngles(), true)))
	vp := fit(legview.ViewBounds(legview.ViewLeft), image.Pt(540, 440))
	anchors := make([]f32.Point, legview.Legs)
	order := []int{0, 1, 2, 3, 4, 5}
	for leg, l := range legs {
		anchors[leg] = projected(legview.ViewLeft, vp, l.Points[2])
	}
	groups := numberGroups(anchors, order, 10)
	seen := map[int]bool{}
	shared := false
	for _, g := range groups {
		for _, leg := range g {
			seen[leg] = true
		}
		if len(g) > 1 {
			shared = true
		}
	}
	if len(seen) != legview.Legs || !shared {
		t.Fatalf("groups %v", groups)
	}
}

// A selected leg behind a nearer one stays visible: the plan draws it again
// after every leg, and the limit rings after that.
func TestSelectedLegDrawnOverNearerLegs(t *testing.T) {
	legs := legview.Body3(legview.FromFrame(testPose(calibrationAngles(), true)))
	for view := legview.View(0); view < legview.ViewCount; view++ {
		for _, selected := range []int{4, 1} { // a right leg (far side in the left view) and a left one
			plan, order := drawPlan(view, legs, selected)
			lastLeg, sel, firstRing, plates := -1, -1, -1, 0
			legsDrawn := []int{}
			for i, step := range plan {
				switch step.kind {
				case stepLeg:
					lastLeg = i
					legsDrawn = append(legsDrawn, step.leg)
				case stepSelected:
					sel = i
					if step.leg != selected {
						t.Fatalf("view %d: overlay of leg %d", view, step.leg)
					}
				case stepRings:
					if firstRing < 0 {
						firstRing = i
					}
				case stepPlate:
					plates++
				}
			}
			if plates != 1 || sel < lastLeg || firstRing < sel || len(legsDrawn) != legview.Legs {
				t.Fatalf("view %d selected %d: plan %v", view, selected, plan)
			}
			for i := range order {
				if legsDrawn[i] != order[i] {
					t.Fatalf("view %d: legs %v, not far to near %v", view, legsDrawn, order)
				}
			}
		}
	}
}
