// Package ui is the desktop calibrator's Gio user interface in the
// Blackfyre look (design D7). It draws session snapshots and turns the
// operator's input into session and link calls; it never touches the port.
// The window, theme and test scaffolding follow bopen's internal/ui.
package ui

import (
	"context"
	"fmt"
	"io"
	"os"
	"slices"
	"strings"
	"sync/atomic"
	"time"

	"gioui.org/app"
	"gioui.org/io/clipboard"
	"gioui.org/io/event"
	"gioui.org/io/key"
	"gioui.org/layout"
	"gioui.org/op"
	"gioui.org/op/paint"
	"gioui.org/text"
	"gioui.org/unit"
	"gioui.org/widget"
	"gioui.org/widget/material"

	"github.com/blackfyre/spooder/desktop/internal/appearance"
	"github.com/blackfyre/spooder/desktop/internal/calib"
	"github.com/blackfyre/spooder/desktop/internal/export"
	"github.com/blackfyre/spooder/desktop/internal/link"
	"github.com/blackfyre/spooder/desktop/internal/protocol"
	"github.com/blackfyre/spooder/desktop/internal/session"
	"github.com/blackfyre/spooder/desktop/internal/ui/fonts"
)

// Linker is what the window needs of the link.
type Linker interface {
	Connect(device link.PortInfo)
	Disconnect()
	Close()
}

// Redraw asks the window for a new frame; it may be used from any
// goroutine, and before the window exists (it then does nothing).
type Redraw struct{ win atomic.Pointer[app.Window] }

// Invalidate requests a frame.
func (r *Redraw) Invalidate() {
	if w := r.win.Load(); w != nil {
		w.Invalidate()
	}
}

// Env is what the window works with.
type Env struct {
	Session    *session.Session
	Redraw     *Redraw // the session's change hook calls Redraw.Invalidate
	Link       Linker
	Ports      func() ([]link.PortInfo, error)
	Port       string   // a port given with --port: preselected ahead of a Servo 2040
	ConfigDir  string   // for the remembered checkout
	Checkouts  []string // candidate checkouts, see CheckoutCandidates
	Appearance appearance.Settings
	// OpenViews opens the four-view window (nil: no button action), which
	// reads the editor's selected joint from Selection.
	OpenViews func()
	Selection *atomic.Int32
}

// Panes of the right-hand column.
const (
	tabLog = iota
	tabFrames
	tabExport
	tabLegs
	tabCharts
	tabCount
)

var tabNames = [tabCount]string{"debug log", "frames", "export", "legs", "charts"}

// Mode buttons, in display order.
var modeButtons = [3]session.RequestedMode{session.RequestStand, session.RequestCalibrate, session.RequestSleep}

// portRefresh is how often the port list is read again.
const portRefresh = time.Second

// noticeFor is how long a refused action's explanation stays.
const noticeFor = 4 * time.Second

type window struct {
	env  Env
	th   *material.Theme
	pal  Palette
	look appearance.Settings
	// looks delivers appearance changes to the UI goroutine.
	looks chan appearance.Settings
	win   *app.Window // nil in tests
	snap  session.Snapshot

	ports      []link.PortInfo
	portSel    int  // index into ports, -1 for none
	picked     bool // the operator chose a port (not a preselection)
	portsCh    chan []link.PortInfo
	portClicks []widget.Clickable
	portList   layout.List
	// device is the device the operator connected to, nil when released.
	device *link.PortInfo

	refresh, connect, control, estop, clearFault widget.Clickable
	views                                        widget.Clickable // opens the four-view window
	modes                                        [len(modeButtons)]widget.Clickable
	gestures                                     [len(gestureButtons)]widget.Clickable
	tabs                                         [tabCount]widget.Clickable
	tab                                          int

	nudges [calib.Joints][3][4]widget.Clickable // joint, value field, -coarse -fine +fine +coarse
	flips  [calib.Joints]widget.Clickable
	cards  [calib.Joints]widget.Clickable
	sel    selection

	editorList, logList, framesList, diffList widget.List

	exp            exportModel
	checkoutClicks []widget.Clickable
	prepare        widget.Clickable
	write          widget.Clickable
	cancel         widget.Clickable
	copyText       widget.Clickable
	copied         time.Time

	notice   string
	noticeAt time.Time

	space keyLatch // the e-stop key
	// The mode, clear-fault and gesture keys: one action per physical
	// press, as for Space (the nudge keys repeat on purpose).
	actionKeys map[key.Name]*keyLatch
	// The drive keys held now, by name; the session gets the set on every
	// change. Keys down while the app was not in control are suppressed
	// until released: their key repeat must not count as a fresh press.
	charts chartsState

	held       map[key.Name]bool
	suppressed map[key.Name]bool
	unfocused  bool // the window has lost the keyboard focus
}

// Run shows the calibrator window and exits the process when it closes,
// after releasing control and closing the link. It never returns.
func Run(env Env) {
	env.Selection = new(atomic.Int32)
	env.OpenViews = func() { openViews(env, runViews) }
	w, err := newWindow(env)
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-calibrator:", err)
		os.Exit(1)
	}
	go func() {
		win := new(app.Window)
		win.Option(app.Title("Spooder calibrator"), app.Size(unit.Dp(1360), unit.Dp(900)), app.MinSize(unit.Dp(1000), unit.Dp(640)))
		err := w.loop(win)
		env.Session.Shutdown()
		env.Link.Close()
		if err != nil {
			fmt.Fprintln(os.Stderr, "spooder-calibrator:", err)
			os.Exit(1)
		}
		os.Exit(0)
	}()
	app.Main()
}

func newWindow(env Env) (*window, error) {
	faces, err := fonts.Collection()
	if err != nil {
		return nil, err
	}
	th := material.NewTheme()
	th.Shaper = text.NewShaper(text.NoSystemFonts(), text.WithCollection(faces))
	th.Face = fonts.Text
	th.TextSize = sizeText
	w := &window{env: env, th: th, looks: make(chan appearance.Settings, 4), portsCh: make(chan []link.PortInfo, 1), portSel: -1}
	w.charts.window = defaultChartWindow
	w.charts.list.Axis = layout.Vertical
	w.editorList.Axis = layout.Vertical
	w.logList.Axis = layout.Vertical
	w.logList.ScrollToEnd = true
	w.framesList.Axis = layout.Vertical
	w.framesList.ScrollToEnd = true
	w.diffList.Axis = layout.Vertical
	w.exp = exportModel{candidates: env.Checkouts, save: func(dir string) error {
		if env.ConfigDir == "" {
			return nil
		}
		return export.SaveLastDir(env.ConfigDir, dir)
	}}
	w.exp.preferred()
	w.checkoutClicks = make([]widget.Clickable, len(env.Checkouts))
	w.setAppearance(env.Appearance)
	w.snap = env.Session.Snapshot()
	return w, nil
}

func (w *window) setAppearance(s appearance.Settings) {
	w.look = s
	w.pal = NewPalette(s)
	w.pal.apply(w.th)
}

// receive applies what background goroutines delivered.
func (w *window) receive() {
	for {
		select {
		case s := <-w.looks:
			if s != w.look {
				w.setAppearance(s)
			}
		case ports := <-w.portsCh:
			w.setPorts(ports)
		default:
			return
		}
	}
}

// setPorts replaces the port list, keeping the selection on the same
// device, or preselecting the --port port, else a Servo 2040.
func (w *window) setPorts(ports []link.PortInfo) {
	var selected *link.PortInfo
	if w.portSel >= 0 && w.portSel < len(w.ports) {
		p := w.ports[w.portSel]
		selected = &p
	}
	if slices.Equal(ports, w.ports) {
		return // keep the buttons, and any press in progress on them
	}
	w.ports = ports
	w.portClicks = make([]widget.Clickable, len(ports))
	w.portSel = -1
	// A preselection is made again, so the --port path wins once it
	// appears; the operator's own pick, or the connected device, is kept.
	if selected != nil && (w.picked || w.device != nil) {
		for i, p := range ports {
			if link.SameDevice(*selected, p) {
				w.portSel = i
				break
			}
		}
	}
	if w.portSel < 0 && w.env.Port != "" {
		for i, p := range ports {
			if p.Name == w.env.Port {
				w.portSel = i
				break
			}
		}
	}
	if w.portSel < 0 {
		w.portSel = link.Preselect(ports)
	}
}

func (w *window) loop(win *app.Window) error {
	w.win = win
	if w.env.Redraw != nil {
		w.env.Redraw.win.Store(win)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	appearance.Watch(ctx, func(s appearance.Settings) {
		w.looks <- s
		win.Invalidate()
	})
	go w.watchPorts(ctx, win)
	var ops op.Ops
	for {
		switch e := win.Event().(type) {
		case app.DestroyEvent:
			return e.Err
		case app.ConfigEvent:
			w.focusChanged(e.Config.Focused)
		case app.FrameEvent:
			gtx := app.NewContext(&ops, e)
			w.frame(gtx)
			e.Frame(gtx.Ops)
		}
	}
}

// focusChanged follows the window focus. Losing it forgets a held Space,
// whose release then goes elsewhere. Windows has no appearance change
// notification, so the appearance is read again on regaining it (as bopen
// does).
func (w *window) focusChanged(focused bool) {
	w.unfocused = !focused
	if !focused {
		w.space.reset()
		for _, l := range w.actionKeys {
			l.reset()
		}
		// No key held stays applied while the keyboard reports elsewhere.
		// Their releases may go elsewhere too, so a key held now needs its
		// release and a fresh press before it drives again.
		for name, down := range w.held {
			if down {
				w.suppress(name)
			}
		}
		w.held = nil
		_ = w.env.Session.SetDrive(session.Drive{})
		return
	}
	select {
	case w.looks <- appearance.Read():
	default:
	}
}

// watchPorts reads the port list now and every portRefresh.
func (w *window) watchPorts(ctx context.Context, win *app.Window) {
	t := time.NewTicker(portRefresh)
	defer t.Stop()
	for {
		if ports, err := w.env.Ports(); err == nil {
			select {
			case <-w.portsCh: // drop an unread older list
			default:
			}
			w.portsCh <- ports
			win.Invalidate()
		}
		select {
		case <-ctx.Done():
			return
		case <-t.C:
		}
	}
}

// frame handles input and lays out one frame.
func (w *window) frame(gtx layout.Context) {
	w.receive()
	w.snap = w.env.Session.Snapshot()
	w.handle(gtx)
	w.snap = w.env.Session.Snapshot()
	w.layout(gtx)
	if w.env.Selection != nil {
		w.env.Selection.Store(int32(w.sel.joint)) // for the four-view window
	}
	if w.snap.Link.Status != link.Disconnected {
		// Keep ages and the "not current" marker moving.
		gtx.Execute(op.InvalidateCmd{At: gtx.Now.Add(250 * time.Millisecond)})
	}
}

func (w *window) say(gtx layout.Context, err error) {
	if err != nil {
		w.notice, w.noticeAt = err.Error(), gtx.Now
	}
}

// actionKey handles the mode, clear-fault and gesture keys, once per
// physical press; it reports whether ke was one of them.
func (w *window) actionKey(gtx layout.Context, ke key.Event) bool {
	s := w.env.Session
	var act func() error
	for i, k := range modeKeys {
		if ke.Name == k {
			m := modeButtons[i]
			act = func() error { return s.RequestMode(m) }
		}
	}
	if ke.Name == clearFaultKey {
		act = s.ClearFault
	}
	for _, k := range settingKeys {
		if ke.Name == k.name {
			act = k.action(s)
		}
	}
	for i, k := range gestureKeys {
		if ke.Name == k {
			g := gestureButtons[i].id
			act = func() error { return s.RequestGesture(g) }
		}
	}
	if act == nil {
		return false
	}
	if w.actionKeys == nil {
		w.actionKeys = map[key.Name]*keyLatch{}
	}
	l := w.actionKeys[ke.Name]
	if l == nil {
		l = &keyLatch{}
		w.actionKeys[ke.Name] = l
	}
	if l.press(ke.State) {
		w.say(gtx, act())
	}
	return true
}

// keyFilters are the window's shortcuts. Tab is taken so that keyboard
// focus never moves onto a button, which would take Space for itself:
// Space is always the e-stop.
var keyFilters = []event.Filter{
	// Every modifier, so that each release reaches the e-stop latch.
	key.Filter{Name: key.NameSpace, Optional: key.ModShift | key.ModCtrl | key.ModAlt | key.ModSuper | key.ModCommand},
	key.Filter{Name: key.NameTab, Optional: key.ModShift},
	key.Filter{Name: key.NameLeftArrow}, key.Filter{Name: key.NameRightArrow},
	key.Filter{Name: key.NameUpArrow}, key.Filter{Name: key.NameDownArrow},
	key.Filter{Name: "=", Optional: key.ModShift}, key.Filter{Name: "+", Optional: key.ModShift},
	key.Filter{Name: "-", Optional: key.ModShift}, key.Filter{Name: "_", Optional: key.ModShift},
	key.Filter{Name: "*", Optional: key.ModShift}, key.Filter{Name: "R", Optional: key.ModShift},
	// The mode buttons, in their order, Clear fault and the gestures: with
	// every modifier, so that a release with a modifier down still reaches
	// the key's latch.
	key.Filter{Name: modeKeys[0], Optional: anyModifier}, key.Filter{Name: modeKeys[1], Optional: anyModifier},
	key.Filter{Name: modeKeys[2], Optional: anyModifier}, key.Filter{Name: clearFaultKey, Optional: anyModifier},
	key.Filter{Name: gestureKeys[0], Optional: anyModifier}, key.Filter{Name: gestureKeys[1], Optional: anyModifier},
	key.Filter{Name: gestureKeys[2], Optional: anyModifier}, key.Filter{Name: gestureKeys[3], Optional: anyModifier},
	key.Filter{Name: gestureKeys[4], Optional: anyModifier},
	// Driving, with every modifier, so that each release reaches the held
	// set (a release with Shift down would otherwise be dropped and leave
	// the robot walking); and the walking settings.
	key.Filter{Name: "W", Optional: anyModifier}, key.Filter{Name: "S", Optional: anyModifier},
	key.Filter{Name: "A", Optional: anyModifier}, key.Filter{Name: "D", Optional: anyModifier},
	key.Filter{Name: "Q", Optional: anyModifier}, key.Filter{Name: "E", Optional: anyModifier},
	key.Filter{Name: "Z", Optional: anyModifier}, key.Filter{Name: "X", Optional: anyModifier},
	key.Filter{Name: "C", Optional: anyModifier}, key.Filter{Name: "V", Optional: anyModifier},
	key.Filter{Name: "G", Optional: anyModifier}, key.Filter{Name: "T", Optional: anyModifier},
}

const anyModifier = key.ModShift | key.ModCtrl | key.ModAlt | key.ModSuper | key.ModCommand

// driveKeys are held to drive, as the bridge's car layout: W and S walk, A
// and D steer, Q and E step sideways. They act only in control, in Stand
// or Walk.
var driveKeys = map[key.Name]func(d *session.Drive) *bool{
	"W": func(d *session.Drive) *bool { return &d.Forward },
	"S": func(d *session.Drive) *bool { return &d.Back },
	"A": func(d *session.Drive) *bool { return &d.SteerLeft },
	"D": func(d *session.Drive) *bool { return &d.SteerRight },
	"Q": func(d *session.Drive) *bool { return &d.StepLeft },
	"E": func(d *session.Drive) *bool { return &d.StepRight },
}

func (w *window) suppress(name key.Name) {
	if w.suppressed == nil {
		w.suppressed = map[key.Name]bool{}
	}
	w.suppressed[name] = true
}

// settingKeys change the walking settings, once per press, only in
// control.
var settingKeys = [...]struct {
	name   key.Name
	action func(s *session.Session) func() error
}{
	{"Z", func(s *session.Session) func() error { return func() error { return s.StepSpeed(-1) } }},
	{"X", func(s *session.Session) func() error { return func() error { return s.StepSpeed(1) } }},
	{"C", func(s *session.Session) func() error { return func() error { return s.StepHeight(-1) } }},
	{"V", func(s *session.Session) func() error { return func() error { return s.StepHeight(1) } }},
	{"G", func(s *session.Session) func() error { return s.NextGait }},
	{"T", func(s *session.Session) func() error { return s.ToggleStride }},
}

// driveKey follows a drive key's press or release; it reports whether ke
// was one. A refused press (not in control) is said once per press.
func (w *window) driveKey(gtx layout.Context, ke key.Event) bool {
	if _, ok := driveKeys[ke.Name]; !ok {
		return false
	}
	down := ke.State == key.Press
	if !down && w.suppressed[ke.Name] {
		delete(w.suppressed, ke.Name)
		return true
	}
	if w.suppressed[ke.Name] {
		return true // still held from before control: its key repeat
	}
	if !w.snap.Control || w.unfocused {
		// Pressed without control, or handled after the focus went (queued
		// before): it needs a fresh press once in control and focused.
		if down {
			w.suppress(ke.Name)
			w.say(gtx, session.ErrNotInControl)
		}
		return true
	}
	if w.held[ke.Name] == down {
		return true // key repeat
	}
	if w.held == nil {
		w.held = map[key.Name]bool{}
	}
	w.held[ke.Name] = down
	var d session.Drive
	for name, isDown := range w.held {
		if isDown {
			*driveKeys[name](&d) = true
		}
	}
	err := w.env.Session.SetDrive(d)
	if down {
		w.say(gtx, err)
	}
	return true
}

// gestureButtons are the gestures, in protocol id order, with their labels;
// gestureKeys their shortcuts. They act only in control.
var gestureButtons = [...]struct {
	id    uint8
	label string
}{
	{protocol.GestureSayHi, "Hi"},
	{protocol.GestureWiggle, "Wiggle"},
	{protocol.GestureDance, "Dance"},
	{protocol.GestureAttack, "Attack"},
	{protocol.GestureBall, "Ball"},
}

var gestureKeys = [len(gestureButtons)]key.Name{"4", "5", "6", "7", "8"}

// modeKeys are the mode buttons' shortcuts, in modeButtons order;
// clearFaultKey is Clear fault's. Like the buttons, they act only in control.
var modeKeys = [len(modeButtons)]key.Name{"1", "2", "3"}

const clearFaultKey key.Name = "F"

// handle processes shortcuts and clicks.
func (w *window) handle(gtx layout.Context) {
	s := w.env.Session
	if !w.snap.Control {
		// Control released or the link lost while keys were held: they need
		// a fresh press.
		for name, down := range w.held {
			if down {
				w.suppress(name)
			}
		}
		w.held = nil
	}
	for {
		ev, ok := gtx.Event(keyFilters...)
		if !ok {
			break
		}
		ke, ok := ev.(key.Event)
		if !ok {
			continue
		}
		if ke.Name == key.NameSpace {
			if w.space.press(ke.State) {
				s.ToggleEmergencyStop()
			}
			continue
		}
		if w.driveKey(gtx, ke) || w.actionKey(gtx, ke) {
			continue
		}
		if ke.State != key.Press {
			continue
		}
		switch ke.Name {
		case key.NameTab:
		case key.NameLeftArrow, key.NameRightArrow, key.NameUpArrow, key.NameDownArrow:
			w.sel = w.sel.move(ke.Name)
			w.editorList.ScrollTo(w.sel.joint / 3)
		default:
			field, steps, coarse, ok := nudgeKey(ke.Name, ke.Modifiers)
			if !ok {
				break
			}
			f := w.sel.field
			if field != nil {
				f = *field
			}
			w.say(gtx, s.Nudge(w.sel.joint, f, steps, coarse))
		}
	}

	if w.views.Clicked(gtx) && w.env.OpenViews != nil {
		w.env.OpenViews()
	}
	if w.estop.Clicked(gtx) {
		s.ToggleEmergencyStop()
	}
	if w.refresh.Clicked(gtx) {
		if ports, err := w.env.Ports(); err == nil {
			w.setPorts(ports)
		} else {
			w.say(gtx, err)
		}
	}
	for i := range w.portClicks {
		if w.portClicks[i].Clicked(gtx) && w.device == nil {
			w.portSel = i
			w.picked = true
		}
	}
	if w.connect.Clicked(gtx) {
		if w.device != nil {
			w.device = nil
			s.ReleaseControl()
			w.env.Link.Disconnect()
		} else if w.portSel >= 0 && w.portSel < len(w.ports) {
			p := w.ports[w.portSel]
			w.device = &p
			w.env.Link.Connect(p)
		}
	}
	if w.control.Clicked(gtx) {
		if w.snap.Control {
			s.ReleaseControl()
		} else {
			w.say(gtx, s.TakeControl())
		}
	}
	for i := range w.modes {
		if w.modes[i].Clicked(gtx) {
			w.say(gtx, s.RequestMode(modeButtons[i]))
		}
	}
	if w.clearFault.Clicked(gtx) {
		w.say(gtx, s.ClearFault())
	}
	for i := range w.gestures {
		if w.gestures[i].Clicked(gtx) {
			w.say(gtx, s.RequestGesture(gestureButtons[i].id))
		}
	}
	for i := range w.tabs {
		if w.tabs[i].Clicked(gtx) {
			w.tab = i
		}
	}
	w.handleEditor(gtx)
	w.handleExport(gtx)
	w.handleCharts(gtx)
}

// nudge steps for the four buttons of a value.
var nudgeSteps = [4]struct {
	steps  int
	coarse bool
}{{-1, true}, {-1, false}, {1, false}, {1, true}}

func (w *window) handleEditor(gtx layout.Context) {
	for j := range w.cards {
		if w.cards[j].Clicked(gtx) {
			w.sel.joint = j
		}
		for f := range w.nudges[j] {
			for b := range w.nudges[j][f] {
				if w.nudges[j][f][b].Clicked(gtx) {
					w.sel = selection{joint: j, field: calib.Field(f)}
					w.say(gtx, w.env.Session.Nudge(j, calib.Field(f), nudgeSteps[b].steps, nudgeSteps[b].coarse))
				}
			}
		}
		if w.flips[j].Clicked(gtx) {
			w.sel = selection{joint: j, field: calib.FieldDirection}
			w.say(gtx, w.env.Session.Nudge(j, calib.FieldDirection, 1, false))
		}
	}
}

func (w *window) handleExport(gtx layout.Context) {
	for i := range w.checkoutClicks {
		if w.checkoutClicks[i].Clicked(gtx) {
			w.exp.choose(i)
		}
	}
	table := w.snap.Table
	if w.prepare.Clicked(gtx) && w.snap.HaveReport {
		w.exp.prepare(table)
	}
	if w.write.Clicked(gtx) {
		w.exp.write(table)
	}
	if w.cancel.Clicked(gtx) {
		w.exp.cancel()
	}
	if w.copyText.Clicked(gtx) && w.snap.HaveReport {
		gtx.Execute(clipboard.WriteCmd{Type: "application/text", Data: io.NopCloser(strings.NewReader(export.Format(table)))})
		w.copied = gtx.Now
	}
}

// layout draws the window.
func (w *window) layout(gtx layout.Context) layout.Dimensions {
	paint.Fill(gtx.Ops, w.pal.Bg)
	return layout.UniformInset(16).Layout(gtx, func(gtx layout.Context) layout.Dimensions {
		return layout.Flex{Axis: layout.Vertical}.Layout(gtx,
			layout.Rigid(w.header),
			layout.Rigid(layout.Spacer{Height: 12}.Layout),
			layout.Rigid(w.banners),
			layout.Rigid(w.telemetryStrip),
			layout.Rigid(layout.Spacer{Height: 12}.Layout),
			layout.Flexed(1, func(gtx layout.Context) layout.Dimensions {
				return layout.Flex{}.Layout(gtx,
					layout.Flexed(0.62, w.editor),
					layout.Rigid(layout.Spacer{Width: 12}.Layout),
					layout.Flexed(0.38, w.sidePanes),
				)
			}),
		)
	})
}
