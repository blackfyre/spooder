package session

import (
	"bufio"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/blackfyre/spooder/desktop/internal/protocol"
)

// csvColumns name the CSV export's columns, with their units.
var csvColumns = []string{
	"time_s", "body_mode", "gait", "gesture", "source", "bridge_link_up", "bridge_gamepad", "outputs_on", "faults",
	"voltage_v", "current_a",
	"robot_link_ok_per_s", "robot_link_bad_per_s", "bridge_link_ok_per_s", "bridge_link_bad_per_s",
	"host_link_ok_per_s", "host_link_bad_per_s",
	"average_current_a", "motion_share_pct", "backstop_ticks", "tick_max_ms", "tick_mean_ms",
	"trip_current_a", "sustained_current_a", "min_v", "max_v", "brownout_v", "budget_deg_per_s", "rated_deg_per_s",
	"coxa_fastest_deg_per_s", "femur_fastest_deg_per_s", "tibia_fastest_deg_per_s",
}

func number(v float64, decimals int) string {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return ""
	}
	return strconv.FormatFloat(v, 'f', decimals, 64)
}

// WriteCSV writes the history as CSV: a header row, then one row per
// telemetry sample, with the latest diagnostics and pose speeds at or
// before its time (empty where there are none yet).
func (h History) WriteCSV(w io.Writer) error {
	b := bufio.NewWriter(w)
	for i, c := range csvColumns {
		if i > 0 {
			b.WriteByte(',')
		}
		b.WriteString(c)
	}
	b.WriteByte('\n')
	d, p := -1, -1
	for i, t := range h.Telemetry {
		for d+1 < len(h.Diagnostics) && !h.Diagnostics[d+1].At.After(t.At) {
			d++
		}
		for p+1 < len(h.Poses) && !h.Poses[p+1].At.After(t.At) {
			p++
		}
		rates := [6]float64{math.NaN(), math.NaN(), math.NaN(), math.NaN(), math.NaN(), math.NaN()}
		if i > 0 {
			prev := h.Telemetry[i-1]
			dt := t.At.Sub(prev.At)
			rates = [6]float64{
				CountRate(prev.T.ValidFrames, t.T.ValidFrames, dt), CountRate(prev.T.DiscardedFrames, t.T.DiscardedFrames, dt),
				CountRate(prev.T.BridgeValidFrames, t.T.BridgeValidFrames, dt), CountRate(prev.T.BridgeDiscardedFrames, t.T.BridgeDiscardedFrames, dt),
				HostRate(prev.Host.Valid, t.Host.Valid, dt), HostRate(prev.Host.Discarded, t.Host.Discarded, dt),
			}
		}
		flag := func(on bool) string {
			if on {
				return "1"
			}
			return "0"
		}
		gesture := ""
		if t.T.Gesture != protocol.GestureNone {
			gesture = GestureName(t.T.Gesture)
		}
		row := []string{
			number(t.At.Sub(h.Start).Seconds(), 3), BodyMode(t.T.BodyMode).String(), GaitName(t.T.Gait),
			gesture, SourceFrom(t.T.Flags).String(),
			flag(t.T.Flags&protocol.TelemetryBridgeLinkUp != 0), flag(t.T.Flags&protocol.TelemetryBridgeGamepad != 0),
			flag(t.T.OutputsOn), strconv.Itoa(int(t.T.Faults)),
			number(float64(t.T.VoltageMv)/1000, 3), number(float64(t.T.CurrentMa)/1000, 3),
		}
		for _, r := range rates {
			row = append(row, number(r, 1))
		}
		if d >= 0 {
			x := h.Diagnostics[d].D
			row = append(row,
				number(float64(x.AverageCurrentMa)/1000, 3), number(float64(x.MotionSharePermille)/10, 1),
				strconv.Itoa(int(x.BackstopTicks)), number(float64(x.TickMaxUs)/1000, 3), number(float64(x.TickMeanUs)/1000, 3),
				number(float64(x.TripCurrentMa)/1000, 3), number(float64(x.SustainedCurrentMa)/1000, 3),
				number(float64(x.MinMv)/1000, 3), number(float64(x.MaxMv)/1000, 3), number(float64(x.BrownoutMv)/1000, 3),
				strconv.Itoa(int(x.BudgetDegPerS)), strconv.Itoa(int(x.RatedDegPerS)))
		} else {
			for k := 0; k < 12; k++ {
				row = append(row, "")
			}
		}
		for k := 0; k < JointKinds; k++ {
			v := math.NaN()
			if p >= 0 {
				v = h.Poses[p].FastestDeg[k]
			}
			row = append(row, number(v, 1))
		}
		for j, c := range row {
			if j > 0 {
				b.WriteByte(',')
			}
			b.WriteString(c)
		}
		b.WriteByte('\n')
	}
	return b.Flush()
}

// SaveCSV writes the history to dir as spooder-<connection start>.csv,
// whole: to a temporary file in dir, then renamed. It returns the path.
func (h History) SaveCSV(dir string) (string, error) {
	name := fmt.Sprintf("spooder-%s.csv", h.Start.Format("20060102-150405"))
	path := filepath.Join(dir, name)
	f, err := os.CreateTemp(dir, ".spooder-*.csv")
	if err != nil {
		return "", err
	}
	tmp := f.Name()
	ok := false
	defer func() {
		if !ok {
			f.Close()
			os.Remove(tmp)
		}
	}()
	if err := h.WriteCSV(f); err != nil {
		return "", err
	}
	if err := f.Sync(); err != nil {
		return "", err
	}
	if err := f.Close(); err != nil {
		return "", err
	}
	if err := os.Rename(tmp, path); err != nil {
		os.Remove(tmp)
		return "", err
	}
	ok = true
	return path, nil
}

// DownloadsDir is where exports go, the first that exists as a directory
// of: $XDG_DOWNLOAD_DIR, the download directory in the XDG user-directory
// file (user-dirs.dirs, as desktops write it, often localised), and
// ~/Downloads; else the home directory.
func DownloadsDir() (string, error) {
	isDir := func(d string) bool {
		st, err := os.Stat(d)
		return d != "" && err == nil && st.IsDir()
	}
	if d := os.Getenv("XDG_DOWNLOAD_DIR"); isDir(d) {
		return d, nil
	}
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	if d := userDirsDownload(home); isDir(d) {
		return d, nil
	}
	if d := filepath.Join(home, "Downloads"); isDir(d) {
		return d, nil
	}
	return home, nil
}

// userDirsDownload reads XDG_DOWNLOAD_DIR from user-dirs.dirs in the XDG
// config directory: a line XDG_DOWNLOAD_DIR="$HOME/..." (or an absolute
// path). Empty when there is none.
func userDirsDownload(home string) string {
	config := os.Getenv("XDG_CONFIG_HOME")
	if config == "" {
		config = filepath.Join(home, ".config")
	}
	data, err := os.ReadFile(filepath.Join(config, "user-dirs.dirs"))
	if err != nil {
		return ""
	}
	for _, line := range strings.Split(string(data), "\n") {
		line = strings.TrimSpace(line)
		v, ok := strings.CutPrefix(line, "XDG_DOWNLOAD_DIR=")
		if !ok {
			continue
		}
		v = strings.Trim(v, `"`)
		if rest, ok := strings.CutPrefix(v, "$HOME"); ok {
			return filepath.Join(home, rest)
		}
		if filepath.IsAbs(v) {
			return v
		}
	}
	return ""
}
