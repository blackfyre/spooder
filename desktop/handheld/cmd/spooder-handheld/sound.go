package main

import (
	"encoding/binary"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/veandco/go-sdl2/sdl"

	"github.com/blackfyre/spooder/desktop/handheld/internal/app"
	"github.com/blackfyre/spooder/desktop/handheld/internal/feedback"
)

// sdlPlayer plays the feedback tones on one SDL audio device (gabagool
// initialises SDL's audio): 16-bit mono at feedback.SampleRate, a new tone
// replacing what is still queued.
type sdlPlayer struct{ dev sdl.AudioDeviceID }

func openPlayer() (*sdlPlayer, error) {
	want := sdl.AudioSpec{Freq: feedback.SampleRate, Format: sdl.AUDIO_S16SYS, Channels: 1, Samples: 1024}
	dev, err := sdl.OpenAudioDevice("", false, &want, nil, 0)
	if err != nil {
		return nil, fmt.Errorf("audio: %w", err)
	}
	sdl.PauseAudioDevice(dev, false)
	return &sdlPlayer{dev: dev}, nil
}

func (p *sdlPlayer) Play(pcm []int16) error {
	b := make([]byte, 2*len(pcm))
	for i, v := range pcm {
		binary.NativeEndian.PutUint16(b[2*i:], uint16(v))
	}
	sdl.ClearQueuedAudio(p.dev)
	return sdl.QueueAudio(p.dev, b)
}

func (p *sdlPlayer) close() { sdl.CloseAudioDevice(p.dev) }

// sound is the pak's sound feedback (design D4 of handheld-observations):
// the feedback over the SDL player, and the Select + B toggle kept between
// runs beside the binary. Without an audio device the pak drives silently.
type sound struct {
	fb      *feedback.Feedback
	player  *sdlPlayer
	setting feedback.Setting
	on      bool
}

// startSound opens the audio device and sets a's toggle from the kept
// setting.
func startSound(a *app.App) *sound {
	s := &sound{on: true}
	if exe, err := os.Executable(); err == nil {
		s.setting = feedback.SettingIn(filepath.Dir(exe))
		s.on = s.setting.On()
	}
	a.SetSoundOn(s.on)
	p, err := openPlayer()
	if err != nil {
		fmt.Fprintln(os.Stderr, "spooder-handheld: no sound:", err)
		return s
	}
	s.player, s.fb = p, feedback.New(p)
	return s
}

// update plays what sc shows happened, and keeps a changed toggle.
func (s *sound) update(sc app.Screen, now time.Time) {
	if sc.SoundOn != s.on {
		s.on = sc.SoundOn
		if s.setting.File != "" {
			if err := s.setting.Save(s.on); err != nil {
				fmt.Fprintln(os.Stderr, "spooder-handheld: sound setting:", err)
			}
		}
	}
	if s.fb == nil {
		return
	}
	if _, err := s.fb.Update(feedback.FromScreen(sc, now), s.on); err != nil {
		fmt.Fprintln(os.Stderr, "spooder-handheld: sound:", err)
	}
}

// drainWait is the longest close waits for a queued tone (the leave's) to
// finish.
const drainWait = 500 * time.Millisecond

func (s *sound) close() {
	if s.player == nil {
		return
	}
	for end := time.Now().Add(drainWait); sdl.GetQueuedAudioSize(s.player.dev) > 0 && time.Now().Before(end); {
		sdl.Delay(10)
	}
	s.player.close()
}
