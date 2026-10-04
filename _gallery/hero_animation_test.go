package main

import (
	"bytes"
	"testing"
	"time"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

func TestHeroAnimationFrames(t *testing.T) {
	require.Equal(t, 10*time.Second, heroExitStart-heroHoldStart)
	ctx := t.Context()
	ch, err := heroAnimationScript().Channels(0)
	require.NoError(t, err)
	take, err := wordmarkTake(ctx, ch)
	require.NoError(t, err)
	style := take.Style
	style.Width, style.Height = 320, 180
	clip, err := kinetograph.NewClip(take.Scene, 16, heroAnimationLength)
	require.NoError(t, err)
	r, err := render.New(ctx, clip, style)
	require.NoError(t, err)
	frame := func(at time.Duration) []byte {
		t.Helper()
		i := int(at * 16 / time.Second)
		require.Equal(t, at, clip.FrameTime(i))
		img, err := r.Frame(ctx, i)
		require.NoError(t, err)
		return img.Pix
	}
	empty := frame(0)
	assembled := frame(2 * time.Second)
	lit := frame(3 * time.Second)
	holdStart := frame(heroHoldStart)
	holdEnd := frame(heroExitStart - time.Second/16)
	exiting := frame(15 * time.Second)
	end := frame(heroAnimationLength - time.Second/16)
	require.False(t, bytes.Equal(empty, assembled), "the letters enter")
	require.False(t, bytes.Equal(assembled, lit), "the lamp lights the letters")
	require.True(t, bytes.Equal(holdStart, holdEnd), "the finished logo stays still")
	require.False(t, bytes.Equal(holdEnd, exiting), "the letters leave")
	require.True(t, bytes.Equal(empty, end), "the loop returns to its first pose")
}

func TestHeroLampSweeps(t *testing.T) {
	ctx := t.Context()
	ch, err := heroAnimationScript().Channels(0)
	require.NoError(t, err)
	take := func() *Take {
		tk, err := wordmarkTake(ctx, ch)
		require.NoError(t, err)
		return tk
	}
	counts, xs := lampSweep(t, take, wordLamp, heroAnimationLength,
		2800*time.Millisecond, 3200*time.Millisecond, 3600*time.Millisecond)
	for i, count := range counts {
		require.Greater(t, count, testWidth*testHeight/200, "lit pixels at sample %d", i)
		if i > 0 {
			require.Greater(t, xs[i]-xs[i-1], 50.0, "lit pixels move right at sample %d", i)
		}
	}
}
