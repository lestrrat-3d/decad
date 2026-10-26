package glyphs_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/glyphs"
	"github.com/stretchr/testify/require"
)

func TestLayoutOfAnIsOneClosedPolygonOnTheBaseline(t *testing.T) {
	t.Parallel()
	got, err := glyphs.Layout(glyphs.DefaultFont(), "I", 20, 0.01)
	require.NoError(t, err)
	require.Len(t, got.Contours, 1)
	lowest, highest := math.Inf(1), math.Inf(-1)
	for _, p := range got.Contours[0] {
		lowest, highest = math.Min(lowest, p[1]), math.Max(highest, p[1])
	}
	require.InDelta(t, 0, lowest, 1e-9, "a capital I rests on the baseline")
	require.Greater(t, highest, 10.0, "a capital of a 20 mm em is more than 10 mm tall")
	require.Less(t, highest, 20.0)
	require.Len(t, got.Advances, 2, "one advance for the rune and one for the line end")
	require.Equal(t, 0.0, got.Advances[1])
}

func TestLayoutFlattensCurvesFinerWithASmallerTolerance(t *testing.T) {
	t.Parallel()
	coarse, err := glyphs.Layout(glyphs.DefaultFont(), "O", 20, 0.5)
	require.NoError(t, err)
	fine, err := glyphs.Layout(glyphs.DefaultFont(), "O", 20, 0.005)
	require.NoError(t, err)
	count := func(r glyphs.Result) int {
		n := 0
		for _, c := range r.Contours {
			n += len(c)
		}
		return n
	}
	require.Len(t, coarse.Contours, 2, "the O is a ring: its outside and its counter")
	require.Len(t, fine.Contours, 2)
	require.Greater(t, count(fine), 3*count(coarse))
	// Every polygon vertex of the fine outline lies on the glyph's ring: the outer
	// contour's vertices are the same distance from its centre to within the glyph's
	// own eccentricity, so the largest radial spread is far below the glyph's size.
	outer := fine.Contours[0]
	var cx, cy float64
	for _, p := range outer {
		cx += p[0]
		cy += p[1]
	}
	cx /= float64(len(outer))
	cy /= float64(len(outer))
	require.Greater(t, math.Hypot(outer[0][0]-cx, outer[0][1]-cy), 5.0)
}

func TestLayoutRefusals(t *testing.T) {
	t.Parallel()
	font := glyphs.DefaultFont()
	for name, call := range map[string]func() error{
		"zero size":       func() error { _, err := glyphs.Layout(font, "I", 0, 0.01); return err },
		"zero tolerance":  func() error { _, err := glyphs.Layout(font, "I", 10, 0); return err },
		"infinite size":   func() error { _, err := glyphs.Layout(font, "I", math.Inf(1), 0.01); return err },
		"a bad font":      func() error { _, err := glyphs.Layout([]byte("no"), "I", 10, 0.01); return err },
		"a missing glyph": func() error { _, err := glyphs.Layout(font, "\U0001F600", 10, 0.01); return err },
		"only a space":    func() error { _, err := glyphs.Layout(font, " ", 10, 0.01); return err },
	} {
		require.ErrorIs(t, call(), glyphs.ErrDegenerate, name)
	}
}
