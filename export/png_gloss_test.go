package export_test

import (
	"bytes"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §15's tests for View.Gloss.

func TestPNGGlossAddsASpecularHighlightToALitFace(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 40, 40, units.Millimeters(10))
	matte := topDown(200)
	glossy := matte
	glossy.Gloss = units.Scalar(1)
	off, on := renderPNG(t, body, matte), renderPNG(t, body, glossy)
	// Looking straight down, the top face has the normal (0, 0, 1); the key light is the headlight raised 0.5 and
	// moved 0.35 to the left, (-0.35, 0.5, 1) normalised, and the eye is along +Z. The highlight adds white by
	// the cosine between the normal and the halfway direction, to the 32nd power.
	light := math.Sqrt(0.35*0.35 + 0.5*0.5 + 1)
	lx, ly, lz := -0.35/light, 0.5/light, 1/light
	hx, hy, hz := lx, ly, lz+1
	hn := math.Sqrt(hx*hx + hy*hy + hz*hz)
	want := 255 * math.Pow(hz/hn, 32)
	a, b := off.RGBAAt(100, 100), on.RGBAAt(100, 100)
	require.InDelta(t, want, float64(b.R)-float64(a.R), 1.5)
	require.InDelta(t, want, float64(b.G)-float64(a.G), 1.5)
	require.InDelta(t, want, float64(b.B)-float64(a.B), 1.5)
	require.Greater(t, want, 40.0, "the highlight is visible")
	// Pixels outside the part are the background.
	require.Equal(t, off.RGBAAt(2, 2), on.RGBAAt(2, 2))
	// A half gloss adds half.
	half := matte
	half.Gloss = units.Scalar(0.5)
	h := renderPNG(t, body, half).RGBAAt(100, 100)
	require.InDelta(t, want/2, float64(h.R)-float64(a.R), 1.5)
}

func TestPNGGlossLeavesAFaceTurnedFromTheLightAlone(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	plain := export.IsometricView(200, 200)
	glossy := plain
	glossy.Gloss = units.Scalar(1)
	off, on := renderPNG(t, body, plain), renderPNG(t, body, glossy)
	changed, same := 0, 0
	for y := range 200 {
		for x := range 200 {
			if off.RGBAAt(x, y) == on.RGBAAt(x, y) {
				same++
			} else {
				changed++
				c, d := off.RGBAAt(x, y), on.RGBAAt(x, y)
				require.GreaterOrEqual(t, int(d.R)+int(d.G)+int(d.B), int(c.R)+int(c.G)+int(c.B), "a highlight only brightens")
			}
		}
	}
	require.Positive(t, same, "most of the image, the background and the faces off the highlight, is unchanged")
	require.GreaterOrEqual(t, changed+same, 200*200)
}

func TestPNGGlossKeepsShadowsAtTheAmbientLevel(t *testing.T) {
	t.Parallel()
	bodies := shadowScene(t)
	view := topDown(400)
	view.Shadows = true
	glossy := view
	glossy.Gloss = units.Scalar(1)
	off := renderShadowScene(t, bodies, view)
	on := renderShadowScene(t, bodies, glossy)
	// Find the pixels the shadow darkens, by comparing with a render that casts none: each is the ambient shade
	// with or without a gloss, while a lit pixel of the plate takes the highlight.
	lit := renderShadowScene(t, bodies, topDown(400))
	shadowed, highlighted := 0, 0
	for y := range 400 {
		for x := range 400 {
			if brightness([4]uint8{off.RGBAAt(x, y).R, off.RGBAAt(x, y).G, off.RGBAAt(x, y).B}) <
				brightness([4]uint8{lit.RGBAAt(x, y).R, lit.RGBAAt(x, y).G, lit.RGBAAt(x, y).B}) {
				shadowed++
				require.Equal(t, off.RGBAAt(x, y), on.RGBAAt(x, y))
			}
			if on.RGBAAt(x, y) != off.RGBAAt(x, y) {
				highlighted++
			}
		}
	}
	require.Greater(t, shadowed, 200, "the pillar casts a shadow")
	require.Positive(t, highlighted, "and the lit plate takes the highlight")
}

func TestPNGGlossRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	render := func(g units.Value) error {
		v := topDown(100)
		v.Gloss = g
		return export.PNG(t.Context(), &bytes.Buffer{}, body, units.Millimeters(0.1), v)
	}
	require.NoError(t, render(units.Scalar(1)))
	require.ErrorIs(t, render(units.Scalar(1.5)), decad.ErrDegenerate)
	require.ErrorIs(t, render(units.Scalar(-0.1)), decad.ErrDegenerate)
	require.ErrorIs(t, render(units.Millimeters(1)), decad.ErrUnitKind)
}
