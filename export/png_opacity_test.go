package export_test

import (
	"bytes"
	"image/color"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §16's tests for View.Opacity.

// opacityScene is a 40 x 40 x 10 mm block under a 10 x 10 x 2 mm plate that floats 1 mm above its top face, seen from
// above: the block is red, the plate's near face green and its far face, which a translucent plate shows, blue.
func opacityScene(t *testing.T) ([]*decad.Body, export.View) {
	t.Helper()
	doc := decad.New()
	block := decadtest.NewBlock(t, doc, 0, 0, 40, 40, units.Millimeters(10))
	plate := translatedUp(t, decadtest.NewBlock(t, doc, 15, 15, 25, 25, units.Millimeters(2)), 11)
	view := topDown(200)
	view.Colors = []color.RGBA{{R: 200}, {G: 200}}
	back := color.RGBA{B: 200}
	view.BackColor = &back
	return []*decad.Body{block, plate}, view
}

func translatedUp(t *testing.T, b *decad.Body, z float64) *decad.Body {
	t.Helper()
	up, err := r3.Translation(r3.Vec{Z: z})
	require.NoError(t, err)
	m, err := decad.Transform(t.Context(), b, up)
	require.NoError(t, err)
	return m
}

func renderOpacity(t *testing.T, bodies []*decad.Body, view export.View) (func(x, y int) color.RGBA, error) {
	t.Helper()
	var out bytes.Buffer
	if err := export.PNGBodies(t.Context(), &out, bodies, units.Millimeters(0.1), view); err != nil {
		return nil, err
	}
	img, err := png.Decode(&out)
	require.NoError(t, err)
	return func(x, y int) color.RGBA {
		r, g, b, _ := img.At(x, y).RGBA()
		return color.RGBA{R: uint8(r >> 8), G: uint8(g >> 8), B: uint8(b >> 8), A: 255}
	}, nil
}

func TestPNGOpacityBlendsAPlateOverTheBlock(t *testing.T) {
	t.Parallel()
	bodies, view := opacityScene(t)
	solid, err := renderOpacity(t, bodies, view)
	require.NoError(t, err)
	// A solid plate hides the block: the plate's near face is green at the middle, the block's top red at the edge.
	plateFace, blockFace := solid(100, 100), solid(30, 100)
	require.Zero(t, plateFace.R)
	require.Zero(t, blockFace.G)
	level := float64(blockFace.R) / 200 // the headlight's level, the same for every face square to the camera
	require.InDelta(t, 200*level, float64(plateFace.G), 1.5)

	const alpha = 0.5
	view.Opacity = []units.Value{units.Scalar(1), units.Scalar(alpha)}
	glass, err := renderOpacity(t, bodies, view)
	require.NoError(t, err)
	// Over the block a half-clear plate takes half its near face's green, half of its far face's blue (itself half over the
	// block's red): g = α·G, b = (1−α)·α·B, r = (1−α)²·R, each to the rounding of two blends.
	got := glass(100, 100)
	require.InDelta(t, alpha*float64(plateFace.G), float64(got.G), 1.5)
	require.InDelta(t, (1-alpha)*alpha*200*level, float64(got.B), 1.5)
	require.InDelta(t, (1-alpha)*(1-alpha)*float64(blockFace.R), float64(got.R), 1.5)
	require.Equal(t, blockFace, glass(30, 100), "beside the plate the block is as it was")
	require.Equal(t, solid(5, 5), glass(5, 5), "and the background")
}

func TestPNGOpacityOfASolidBodyIsTheSolidImage(t *testing.T) {
	t.Parallel()
	bodies, view := opacityScene(t)
	var plain, solid bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &plain, bodies, units.Millimeters(0.1), view))
	view.Opacity = []units.Value{units.Scalar(1), units.Scalar(1)}
	require.NoError(t, export.PNGBodies(t.Context(), &solid, bodies, units.Millimeters(0.1), view))
	require.Equal(t, plain.Bytes(), solid.Bytes())
}

func TestPNGOpacityRefusals(t *testing.T) {
	t.Parallel()
	bodies, view := opacityScene(t)
	for name, tc := range map[string]struct {
		edit func(*export.View)
		want error
	}{
		"count":      {func(v *export.View) { v.Opacity = []units.Value{units.Scalar(0.5)} }, decad.ErrDegenerate},
		"zero":       {func(v *export.View) { v.Opacity = []units.Value{units.Scalar(1), units.Scalar(0)} }, decad.ErrDegenerate},
		"above one":  {func(v *export.View) { v.Opacity = []units.Value{units.Scalar(1), units.Scalar(1.5)} }, decad.ErrDegenerate},
		"kind":       {func(v *export.View) { v.Opacity = []units.Value{units.Scalar(1), units.Millimeters(1)} }, decad.ErrUnitKind},
		"with shade": {func(v *export.View) { v.Opacity = []units.Value{units.Scalar(1), units.Scalar(0.5)}; v.Shadows = true }, decad.ErrDegenerate},
	} {
		v := view
		tc.edit(&v)
		var out bytes.Buffer
		require.ErrorIs(t, export.PNGBodies(t.Context(), &out, bodies, units.Millimeters(0.1), v), tc.want, name)
		require.Empty(t, out.Bytes(), "%s writes nothing", name)
	}
}
