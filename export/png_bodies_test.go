package export_test

import (
	"bytes"
	"context"
	"image/color"
	"image/png"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/png-export-design.md §5's tests for PNGBodies.

func TestPNGBodiesDrawsBothBodiesAndTheGapBetween(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(5))
	b := decadtest.NewBlock(t, doc, 30, 0, 40, 10, units.Millimeters(5))
	var out bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &out, []*decad.Body{a, b}, units.Millimeters(0.1), topDown(400)))
	img, err := png.Decode(&out)
	require.NoError(t, err)
	// The scene spans x 0 to 40 mm and y 0 to 10 mm: 40 mm is 380 px wide with a
	// 10 px margin, so 9.5 px per mm. Block A covers px 10 to 105, the gap 105 to
	// 295 and block B 295 to 390, all across rows 152 to 247.
	face := img.At(50, 200)
	require.NotEqual(t, pngWhite, face)
	require.Equal(t, face, img.At(340, 200), "the second block is drawn in the same scheme")
	require.Equal(t, pngWhite, img.At(200, 200), "the gap between the blocks is background")
	require.Equal(t, pngWhite, img.At(50, 100), "above the blocks is background")
}

func TestPNGBodiesDoesNotDependOnTheOrderOfTheBodies(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	low := decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(2))
	high := decadtest.NewBlock(t, doc, 5, 5, 15, 15, units.Millimeters(8))
	var both bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &both, []*decad.Body{high, low}, units.Millimeters(0.1), topDown(200)))
	var reversed bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &reversed, []*decad.Body{low, high}, units.Millimeters(0.1), topDown(200)))
	require.Equal(t, both.Bytes(), reversed.Bytes(), "the order of the bodies does not change what is in front")
	img, err := png.Decode(&both)
	require.NoError(t, err)
	// Both silhouettes are in the scene: the plate's corner and the tall block.
	require.NotEqual(t, pngWhite, img.At(100, 100))
	require.NotEqual(t, pngWhite, img.At(30, 30))
	require.Equal(t, pngWhite, img.At(3, 3))
}

func TestPNGBodiesOfOneBodyMatchesPNG(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	var single, group bytes.Buffer
	require.NoError(t, export.PNG(t.Context(), &single, body, units.Millimeters(0.1), export.IsometricView(120, 90)))
	require.NoError(t, export.PNGBodies(t.Context(), &group, []*decad.Body{body}, units.Millimeters(0.1), export.IsometricView(120, 90)))
	require.Equal(t, single.Bytes(), group.Bytes())
}

func TestPNGBodiesRefusals(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 4, 3, units.Millimeters(2))
	var nilContext context.Context
	view := topDown(64)
	tol := units.Millimeters(0.1)
	require.ErrorIs(t, export.PNGBodies(nilContext, &bytes.Buffer{}, []*decad.Body{body}, tol, view), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNGBodies(t.Context(), nil, []*decad.Body{body}, tol, view), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNGBodies(t.Context(), &bytes.Buffer{}, nil, tol, view), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNGBodies(t.Context(), &bytes.Buffer{}, []*decad.Body{body, nil}, tol, view), decad.ErrDegenerate)
	require.ErrorIs(t, export.PNGBodies(t.Context(), &bytes.Buffer{}, []*decad.Body{body}, units.Degrees(1), view), decad.ErrUnitKind)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	var out bytes.Buffer
	require.ErrorIs(t, export.PNGBodies(canceled, &out, []*decad.Body{body}, tol, view), context.Canceled)
	require.Zero(t, out.Len(), "a canceled draw writes nothing")
}

// This file is also agent-docs/png-export-design.md §8's tests for View.Colors.

func TestPNGBodiesColoursEachBodyItself(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(5))
	b := decadtest.NewBlock(t, doc, 30, 0, 40, 10, units.Millimeters(5))
	view := topDown(400)
	view.Colors = []color.RGBA{{R: 200, G: 40, B: 40}, {R: 40, G: 200, B: 40, A: 7}}
	var out bytes.Buffer
	require.NoError(t, export.PNGBodies(t.Context(), &out, []*decad.Body{a, b}, units.Millimeters(0.1), view))
	img, err := png.Decode(&out)
	require.NoError(t, err)
	red := color.RGBAModel.Convert(img.At(50, 200)).(color.RGBA)
	green := color.RGBAModel.Convert(img.At(340, 200)).(color.RGBA)
	require.Greater(t, int(red.R), 2*int(red.G), "the first block takes the first colour")
	require.Greater(t, int(green.G), 2*int(green.R), "the second block takes the second colour")
	require.Equal(t, uint8(255), green.A, "the alpha of a colour is ignored")
	require.Equal(t, pngWhite, img.At(200, 200))
}

func TestPNGColoursOneBodyAndAFrontColourIsTheDefault(t *testing.T) {
	t.Parallel()
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(10))
	var plain, tinted, same bytes.Buffer
	require.NoError(t, export.PNG(t.Context(), &plain, body, units.Millimeters(0.1), export.IsometricView(120, 90)))
	view := export.IsometricView(120, 90)
	view.Colors = []color.RGBA{{R: 120, G: 160, B: 200}}
	require.NoError(t, export.PNG(t.Context(), &same, body, units.Millimeters(0.1), view))
	require.Equal(t, plain.Bytes(), same.Bytes(), "the default front colour is the colour the scheme already uses")
	view.Colors = []color.RGBA{{R: 220, G: 30, B: 30}}
	require.NoError(t, export.PNG(t.Context(), &tinted, body, units.Millimeters(0.1), view))
	require.NotEqual(t, plain.Bytes(), tinted.Bytes())
}

func TestPNGColoursRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(5))
	b := decadtest.NewBlock(t, doc, 30, 0, 40, 10, units.Millimeters(5))
	view := topDown(100)
	view.Colors = []color.RGBA{{R: 200}}
	var out bytes.Buffer
	require.ErrorIs(t, export.PNGBodies(t.Context(), &out, []*decad.Body{a, b}, units.Millimeters(0.1), view), decad.ErrDegenerate,
		"one colour for two bodies")
	view.Colors = []color.RGBA{{R: 200}, {G: 200}}
	require.ErrorIs(t, export.PNG(t.Context(), &out, a, units.Millimeters(0.1), view), decad.ErrDegenerate, "two colours for one body")
	require.Zero(t, out.Len(), "a refused image writes nothing")
}
