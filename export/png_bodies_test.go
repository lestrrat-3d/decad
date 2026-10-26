package export_test

import (
	"bytes"
	"context"
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
