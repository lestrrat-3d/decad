package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// thickenRibbonOf extrudes the open polyline through pts into a 10 mm ribbon.
func thickenRibbonOf(t *testing.T, pts [][2]float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := 0; i+1 < len(points); i++ {
		s.CreateLine(points[i], points[i+1])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	chains := s.Chains()
	require.Len(t, chains, 1)
	doc := decad.New()
	ribbon, err := doc.ExtrudeChain(s, chains[0], decad.Distance{
		D: units.Millimeters(10), Dir: decad.Along,
	})
	require.NoError(t, err)
	return doc, ribbon
}

// TestThickenRibbonStraight is T162: the assembled section is a 40 x 2 mm
// rectangle whichever side receives the material, so all three sides publish
// the same Exact volume and area and differ only in where the box sits.
func TestThickenRibbonStraight(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		side     decad.ThickenSide
		minY     float64
		maxY     float64
		thicknes float64
	}{
		{"positive", decad.ThickenPositive, -2, 0, 2},
		{"negative", decad.ThickenNegative, 0, 2, 2},
		{"centered 2 mm", decad.ThickenCentered, -1, 1, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, ribbon := thickenRibbonOf(t, [][2]float64{{0, 0}, {40, 0}})
			require.Equal(t, decad.BodySheet, ribbon.Kind())
			solid, err := ribbon.Thicken(t.Context(), units.Millimeters(tc.thicknes),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.True(t, solid.IsSolid())
			require.Len(t, solid.Faces(), 6)
			decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(800), decadtest.Exactly())
			decadtest.MeasuresArea(t, solid, units.SquareMillimeters(1000), decadtest.Exactly())
			decadtest.MeasuresBounds(t, solid,
				r3.NewVec(0, tc.minY, 0), r3.NewVec(40, tc.maxY, 10), decadtest.Exactly())
		})
	}
}

// TestThickenRibbonCorner is T163: the outward side grows the quarter disc a
// corner arc adds, and the inward side takes the mitred quarter square.
func TestThickenRibbonCorner(t *testing.T) {
	t.Parallel()

	t.Run("outward", func(t *testing.T) {
		t.Parallel()
		_, ribbon := thickenRibbonOf(t, [][2]float64{{0, 0}, {40, 0}, {40, 30}})
		solid, err := ribbon.Thicken(t.Context(), units.Millimeters(2))
		require.NoError(t, err)
		require.Len(t, solid.Faces(), 9)
		volume, err := solid.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Approximate, volume.Exactness)
		require.Positive(t, volume.Bound.Base())
		requirePiLinearEnclosed(t, volume, 1400, 10)
		decadtest.MeasuresBounds(t, solid, r3.NewVec(0, -2, 0), r3.NewVec(42, 30, 10),
			decadtest.Exactly())
	})

	t.Run("inward", func(t *testing.T) {
		t.Parallel()
		_, ribbon := thickenRibbonOf(t, [][2]float64{{0, 0}, {40, 0}, {40, 30}})
		solid, err := ribbon.Thicken(t.Context(), units.Millimeters(2),
			decad.WithThickenSide(decad.ThickenNegative))
		require.NoError(t, err)
		require.Len(t, solid.Faces(), 8)
		decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(1360), decadtest.Exactly())
		decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(40, 30, 10),
			decadtest.Exactly())
	})
}

// TestThickenRibbonNeck is T164: eroding a 4 mm neck by 3 mm makes the two
// inner copies meet inside the interval, while 1 mm leaves the U-shaped band
// [0,4]x[0,10] less [1,3]x[1,10].
func TestThickenRibbonNeck(t *testing.T) {
	t.Parallel()
	neck := [][2]float64{{0, 10}, {0, 0}, {4, 0}, {4, 10}}

	t.Run("contact refuses", func(t *testing.T) {
		t.Parallel()
		doc, ribbon := thickenRibbonOf(t, neck)
		_, err := ribbon.Thicken(t.Context(), units.Millimeters(3),
			decad.WithThickenSide(decad.ThickenNegative))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "the offset interval contains a nonadjacent line contact")
		require.Len(t, doc.Bodies(), 1)
		require.Equal(t, decad.BodySheet, ribbon.Kind())
	})

	t.Run("clear builds", func(t *testing.T) {
		t.Parallel()
		_, ribbon := thickenRibbonOf(t, neck)
		solid, err := ribbon.Thicken(t.Context(), units.Millimeters(1),
			decad.WithThickenSide(decad.ThickenNegative))
		require.NoError(t, err)
		require.Len(t, solid.Faces(), 10)
		decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(220), decadtest.Exactly())
		decadtest.MeasuresBounds(t, solid, r3.NewVec(0, 0, 0), r3.NewVec(4, 10, 10),
			decadtest.Exactly())
	})
}
