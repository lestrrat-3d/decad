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

// thickenChainShellOf spins the open polyline through pts a full turn about
// the sketch-plane V axis.
func thickenChainShellOf(t *testing.T, pts [][2]float64) (*decad.Document, *decad.Body) {
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
	shell, err := doc.RevolveChain(s, chains[0], thickenMeridianAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return doc, shell
}

// TestThickenChainRevolveCylinder is T165: the meridian at r = 10 gives a
// cylinder shell, and each side spins its own band. Pappus reads the volume
// off the band's first moment: 40 * (12^2 - 10^2)/2 = 880 outward,
// 40 * (10^2 - 8^2)/2 = 720 inward and 40 * (12^2 - 8^2)/2 = 1600 centered.
func TestThickenChainRevolveCylinder(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name      string
		side      decad.ThickenSide
		thickness float64
		piCoeff   float64
		radial    float64
	}{
		{"outward", decad.ThickenPositive, 2, 1760, 12},
		{"inward", decad.ThickenNegative, 2, 1440, 10},
		{"centered 4 mm", decad.ThickenCentered, 4, 3200, 12},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, shell := thickenChainShellOf(t, [][2]float64{{10, 0}, {10, 40}})
			require.Equal(t, decad.BodySheet, shell.Kind())
			solid, err := shell.Thicken(t.Context(), units.Millimeters(tc.thickness),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.Len(t, solid.Faces(), 4)
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, volume.Exactness)
			require.Positive(t, volume.Bound.Base())
			requirePiLinearEnclosed(t, volume, 0, tc.piCoeff)
			decadtest.MeasuresBounds(t, solid,
				r3.NewVec(-tc.radial, 0, -tc.radial),
				r3.NewVec(tc.radial, 40, tc.radial), decadtest.Exactly())
		})
	}
}

// TestThickenChainRevolveReachesAxis is T166: eroding a radius-10 shell by
// 11 mm puts the band across the axis, which the radial gate names exactly.
func TestThickenChainRevolveReachesAxis(t *testing.T) {
	t.Parallel()
	doc, shell := thickenChainShellOf(t, [][2]float64{{10, 0}, {10, 40}})
	_, err := shell.Thicken(t.Context(), units.Millimeters(11),
		decad.WithThickenSide(decad.ThickenNegative))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "reaches the revolve axis")
	require.Contains(t, err.Error(), "-1.000000000")
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, decad.BodySheet, shell.Kind())
}

// TestThickenChainRevolveStep is T167: the outer copy mitres the corner, so
// the band is [10,12]x[0,18] plus [10,20]x[18,20] and its first moment is
// 18*22 + 2*150 = 696.
func TestThickenChainRevolveStep(t *testing.T) {
	t.Parallel()
	_, shell := thickenChainShellOf(t, [][2]float64{{10, 0}, {10, 20}, {20, 20}})
	solid, err := shell.Thicken(t.Context(), units.Millimeters(2))
	require.NoError(t, err)
	require.Len(t, solid.Faces(), 6)
	volume, err := solid.Volume()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, volume, 0, 1392)
	decadtest.MeasuresBounds(t, solid, r3.NewVec(-20, 0, -20), r3.NewVec(20, 20, 20),
		decadtest.Exactly())
}

// TestThickenChainRevolvePoleRefuses is T169: a free end ON the axis makes the
// assembled section touch the axis at one point with two off-axis pieces
// meeting there, which is the pinched meridian no axis-incidence audit on this
// path has cleared.
func TestThickenChainRevolvePoleRefuses(t *testing.T) {
	t.Parallel()
	doc, shell := thickenChainShellOf(t, [][2]float64{{0, 0}, {10, 0}})
	area, err := shell.Area()
	require.NoError(t, err)
	_, err = shell.Thicken(t.Context(), units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "reaches the revolve axis")
	require.Contains(t, err.Error(), "0.000000000")
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, decad.BodySheet, shell.Kind())
	after, err := shell.Area()
	require.NoError(t, err)
	require.Equal(t, area.Value.Base(), after.Value.Base())
}
