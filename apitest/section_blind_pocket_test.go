package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestSectionBlindPocketFeedsExtrude cuts a real 4 x 4 x 4 mm blind pocket from
// a 10 mm cube, sections it inside each slab and extrudes the result.
func TestSectionBlindPocketFeedsExtrude(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocketed, err := decad.Cut(t.Context(), cube, tool)
	require.NoError(t, err)

	for _, tc := range []struct {
		z, area, holes float64
	}{{z: 8, area: 84, holes: 1}, {z: 3, area: 100, holes: 0}} {
		frame, frameErr := r3.NewFrame(r3.Vec{Z: tc.z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, frameErr)
		section, profiles, sectionErr := pocketed.Section(t.Context(), frame)
		require.NoError(t, sectionErr, "z=%g", tc.z)
		require.Len(t, profiles, 1)
		require.Len(t, profiles[0].Holes, int(tc.holes))
		for _, hole := range profiles[0].Holes {
			xlo, xhi, ylo, yhi := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
			for _, seg := range hole {
				line, isLine := seg.Entity.(*sketch.Line)
				require.True(t, isLine)
				for _, pt := range []*sketch.Point{line.Start, line.End} {
					xlo, xhi = math.Min(xlo, pt.X()), math.Max(xhi, pt.X())
					ylo, yhi = math.Min(ylo, pt.Y()), math.Max(yhi, pt.Y())
				}
			}
			require.Equal(t, []float64{3, 7, 3, 7}, []float64{xlo, xhi, ylo, yhi})
		}
		reading, measureErr := decad.MeasureProfile(section, profiles[0])
		require.NoError(t, measureErr)
		area, areaErr := reading.Area()
		require.NoError(t, areaErr)
		require.LessOrEqual(t, math.Abs(area.Value.Base()-tc.area), area.Bound.Base())
		require.LessOrEqual(t, area.Bound.Base(), 1e-9)

		child, extrudeErr := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
			decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
		require.NoError(t, extrudeErr)
		volume, volumeErr := child.Volume()
		require.NoError(t, volumeErr)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-2*tc.area), volume.Bound.Base())
		mesh, meshErr := child.Tessellate(t.Context(), units.Millimeters(0.1),
			decad.WithVerification(decad.VerifyAll))
		require.NoError(t, meshErr)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	}
}

// TestSectionBlindPocketRefusesLevelsOffSlabInteriors keeps the pocket floor,
// the bottom face and the top face out of the admitted levels.
func TestSectionBlindPocketRefusesLevelsOffSlabInteriors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocketed, err := decad.Cut(t.Context(), cube, tool)
	require.NoError(t, err)
	for _, z := range []float64{0, 6, 10, 12} {
		frame, frameErr := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, frameErr)
		_, _, sectionErr := pocketed.Section(t.Context(), frame)
		require.ErrorIs(t, sectionErr, decad.ErrUnsupported, "z=%g", z)
	}
}

func blindPocketBody(t *testing.T) *decad.Body {
	t.Helper()
	doc := decad.New()
	cube := boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10)
	tool := boxBodyAtZ(t, doc, 3, 3, 7, 7, 6, 4)
	pocketed, err := decad.Cut(t.Context(), cube, tool)
	require.NoError(t, err)
	return pocketed
}

// TestSectionStackedPrismGateRefusals reaches the stack gates, not the base
// "exact straight prism" refusal, and checks which gate fired.
func TestSectionStackedPrismGateRefusals(t *testing.T) {
	t.Parallel()
	level := func(z float64) r3.Frame {
		f, err := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		return f
	}
	t.Run("pocket floor", func(t *testing.T) {
		t.Parallel()
		_, _, err := blindPocketBody(t).Section(t.Context(), level(6))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "outside slab stack interiors")
	})
	t.Run("reversed frame", func(t *testing.T) {
		t.Parallel()
		f, err := r3.NewFrame(r3.Vec{Z: 8}, r3.Vec{X: -1}, r3.Vec{Y: -1})
		require.NoError(t, err)
		_, _, err = blindPocketBody(t).Section(t.Context(), f)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "outside slab stack interiors")
	})
	t.Run("placed body", func(t *testing.T) {
		t.Parallel()
		move, err := r3.Translation(r3.Vec{X: 20, Y: 30, Z: 1})
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), blindPocketBody(t), move)
		require.NoError(t, err)
		f, err := r3.NewFrame(r3.Vec{X: 20, Y: 30, Z: 9}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		section, profiles, err := placed.Section(t.Context(), f)
		require.NoError(t, err)
		require.Len(t, profiles, 1)
		require.Len(t, profiles[0].Holes, 1)
		reading, err := decad.MeasureProfile(section, profiles[0])
		require.NoError(t, err)
		area, err := reading.Area()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(area.Value.Base()-84), area.Bound.Base())
	})
}
