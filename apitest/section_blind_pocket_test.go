package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
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
