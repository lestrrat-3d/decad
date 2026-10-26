package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// compositeThickenPath is a +Z line of length 4 then an XZ quarter turn of
// radius 5 about the axis x=5 along Y, so a section centred on x=0 sits 5 mm
// from the axis and sweeps the arc by Pappus.
func compositeThickenPath(t testing.TB) *decad.Path {
	t.Helper()
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 4)},
		decad.ArcThrough{Through: r3.NewVec(2, 0, 8), End: r3.NewVec(5, 0, 9)})
	require.NoError(t, err)
	return path
}

func TestThickenCompositeSweepSheetBuildsExactSolid(t *testing.T) {
	for _, tc := range []struct {
		name string
		side decad.ThickenSide
		mm   float64
		area float64 // section area of the thickened annulus; an outward offset of the
		// square rounds its corners
	}{
		{"outward", decad.ThickenPositive, 1, 8 + math.Pi},
		{"inward", decad.ThickenNegative, 0.5, 4 - 1},
		{"both sides", decad.ThickenCentered, 1, 7 + math.Pi/4},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, profile := orthogonalSweepProfile(t)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, profile, compositeThickenPath(t),
				decad.WithSurfaceResult())
			require.NoError(t, err)
			require.Equal(t, decad.BodySheet, sheet.Kind())

			solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(tc.mm),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			require.Equal(t, decad.BodySolid, solid.Kind())
			require.True(t, solid.IsSolid())
			require.Len(t, solid.Lumps(), 1)
			require.Len(t, doc.Bodies(), 1)
			require.Same(t, solid, doc.Bodies()[0])
			_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(tc.mm))
			require.ErrorIs(t, err, decad.ErrRetiredBody)
			for _, edge := range solid.Edges() {
				require.Len(t, edge.Faces(), 2)
			}

			want := tc.area * (4 + 5*math.Pi/2)
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-12)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
			decadtest.MeasuresVolume(t, solid, units.CubicMillimeters(want))
		})
	}
}

// A section thick enough to reach the arc axis folds there. The thickened
// section goes back through the span gates, which refuse it.
func TestThickenCompositeSweepSheetRefusesFoldingSection(t *testing.T) {
	t.Parallel()
	s, profile := orthogonalSweepProfile(t)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, profile, compositeThickenPath(t),
		decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(5))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])
}
