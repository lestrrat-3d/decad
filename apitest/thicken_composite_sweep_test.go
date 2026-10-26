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

// TestThickenCompositeSweepSheetBuildsExactSolid is T157b's build half.
func TestThickenCompositeSweepSheetBuildsExactSolid(t *testing.T) {
	for _, tc := range []struct {
		name string
		side decad.ThickenSide
		mm   float64
		// area is the thickened annulus's section area. An outward offset of
		// the square rounds its corners.
		area float64
	}{
		{"outward offset", decad.ThickenPositive, 1, 8 + math.Pi},
		{"inward offset", decad.ThickenNegative, 0.5, 4 - 1},
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
			require.NotZero(t, volume.Value.Base())

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

// TestThickenCompositeSweepSheetRefusesAxisContact is T157b's radial half: a
// thickness whose offset touches (4 mm) or crosses (5 mm) the arc span's axis
// refuses with the one-span arc sheet's error class and leaves the sheet live.
func TestThickenCompositeSweepSheetRefusesAxisContact(t *testing.T) {
	for _, tc := range []struct {
		name string
		mm   float64
	}{
		{"touches the axis", 4},
		{"crosses the axis", 5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, profile := orthogonalSweepProfile(t)
			doc := decad.New()
			sheet, err := decad.Sweep(t.Context(), doc, s, profile, compositeThickenPath(t),
				decad.WithSurfaceResult())
			require.NoError(t, err)
			_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(tc.mm))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Contains(t, err.Error(), "reaches the revolve axis")
			require.Len(t, doc.Bodies(), 1)
			require.Same(t, sheet, doc.Bodies()[0])
		})
	}
}

// TestThickenCompositeSweepSheetKeepsR24 is T157b's refusal half: a holed
// profile and a placed sheet stay R24 and remain live.
func TestThickenCompositeSweepSheetKeepsR24(t *testing.T) {
	t.Parallel()
	t.Run("holed profile", func(t *testing.T) {
		t.Parallel()
		s, profile := orthogonalSweepProfileWithHole(t)
		doc := decad.New()
		sheet, err := decad.Sweep(t.Context(), doc, s, profile, compositeThickenPath(t),
			decad.WithSurfaceResult())
		require.NoError(t, err)
		_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(0.2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Same(t, sheet, doc.Bodies()[0])
	})
	t.Run("placed sheet", func(t *testing.T) {
		t.Parallel()
		s, profile := orthogonalSweepProfile(t)
		doc := decad.New()
		sheet, err := decad.Sweep(t.Context(), doc, s, profile, compositeThickenPath(t),
			decad.WithSurfaceResult())
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(3, 2, 1))
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), sheet, move)
		require.NoError(t, err)
		_, err = decad.Thicken(t.Context(), placed, units.Millimeters(0.2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, decad.BodySheet, placed.Kind())
	})
}
