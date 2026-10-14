package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepChainArcThickenSolidAndReplay(t *testing.T) {
	t.Parallel()
	s, chain := sweepChainArcLine(t, r3.Vec{}, r3.NewVec(4, 0, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())
	solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())
	requireManifold(t, solid)
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, solid, doc.Bodies()[0])

	volume, err := solid.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-12*math.Pi), volume.Bound.Base()+1e-12)
	area, err := solid.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-(30*math.Pi+8)), area.Bound.Base()+1e-12)
	box, err := solid.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, -1, 0), box.Min)
	require.Equal(t, r3.NewVec(10, 0, 5), box.Max)
	for _, point := range faceVertexReadings(faceByRole(t, solid, "capStart")) {
		require.LessOrEqual(t, point.Value.X, 4+point.Bound.Base())
	}
	for _, point := range faceVertexReadings(faceByRole(t, solid, "capEnd")) {
		require.GreaterOrEqual(t, point.Value.X, 6-point.Bound.Base())
	}
	require.NotNil(t, faceByRole(t, solid, "side(0,0,0)"))
	mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	result, err := report.ForBody(solid)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, result.Status)

	motion, err := r3.Translation(r3.NewVec(3, 7, 11))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), solid, decad.WithCopyPlacement(motion))
	require.NoError(t, err)
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(placedVolume.Value.Base()-12*math.Pi),
		placedVolume.Bound.Base()+1e-12)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
	_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

func TestSweepChainArcThickenSides(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		side decad.ThickenSide
		minY float64
		maxY float64
	}{
		{"negative", decad.ThickenNegative, 0, 1},
		{"centered band", decad.ThickenCentered, -0.5, 0.5},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			s, chain := sweepChainArcLine(t, r3.Vec{}, r3.NewVec(4, 0, 0))
			path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
				Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
			})
			require.NoError(t, err)
			sheet, err := decad.Sweep(t.Context(), decad.New(), s, chain, path)
			require.NoError(t, err)
			solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(1),
				decad.WithThickenSide(tc.side))
			require.NoError(t, err)
			volume, err := solid.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-12*math.Pi), volume.Bound.Base()+1e-12)
			box, err := solid.Bounds()
			require.NoError(t, err)
			require.Equal(t, tc.minY, box.Min.Y)
			require.Equal(t, tc.maxY, box.Max.Y)
		})
	}
}

func TestSweepChainArcThickenPositiveAxisSideKeepsCapOrder(t *testing.T) {
	t.Parallel()
	s, chain := sweepChainArcLine(t, r3.NewVec(6, 0, 0), r3.NewVec(10, 0, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	sheet, err := decad.Sweep(t.Context(), decad.New(), s, chain, path)
	require.NoError(t, err)
	solid, err := decad.Thicken(t.Context(), sheet, units.Millimeters(1))
	require.NoError(t, err)
	volume, err := solid.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-12*math.Pi), volume.Bound.Base()+1e-12)
	for _, point := range faceVertexReadings(faceByRole(t, solid, "capStart")) {
		require.GreaterOrEqual(t, point.Value.X, 6-point.Bound.Base())
	}
	for _, point := range faceVertexReadings(faceByRole(t, solid, "capEnd")) {
		require.LessOrEqual(t, point.Value.X, 4+point.Bound.Base())
	}
	mesh, err := solid.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestSweepChainArcThickenNoncardinalRefusesWithoutRetiringSheet(t *testing.T) {
	t.Parallel()
	s, chain := sweepChainArcLine(t, r3.Vec{}, r3.NewVec(4, 0, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(3, 4, 5), End: r3.NewVec(6, 8, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	sheet, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	_, err = decad.Thicken(t.Context(), sheet, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])
}
