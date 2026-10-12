package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStraightPrismScaledAtAcceptsPlacedSource(t *testing.T) {
	doc := decad.New()
	prism := boxBody(t, doc, 0, 0, 4, 3, 5)
	turn, err := r3.RotationAround(r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0), units.Radians(0.2))
	require.NoError(t, err)
	placed, err := prism.Placed(t.Context(), turn)
	require.NoError(t, err)
	scaled, err := placed.ScaledAt(t.Context(), r3.NewVec(1, 2, 3), units.Scalar(0.9))
	require.NoError(t, err)
	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 60*0.9*0.9*0.9, volume.Value.Base(), volume.Bound.Base()+1e-10)
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestStraightPrismScaledAtPreservesVolumeAndCapRoles(t *testing.T) {
	doc := decad.New()
	prism := boxBody(t, doc, 0, 0, 4, 3, 5)
	scaled, err := prism.ScaledAt(t.Context(), r3.NewVec(1, 1, 1), units.Scalar(1.5))
	require.NoError(t, err)
	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 202.5, volume.Value.Base(), volume.Bound.Base()+1e-10)
	for _, role := range []decad.FeatureRef{decad.CapStart(prism), decad.CapEnd(prism)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		require.IsType(t, decad.Plane{}, faces[0].Surface())
	}
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)

	shift, err := r3.Translation(r3.NewVec(3, -2, 1))
	require.NoError(t, err)
	placed, err := scaled.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	again, err := placed.ScaledAt(t.Context(), r3.NewVec(0, 0, 0), units.Scalar(0.75))
	require.NoError(t, err)
	volume, err = again.Volume()
	require.NoError(t, err)
	require.InDelta(t, 202.5*0.75*0.75*0.75, volume.Value.Base(), volume.Bound.Base()+1e-10)
	mesh, err = again.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestStraightPrismScaledAtKeepsPolygonHole(t *testing.T) {
	doc := decad.New()
	prism := holedBoxSymmetric(t, doc, 10, 4, 2)
	scaled, err := prism.ScaledAt(t.Context(), r3.NewVec(0, 0, 0), units.Scalar(1.25))
	require.NoError(t, err)
	volume, err := scaled.Volume()
	require.NoError(t, err)
	require.InDelta(t, 2625.0, volume.Value.Base(), volume.Bound.Base()+1e-10)
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestPrismScaledAtRefusesSheetSource(t *testing.T) {
	sheetDoc := decad.New()
	s, profile := plateSketch(t)
	sheet, err := sheetDoc.Extrude(s, profile,
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	_, err = sheet.ScaledAt(t.Context(), r3.NewVec(0, 0, 0), units.Scalar(2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, []*decad.Body{sheet}, sheetDoc.Bodies())
}
