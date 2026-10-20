package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The tilted box crosses the first box's floor without sharing a face plane.
// Their mesh Union keeps a bounded source face for each operand.
func facetedBooleanScaleSource(t *testing.T) (*decad.Document, *decad.Body, [2]decad.FeatureRef) {
	t.Helper()
	doc := decad.New()
	first := boxBody(t, doc, 0, 0, 10, 10, 10)
	second := boxBody(t, doc, 4, 4, 12, 8, 8)
	turn, err := r3.RotationAround(r3.NewVec(8, 6, 4), r3.Vec{X: 1}, units.Degrees(20))
	require.NoError(t, err)
	second, err = decad.Transform(t.Context(), second, turn)
	require.NoError(t, err)
	roles := [2]decad.FeatureRef{decad.CapStart(first), decad.CapEnd(second)}
	joined, err := decad.Union(t.Context(), first, second)
	require.NoError(t, err)
	require.Len(t, joined.Lumps(), 1)
	require.True(t, tookMeshPath(joined))
	return doc, joined, roles
}

func TestFacetedBooleanScaledAtCarriesVolumeRolesAndVerification(t *testing.T) {
	doc, joined, roles := facetedBooleanScaleSource(t)
	before, err := joined.Volume()
	require.NoError(t, err)
	angle := 20 * math.Pi / 180
	sin, cos := math.Sin(angle), math.Cos(angle)
	belowFloor := 2*sin + 4*cos - 4
	// The only clipped part of the tilted 8×4×8 box is a triangular
	// cross-section below z=0 over the first box's six-millimetre x overlap.
	analytic := 1064 + 3*belowFloor*belowFloor/(sin*cos)
	require.LessOrEqual(t, math.Abs(before.Value.Base()-analytic), before.Bound.Base())

	_, err = decad.ScaledAt(t.Context(), joined, r3.NewVec(1, 2, 3), units.Scalar(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = decad.ScaledAt(canceled, joined, r3.NewVec(1, 2, 3), units.Scalar(1.5))
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, []*decad.Body{joined}, doc.Bodies())

	scaled, err := decad.ScaledAt(t.Context(), joined, r3.NewVec(1, 2, 3), units.Scalar(1.5))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{scaled}, doc.Bodies())
	volume, err := scaled.Volume()
	require.NoError(t, err)
	want := analytic * 3.375
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base())
	for _, role := range roles {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).SelectFaces(scaled)
		require.NoError(t, err)
		require.NotEmpty(t, faces)
	}
	mesh, err := scaled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
}

func TestFacetedBooleanScaledAtAfterPlacementAndRepeat(t *testing.T) {
	_, joined, _ := facetedBooleanScaleSource(t)
	before, err := joined.Volume()
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(4, -3, 2))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), joined, move)
	require.NoError(t, err)
	first, err := decad.ScaledAt(t.Context(), placed, r3.NewVec(1, 2, 3), units.Scalar(1.5))
	require.NoError(t, err)
	again, err := decad.ScaledAt(t.Context(), first, r3.Vec{}, units.Scalar(0.8))
	require.NoError(t, err)
	volume, err := again.Volume()
	require.NoError(t, err)
	want := before.Value.Base() * 3.375 * 0.512
	bound := before.Bound.Base()*3.375*0.512 + volume.Bound.Base()
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), bound)
	mesh, err := again.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
