package apitest_test

import (
	"bytes"
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestPatchSheetTessellates(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	b, err := decad.New().Patch(t.Context(), s, p)
	require.NoError(t, err)
	m, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Len(t, m.Triangles(), 2)
	require.Len(t, m.Vertices(), 4)
	require.Equal(t, 4, directedEdgeCensus(t, m))
	require.InDelta(t, 6000, meshTriangleArea(m), 1e-9)
	require.Zero(t, m.Bound().Mag())
	require.True(t, m.BoundaryVerified())
	require.False(t, m.VolumeVerified())
	for i, tri := range m.Triangles() {
		require.Same(t, b.Faces()[0], m.SourceFaces()[i])
		v := m.Vertices()
		require.Positive(t, v[tri[1]].Sub(v[tri[0]]).Cross(v[tri[2]].Sub(v[tri[0]])).Z)
	}
	got, err := b.Faces()[0].DistanceToPoint(t.Context(), r3.NewVec(50, 30, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(got.Value.Base()-5), got.Bound.Base())

	var a, c bytes.Buffer
	require.NoError(t, export.STL(t.Context(), &a, b, units.Millimeters(0.1)))
	require.NoError(t, export.STL(t.Context(), &c, b, units.Millimeters(0.1)))
	require.Equal(t, a.String(), c.String())
	require.Equal(t, 2, countSTLFacets(a.String()))

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = b.Tessellate(ctx, units.Millimeters(0.1))
	require.ErrorIs(t, err, context.Canceled)
}

func TestPatchSheetWithHoleTessellates(t *testing.T) {
	t.Parallel()
	s, p := rectWithHoleSketch(t)
	b, err := decad.New().Patch(t.Context(), s, p)
	require.NoError(t, err)
	coarse, err := b.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	fine, err := b.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, coarse.BoundaryVerified())
	require.False(t, coarse.VolumeVerified())
	require.Positive(t, coarse.Bound().Mag())
	require.LessOrEqual(t, coarse.Bound().Mag(), 0.5)
	require.Less(t, fine.Bound().Mag(), coarse.Bound().Mag())
	require.Greater(t, directedEdgeCensus(t, coarse), 4)
	require.LessOrEqual(t, math.Abs(meshTriangleArea(coarse)-(6000-100*math.Pi)),
		coarse.Bound().Mag()*1000)
	for _, face := range coarse.SourceFaces() {
		require.Same(t, b.Faces()[0], face)
	}
	// The hole center lies on the plane but outside the trimmed face.
	distance, err := b.Faces()[0].DistanceToPoint(t.Context(), r3.NewVec(70, 30, 0), units.Millimeters(0.05))
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(distance.Value.Base()-10), distance.Bound.Base())
}

func TestPatchSheetReflectedMeshFollowsFaceNormal(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	b, err := decad.New().Patch(t.Context(), s, p)
	require.NoError(t, err)
	plane, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	motion, err := r3.Reflection(plane)
	require.NoError(t, err)
	placed, err := b.Placed(t.Context(), motion)
	require.NoError(t, err)
	m, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 4, directedEdgeCensus(t, m))
	for _, tri := range m.Triangles() {
		v := m.Vertices()
		require.Negative(t, v[tri[1]].Sub(v[tri[0]]).Cross(v[tri[2]].Sub(v[tri[0]])).Z)
	}
}

func TestPatchSheetPlacementBoundAndVerificationLevels(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	b, err := decad.New().Patch(t.Context(), s, p)
	require.NoError(t, err)
	turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	rotated, err := b.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1e7, 3e6, -2e6))
	require.NoError(t, err)
	placed, err := rotated.PlacedCopy(t.Context(), shift)
	require.NoError(t, err)
	all, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	none, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.Greater(t, all.Bound().Mag(), 0.0)
	require.Equal(t, all.Bound(), none.Bound())
	require.Equal(t, all.Vertices(), none.Vertices())
	require.Equal(t, all.Triangles(), none.Triangles())
	require.Equal(t, all.SourceFaces(), none.SourceFaces())
	require.Equal(t, 4, directedEdgeCensus(t, all))
	require.True(t, all.BoundaryVerified())
	require.False(t, none.BoundaryVerified())
}
