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

func TestBodyPatchCircularPrismSheetTessellates(t *testing.T) {
	t.Parallel()
	tube := circularPatchTube(t)
	sourceMesh, err := tube.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, sourceMesh.BoundaryVerified())
	filled, err := tube.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(2))
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, filled.Kind())
	require.Len(t, filled.Faces(), 3)
	mesh, err := filled.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.Greater(t, mesh.Bound().Base(), 0.0)
	require.Len(t, mesh.Vertices(), len(sourceMesh.Vertices()))
	require.Greater(t, len(mesh.Triangles()), len(sourceMesh.Triangles()))
	require.Zero(t, directedEdgeCensus(t, mesh))
	require.InDelta(t, 2*math.Pi*10*20+2*math.Pi*100, meshTriangleArea(mesh), 25)
	requireLiveBodyPatchMeshFaces(t, filled, mesh)
	drawn, err := filled.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	require.False(t, drawn.BoundaryVerified())
	require.Equal(t, mesh.Vertices(), drawn.Vertices())
	require.Equal(t, mesh.Triangles(), drawn.Triangles())
}

func circularPatchTube(t *testing.T) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.CreateCircle(center, 10)
	s.Fix(center)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	tube, err := doc.Extrude(s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return tube
}

func requireLiveBodyPatchMeshFaces(t *testing.T, body *decad.Body, mesh *decad.Mesh) {
	t.Helper()
	live := map[*decad.Face]bool{}
	for _, face := range body.Faces() {
		live[face] = true
	}
	seen := map[*decad.Face]bool{}
	for _, face := range mesh.SourceFaces() {
		require.True(t, live[face])
		seen[face] = true
	}
	require.Len(t, seen, len(body.Faces()))
}

func TestBodyPatchCircularPrismOneRimTessellates(t *testing.T) {
	t.Parallel()
	tube := circularPatchTube(t)
	filled, err := tube.Patch(t.Context(), decad.Edges(decad.Free(),
		decad.EndpointAt(r3.NewVec(10, 0, 0))).Exactly(1))
	require.NoError(t, err)
	mesh, err := filled.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.Equal(t, len(mesh.Vertices())/2, directedEdgeCensus(t, mesh))
	requireLiveBodyPatchMeshFaces(t, filled, mesh)
}

func TestBodyPatchCircularPrismPlacementAndReflection(t *testing.T) {
	t.Parallel()
	tube := circularPatchTube(t)
	filled, err := tube.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(2))
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflection, err := r3.Reflection(frame)
	require.NoError(t, err)
	for _, pose := range []r3.Transform{reflection, func() r3.Transform {
		motion, err := r3.Translation(r3.NewVec(100, -30, 7))
		require.NoError(t, err)
		return motion
	}()} {
		placed, err := filled.PlacedCopy(t.Context(), pose)
		require.NoError(t, err)
		mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.2), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.False(t, mesh.VolumeVerified())
		require.Zero(t, directedEdgeCensus(t, mesh))
		requireLiveBodyPatchMeshFaces(t, placed, mesh)
	}
	reflectedSource, err := circularPatchTube(t).Placed(t.Context(), reflection)
	require.NoError(t, err)
	filledAfterReflection, err := reflectedSource.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(2))
	require.NoError(t, err)
	mesh, err := filledAfterReflection.Tessellate(t.Context(), units.Millimeters(0.2),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.Zero(t, directedEdgeCensus(t, mesh))
	requireLiveBodyPatchMeshFaces(t, filledAfterReflection, mesh)
}

func TestBodyPatchCircularPrismNestedFillMeshRefuses(t *testing.T) {
	t.Parallel()
	tube := circularPatchTube(t)
	oneRim, err := tube.Patch(t.Context(), decad.Edges(decad.Free(),
		decad.EndpointAt(r3.NewVec(10, 0, 0))).Exactly(1))
	require.NoError(t, err)
	bothRims, err := oneRim.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(1))
	require.NoError(t, err)
	_, err = bothRims.Tessellate(t.Context(), units.Millimeters(0.2),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
