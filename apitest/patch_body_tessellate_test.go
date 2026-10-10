package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBodyPatchPlanarCappedTubeTessellates(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := surfaceTube(t, doc)
	filled, err := tube.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)

	mesh, err := filled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 12)
	require.Zero(t, directedEdgeCensus(t, mesh))
	require.Len(t, mesh.SourceFaces(), 12)
	require.InDelta(t, 15200, meshTriangleArea(mesh), 1e-9)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())

	live := map[*decad.Face]struct{}{}
	for _, f := range filled.Faces() {
		live[f] = struct{}{}
	}
	seen := map[*decad.Face]struct{}{}
	for _, f := range mesh.SourceFaces() {
		require.Contains(t, live, f)
		seen[f] = struct{}{}
	}
	require.Len(t, seen, 6)
}

func TestBodyPatchPlanarMeshPlacementAndVerification(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := surfaceTube(t, doc)
	filled, err := tube.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	tol := units.Millimeters(0.1)
	drawn, err := filled.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyNone))
	require.NoError(t, err)
	verified, err := filled.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.Equal(t, drawn.Vertices(), verified.Vertices())
	require.Equal(t, drawn.Triangles(), verified.Triangles())
	require.False(t, drawn.BoundaryVerified())
	require.True(t, verified.BoundaryVerified())

	motion, err := r3.Translation(r3.NewVec(3, 4, 5))
	require.NoError(t, err)
	placed, err := filled.PlacedCopy(t.Context(), motion)
	require.NoError(t, err)
	mesh, err := placed.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Zero(t, directedEdgeCensus(t, mesh))
	require.Greater(t, mesh.Bound().Base(), 0.0)
	require.False(t, mesh.VolumeVerified())

	mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflection, err := r3.Reflection(mirror)
	require.NoError(t, err)
	reflected, err := filled.PlacedCopy(t.Context(), reflection)
	require.NoError(t, err)
	reflMesh, err := reflected.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.Zero(t, directedEdgeCensus(t, reflMesh))
	require.InDelta(t, 60000, meshVolume(reflMesh), 1e-7)

	other := decad.New()
	reflectedWalls, err := surfaceTube(t, other).Placed(t.Context(), reflection)
	require.NoError(t, err)
	filledAfterReflection, err := reflectedWalls.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	meshAfterReflection, err := filledAfterReflection.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.InDelta(t, 60000, meshVolume(meshAfterReflection), 1e-7)
}

func TestBodyPatchCurvedRimMeshRefusesWithoutSharedChording(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	disc := circleSheet(t, doc, 10)
	filled, err := disc.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(1))
	require.NoError(t, err)
	_, err = filled.Tessellate(t.Context(), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "Circle3 edge")
}

func TestBodyPatchPlanarHoleFillTessellates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	outer := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(outer.A)
	s.CreateRectangle(30, 20, 70, 40)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	sheet, err := doc.Patch(t.Context(), s, profile)
	require.NoError(t, err)
	filled, err := sheet.Patch(t.Context(), decad.Edges(decad.Free(), decad.Concave()).Exactly(4))
	require.NoError(t, err)
	mesh, err := filled.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Equal(t, 4, directedEdgeCensus(t, mesh))
	require.InDelta(t, 6000, meshTriangleArea(mesh), 1e-9)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
}

func TestBodyPatchCoincidentFacesRefuseVerifiedMesh(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Patch(t.Context(), s, p)
	require.NoError(t, err)
	doubled, err := sheet.Patch(t.Context(), decad.Edges(decad.Free()).Exactly(4))
	require.NoError(t, err)
	_, err = doubled.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
