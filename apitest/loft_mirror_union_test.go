package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func loftMirrorPlane(t *testing.T) decad.MirrorFrame {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return decad.MirrorFrame{Frame: frame}
}

func TestLoftMirrorUnionSealsTheSharedCap(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 2, 1)
	doc := decad.New()
	lower, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), lower, loftMirrorPlane(t))
	require.NoError(t, err)
	joined, err := decad.Union(upper, lower).Do(t.Context())
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	require.Len(t, doc.Bodies(), 1)

	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.Vertices(), 12)
	require.Len(t, mesh.Triangles(), 20)
	for _, role := range []decad.FeatureRef{decad.CapStart(lower), decad.CapStart(upper),
		decad.CapStart(joined), decad.CapEnd(joined)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(joined)
		require.NoError(t, err)
		require.IsType(t, decad.Plane{}, faces[0].Surface())
	}
	volume, err := joined.Volume()
	require.NoError(t, err)
	// Each half is the frustum between 4x4 and 2x2 squares over 10 mm.
	want := 2 * 10 * (16 + math.Sqrt(16*4) + 4) / 3
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9)

	spin, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(45))
	require.NoError(t, err)
	placed, err := decad.Copy(t.Context(), joined, decad.WithCopyPlacement(spin))
	require.NoError(t, err)
	copyMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, copyMesh.BoundaryVerified())
	require.True(t, copyMesh.VolumeVerified())
	for _, role := range []decad.FeatureRef{decad.CapStart(placed), decad.CapEnd(placed)} {
		faces, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(placed)
		require.NoError(t, err)
		require.IsType(t, decad.Plane{}, faces[0].Surface())
	}
}

// TestLoftMirrorUnionKeepsUnrelatedSourcesSeparate unions a loft with the mirrored copy of another loft of the same
// sections. The mirror join reads one source only, so it declines, and the mesh Union decides the pair: the two lofts
// lie on opposite sides of the shared cap's plane (agent-docs/coplanar-contact-design.md §19). Its result carries the
// operands' cap roles, never the join's own.
func TestLoftMirrorUnionKeepsUnrelatedSourcesSeparate(t *testing.T) {
	t.Parallel()
	s0, p0, s1, p1 := loftSquares(t, 2, 1)
	doc := decad.New(decad.ExactOnly())
	lower, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	other, err := decad.Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	upper, err := decad.MirroredCopy(t.Context(), other, loftMirrorPlane(t))
	require.NoError(t, err)
	joined, err := decad.Union(lower, upper).Do(t.Context())
	require.NoError(t, err)
	require.True(t, joined.IsSolid())
	_, err = decad.Faces(decad.FaceCreatedBy(decad.CapStart(joined))).Exactly(1).SelectFaces(joined)
	require.Error(t, err, "the mirror join's own start cap is absent")
	for _, role := range []decad.FeatureRef{decad.CapStart(lower), decad.CapStart(upper)} {
		_, err := decad.Faces(decad.FaceCreatedBy(role)).Exactly(1).SelectFaces(joined)
		require.NoError(t, err)
	}
	volume, err := joined.Volume()
	require.NoError(t, err)
	want := 2 * 10 * (16 + math.Sqrt(16*4) + 4) / 3
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9)
}
