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

func TestSweepChainSingleArcHalfAnnulusVerifies(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b := s.CreatePoint(0, 0), s.CreatePoint(4, 0)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	require.Empty(t, s.Profiles())

	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, s.Chains()[0], path)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
	require.Len(t, body.Faces(), 1)
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(body)
	require.NoError(t, err)
	require.Len(t, free, 4)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-12*math.Pi), area.Bound.Base()+1e-12)
	box, err := body.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.Vec{}, box.Min)
	require.Equal(t, r3.NewVec(10, 0, 5), box.Max)

	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.False(t, mesh.VolumeVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	require.Greater(t, len(mesh.Triangles()), 2)
	maxZ := math.Inf(-1)
	for _, point := range mesh.Vertices() {
		maxZ = math.Max(maxZ, point.Z)
	}
	require.Greater(t, maxZ, 4.9)
	requireChainArcWinding(t, body, mesh)

	for _, level := range []struct {
		verify   decad.Verification
		boundary bool
	}{
		{decad.VerifyNone, false},
		{decad.VerifyBoundary, true},
	} {
		other, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
			decad.WithVerification(level.verify))
		require.NoError(t, err)
		require.Equal(t, mesh.Vertices(), other.Vertices())
		require.Equal(t, mesh.Triangles(), other.Triangles())
		require.Equal(t, level.boundary, other.BoundaryVerified())
		require.False(t, other.VolumeVerified())
	}
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	result, err := report.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityUndecided, result.Validity.Outcome)
}

func TestSweepChainSingleArcNoncardinalAndReflectedMesh(t *testing.T) {
	s, chain := sweepChainArcLine(t, r3.NewVec(0, 0, 0), r3.NewVec(2, 1, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(3, 4, 5), End: r3.NewVec(6, 8, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	base, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	mesh, err := base.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.LessOrEqual(t, mesh.Bound().Base(), 0.1)
	requireChainArcWinding(t, base, mesh)

	plane, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	flip, err := r3.Reflection(plane)
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), base, flip)
	require.NoError(t, err)
	reflected, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, reflected.BoundaryVerified())
	requireChainArcWinding(t, placed, reflected)
}

func TestSweepChainSingleArcAxisEndMeshRefuses(t *testing.T) {
	s, chain := sweepChainArcLine(t, r3.NewVec(5, 0, 0), r3.NewVec(4, 0, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
}

func TestSweepChainSingleArcOppositeAxisSideWinding(t *testing.T) {
	s, chain := sweepChainArcLine(t, r3.NewVec(6, 0, 0), r3.NewVec(8, 0, 0))
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, chain, path)
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	requireChainArcWinding(t, body, mesh)
}

func TestSweepChainSingleArcMultiSegmentMeshRefuses(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a, b, c := s.CreatePoint(0, 0), s.CreatePoint(2, 0), s.CreatePoint(4, 1)
	s.Fix(a)
	s.Fix(b)
	s.Fix(c)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	path, err := decad.NewPath(r3.Vec{}, decad.ArcThrough{
		Through: r3.NewVec(5, 0, 5), End: r3.NewVec(10, 0, 0),
	})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, s.Chains()[0], path)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 2)
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 1)
}

func sweepChainArcLine(t *testing.T, a, b r3.Vec) (*sketch.Sketch, *sketch.Chain) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	p, q := s.CreatePoint(a.X, a.Y), s.CreatePoint(b.X, b.Y)
	s.Fix(p)
	s.Fix(q)
	s.CreateLine(p, q)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Chains(), 1)
	return s, s.Chains()[0]
}

func requireChainArcWinding(t *testing.T, b *decad.Body, mesh *decad.Mesh) {
	t.Helper()
	verts, tris := mesh.Vertices(), mesh.Triangles()
	require.NotEmpty(t, tris)
	tri := tris[len(tris)/2]
	a, c, d := verts[tri[0]], verts[tri[1]], verts[tri[2]]
	mid := a.Add(c).Add(d).Scale(1.0 / 3)
	faceNormal, err := b.Faces()[0].NormalAt(mid)
	require.NoError(t, err)
	require.Positive(t, c.Sub(a).Cross(d.Sub(a)).Dot(faceNormal.Value))
}
