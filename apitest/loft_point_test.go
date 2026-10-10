package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func pointLoftSquare(t *testing.T, w *sketch.World, height float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), height)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	r := s.CreateRectangle(-2, -2, 2, 2)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func TestLoftFromPointPyramidPreservesApexAndVolume(t *testing.T) {
	t.Parallel()
	s, profile := pointLoftSquare(t, sketch.NewWorld(), 6)
	doc := decad.New()
	apex := r3.NewVec(0, 0, 0)
	body, err := doc.LoftFromPoint(t.Context(), apex, s, profile)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Vertices(), 5)
	foundApex := false
	for _, v := range body.Vertices() {
		if v.Position().Value == apex {
			foundApex = true
			require.Zero(t, v.Position().Bound.Base())
			break
		}
	}
	require.True(t, foundApex)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 32, volume.Value.Base(), 1e-12)
	require.LessOrEqual(t, volume.Bound.Base(), 1e-8)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.Triangles(), 6)
}

func TestLoftFromPointKeepsFittedFarBoundary(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), 5)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	p0 := s.CreatePoint(-2, -1)
	p1 := s.CreatePoint(0, -2)
	p2 := s.CreatePoint(2, -1)
	p3 := s.CreatePoint(2, 2)
	p4 := s.CreatePoint(-2, 2)
	for _, p := range []*sketch.Point{p0, p1, p2, p3, p4} {
		s.Fix(p)
	}
	_, err = s.CreateFitSpline(p0, p1, p2)
	require.NoError(t, err)
	s.CreateLine(p2, p3)
	s.CreateLine(p3, p4)
	s.CreateLine(p4, p0)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	_, fitted := profiles[0].Outer[0].Entity.(*sketch.FitSpline)
	require.True(t, fitted)
	body, err := decad.New().LoftFromPoint(t.Context(), r3.NewVec(0, 0, 0), s, profiles[0])
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.01), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, len(mesh.Vertices()), 5)
	require.Positive(t, mesh.Bound().Base())
	volume, err := body.Volume()
	require.NoError(t, err)
	wantVolume := profiles[0].Area * 5 / 3
	require.InDelta(t, wantVolume, volume.Value.Base(), volume.Bound.Base()+1e-5)
	turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	copy, err := body.PlacedCopy(t.Context(), turn)
	require.NoError(t, err)
	copyMesh, err := copy.Tessellate(t.Context(), units.Millimeters(0.01),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, copyMesh.BoundaryVerified())
	require.True(t, copyMesh.VolumeVerified())
}

func TestLoftFromPointRejectsApexInPlaneAndHoles(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, profile := pointLoftSquare(t, w, 6)
	doc := decad.New()
	_, err := doc.LoftFromPoint(t.Context(), r3.NewVec(0, 0, 6), s, profile)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Empty(t, doc.Bodies())

	plane, err := w.CreateOffsetPlane(w.XY(), 7)
	require.NoError(t, err)
	holed, err := w.CreateSketch(plane)
	require.NoError(t, err)
	outer := holed.CreateRectangle(-3, -3, 3, 3)
	inner := holed.CreateRectangle(-1, -1, 1, 1)
	holed.Fix(outer.A)
	holed.Fix(inner.A)
	_, err = holed.Solve(t.Context())
	require.NoError(t, err)
	for _, p := range holed.Profiles() {
		if len(p.Holes) != 1 {
			continue
		}
		_, err = doc.LoftFromPoint(t.Context(), r3.NewVec(0, 0, 0), holed, p)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Empty(t, doc.Bodies())
		return
	}
	t.Fatal("Sketch did not return the holed region")
}
