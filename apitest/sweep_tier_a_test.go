package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepCompositeTierAFitProfile(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(-1, -1)
	mid := s.CreatePoint(0, -1.25)
	end := s.CreatePoint(1, -1)
	_, err = s.CreateFitSpline(start, mid, end)
	require.NoError(t, err)
	topRight := s.CreatePoint(1, 1)
	topLeft := s.CreatePoint(-1, 1)
	s.CreateLine(end, topRight)
	s.CreateLine(topRight, topLeft)
	s.CreateLine(topLeft, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	measured, err := decad.MeasureProfile(s, profiles[0])
	require.NoError(t, err)
	sectionArea, err := measured.Area()
	require.NoError(t, err)
	sectionCentroid, err := measured.Centroid()
	require.NoError(t, err)

	body, err := decad.New().Sweep(t.Context(), s, profiles[0], orthogonalSweepPath(t))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	freeformWalls := 0
	for _, face := range body.Faces() {
		if face.Surface().Kind() == decad.KindNURBS {
			freeformWalls++
		}
	}
	require.Equal(t, 3, freeformWalls, "each path span must preserve the fitted spline wall")
	volume, err := body.Volume()
	require.NoError(t, err)
	// The first arc bends across the section's symmetric U coordinate. The
	// second bends across V, so its centroid shifts the swept path length.
	expected := sectionArea.Value.Base() * (10 + 5*math.Pi - sectionCentroid.Value.Y*math.Pi/2)
	require.InDelta(t, expected, volume.Value.Base(), 1e-6)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	curved := false
	for _, vertex := range mesh.Vertices() {
		if vertex.Y < -1.1 {
			curved = true
		}
	}
	require.True(t, curved, "the mesh must sample the bowed fitted spline")
	require.Positive(t, meshVolume(mesh))
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	live := map[*decad.Face]struct{}{}
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
}

func TestSweepCompositeTwoFittedSplinesReplayAndVerify(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rightBottom := s.CreatePoint(1, -1)
	leftBottom := s.CreatePoint(-1, -1)
	_, err = s.CreateFitSpline(rightBottom, s.CreatePoint(0, -1.25), leftBottom)
	require.NoError(t, err)
	leftTop := s.CreatePoint(-1, 1)
	rightTop := s.CreatePoint(1, 1)
	s.CreateLine(leftBottom, leftTop)
	_, err = s.CreateFitSpline(leftTop, s.CreatePoint(0, 1.25), rightTop)
	require.NoError(t, err)
	s.CreateLine(rightTop, rightBottom)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	measured, err := decad.MeasureProfile(s, profiles[0])
	require.NoError(t, err)
	sectionArea, err := measured.Area()
	require.NoError(t, err)

	body, err := decad.New().Sweep(t.Context(), s, profiles[0], orthogonalSweepPath(t))
	require.NoError(t, err)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, sectionArea.Value.Base()*(10+5*math.Pi), volume.Value.Base(), 1e-6)
	freeformWalls := 0
	for _, face := range body.Faces() {
		if face.Surface().Kind() == decad.KindNURBS {
			freeformWalls++
		}
	}
	require.Equal(t, 6, freeformWalls)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	duplicate, err := body.Duplicate(t.Context())
	require.NoError(t, err)
	copyMesh, err := duplicate.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, copyMesh.VolumeVerified())
}

func TestSweepCompositeTierAWorkCeiling(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	corners := []*sketch.Point{
		s.CreatePoint(-1, -1), s.CreatePoint(1, -1),
		s.CreatePoint(1, 1), s.CreatePoint(-1, 1),
	}
	middles := []*sketch.Point{
		s.CreatePoint(0, -1.25), s.CreatePoint(1.25, 0),
		s.CreatePoint(0, 1.25), s.CreatePoint(-1.25, 0),
	}
	for i := range corners {
		_, err = s.CreateFitSpline(corners[i], middles[i], corners[(i+1)%len(corners)])
		require.NoError(t, err)
	}
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	doc := decad.New()
	_, err = doc.Sweep(t.Context(), s, profiles[0], orthogonalSweepPath(t))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "fixed work budget of 8388608")
	require.Empty(t, doc.Bodies())
}
