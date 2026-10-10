package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func fittedSplineRevolveProfile(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	start := s.CreatePoint(-1, -1)
	middle := s.CreatePoint(0, -1.25)
	end := s.CreatePoint(1, -1)
	_, err = s.CreateFitSpline(start, middle, end)
	require.NoError(t, err)
	rightTop := s.CreatePoint(1, 1)
	leftTop := s.CreatePoint(-1, 1)
	s.CreateLine(end, rightTop)
	s.CreateLine(rightTop, leftTop)
	s.CreateLine(leftTop, start)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	require.True(t, profiles[0].Valid)
	return s, profiles[0]
}

func fittedSplineRevolveVolume(t *testing.T, s *sketch.Sketch, profile *sketch.Profile, sweep float64) float64 {
	t.Helper()
	measured, err := decad.MeasureProfile(s, profile)
	require.NoError(t, err)
	area, err := measured.Area()
	require.NoError(t, err)
	centroid, err := measured.Centroid()
	require.NoError(t, err)
	return sweep * area.Value.Base() * (centroid.Value.X + 5)
}

func TestRevolveFittedSplineProfile(t *testing.T) {
	s, profile := fittedSplineRevolveProfile(t)
	axis := decad.SketchLine{Start: decad.Point2{U: -5, V: -5}, End: decad.Point2{U: -5, V: 5}}
	body, err := decad.New().Revolve(s, profile, axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	found := false
	for _, face := range body.Faces() {
		if face.Surface().Kind() == decad.KindNURBS {
			found = true
		}
	}
	require.True(t, found, "the fitted spline must stay a free-form surface")
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, fittedSplineRevolveVolume(t, s, profile, 2*math.Pi), volume.Value.Base(), 1e-7)
	require.Greater(t, volume.Value.Base()-volume.Bound.Base(), 0.0)
	require.False(t, math.IsInf(volume.Bound.Base(), 0))
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	curved := false
	for _, vertex := range mesh.Vertices() {
		if vertex.Y < -1.1 {
			curved = true
		}
	}
	require.True(t, curved, "the mesh must contain the fitted spline's bowed meridian")
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestRevolveFittedSplinePartial(t *testing.T) {
	s, profile := fittedSplineRevolveProfile(t)
	axis := decad.SketchLine{Start: decad.Point2{U: -5, V: -5}, End: decad.Point2{U: -5, V: 5}}
	body, err := decad.New().Revolve(s, profile, axis,
		decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
	require.NoError(t, err)
	freeformEdges := 0
	for _, edge := range body.Edges() {
		if _, ok := edge.Curve().(decad.NURBSCurve); ok {
			freeformEdges++
		}
	}
	require.Equal(t, 2, freeformEdges)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, fittedSplineRevolveVolume(t, s, profile, math.Pi/2), volume.Value.Base(), 1e-7)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}
