package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func dividedSquareUnion(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	points := []*sketch.Point{
		s.CreatePoint(0, 0), s.CreatePoint(10, 0),
		s.CreatePoint(10, 10), s.CreatePoint(0, 10),
	}
	for i := range points {
		s.Fix(points[i])
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	s.CreateLine(points[0], points[2])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 2)
	profile, err := s.UnionProfiles(0, 1)
	require.NoError(t, err)
	require.True(t, profile.Valid)
	require.Len(t, profile.Outer, 4)
	require.InDelta(t, 100, profile.Area, 1e-10)
	return s, profile
}

func TestExtrudeAuthenticatedUnionProfile(t *testing.T) {
	s, p := dividedSquareUnion(t)
	body, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	volume, err := body.Volume()
	require.NoError(t, err)
	require.InDelta(t, 200, volume.Value.Base(), volume.Bound.Base()+1e-10)
}

func TestUnionProfileRejectsTamperedBoundary(t *testing.T) {
	s, p := dividedSquareUnion(t)
	p.Outer[0].TStart += 0.01
	_, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}

func TestUnionProfileRejectsStaleSelection(t *testing.T) {
	s, p := dividedSquareUnion(t)
	s.CreatePoint(20, 20)
	_, err := decad.New().Extrude(s, p, decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrStaleProfile)
}
