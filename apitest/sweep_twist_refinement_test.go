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

func roundedLargeTwistProfile(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	ends := [8]*sketch.Point{
		point(-2.5, -3), point(2.5, -3), point(4, -1.5), point(4, 1.5),
		point(2.5, 3), point(-2.5, 3), point(-4, 1.5), point(-4, -1.5),
	}
	s.CreateLine(ends[0], ends[1])
	s.CreateArc(point(2.5, -1.5), ends[1], ends[2])
	s.CreateLine(ends[2], ends[3])
	s.CreateArc(point(2.5, 1.5), ends[3], ends[4])
	s.CreateLine(ends[4], ends[5])
	s.CreateArc(point(-2.5, 1.5), ends[5], ends[6])
	s.CreateLine(ends[6], ends[7])
	s.CreateArc(point(-2.5, -1.5), ends[7], ends[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Outer) == 8 && len(profile.Holes) == 0 {
			return s, profile
		}
	}
	t.Fatal("Sketch did not produce the rounded outline")
	return nil, nil
}

func TestSweepTwistRoundedLargeProfileVerifyAll(t *testing.T) {
	s, profile := roundedLargeTwistProfile(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), decad.New(), s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	require.Greater(t, len(mesh.Triangles()), 1000)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-5*(39+2.25*math.Pi)), volume.Bound.Base()+1e-12)
}

func TestSweepTwistRoundedLargeProfileKeepsCoarseCertifiedBuild(t *testing.T) {
	s, profile := roundedLargeTwistProfile(t)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	body, err := decad.Sweep(t.Context(), decad.New(), s, profile, path,
		decad.WithSweepTwist(units.Radians(1.4)))
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.2),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Len(t, mesh.Triangles(), 2900)
	require.InDelta(t, 0.19517649705582835, mesh.Bound().Base(), 1e-12)
}
