package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func roundedTwistProfile(t *testing.T, offsetX float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	point := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x+offsetX, y)
		s.Fix(p)
		return p
	}
	ends := [8]*sketch.Point{
		point(-1.5, -1), point(1.5, -1), point(2, -0.5), point(2, 0.5),
		point(1.5, 1), point(-1.5, 1), point(-2, 0.5), point(-2, -0.5),
	}
	for i := 0; i < 8; i += 2 {
		s.CreateLine(ends[i], ends[(i+1)%8])
	}
	s.CreateArc(point(1.5, -0.5), ends[1], ends[2])
	s.CreateArc(point(1.5, 0.5), ends[3], ends[4])
	s.CreateArc(point(-1.5, 0.5), ends[5], ends[6])
	s.CreateArc(point(-1.5, -0.5), ends[7], ends[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if candidate.Valid && len(candidate.Outer) == 8 && len(candidate.Holes) == 0 {
			profile = candidate
			break
		}
	}
	require.NotNil(t, profile)
	return s, profile
}

// A solved Sketch supplies the mixed boundary; the real Sweep and tessellator
// must carry its circular corners through the public producer-consumer path.
func TestSweepTwistRoundedProfileVerifyAll(t *testing.T) {
	s, profile := roundedTwistProfile(t, 0)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	require.Len(t, doc.Bodies(), 1)
	require.Len(t, body.Faces(), 10)
	roles := map[string]struct{}{}
	for _, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		_, exists := roles[origins[0].Role]
		require.False(t, exists)
		roles[origins[0].Role] = struct{}{}
	}
	for edge := range 8 {
		require.Contains(t, roles, fmt.Sprintf("side(0,0,%d)", edge))
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-5*(7+math.Pi/4)), volume.Bound.Base()+1e-12)
	require.Less(t, volume.Bound.Base(), 1e-6)
	centroid, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 2.5), centroid.Value)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Less(t, mesh.Bound().Base(), 0.1)
	require.Greater(t, len(mesh.Triangles()), 16)
	live := map[*decad.Face]struct{}{}
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}
	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), body, move)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())

	negative, err := decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(-30)))
	require.NoError(t, err)
	negativeMesh, err := negative.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, negativeMesh.BoundaryVerified())
	require.True(t, negativeMesh.VolumeVerified())
}

func TestSweepTwistRoundedProfileAxisRefusalIsAtomic(t *testing.T) {
	s, profile := roundedTwistProfile(t, 1)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	_, err = decad.Sweep(t.Context(), doc, s, profile, path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Empty(t, doc.Bodies())
}
