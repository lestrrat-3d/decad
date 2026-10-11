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

// TestSweepTwistStraightPolygon verifies the true rotating solid against its
// distinct held facet shell. A loft between the endpoint squares encloses
// only 15.77 mm³, while the rotating square encloses exactly 20 mm³.
func TestSweepTwistStraightPolygon(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, profiles[0], path,
		decad.WithSweepTwist(units.Degrees(30)))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	require.Len(t, body.Faces(), 6)
	require.Len(t, body.Edges(), 12)
	require.Len(t, body.Vertices(), 8)
	var facets int
	roles := make(map[string]bool)
	for _, face := range body.Faces() {
		origin := face.Origins()
		require.Len(t, origin, 1)
		require.False(t, roles[origin[0].Role])
		roles[origin[0].Role] = true
		if face.Surface().Kind() == decad.KindFaceted {
			facets++
		}
	}
	require.Equal(t, 4, facets)
	for edge := range 4 {
		role := fmt.Sprintf("side(0,0,%d)", edge)
		require.True(t, roles[role], role)
	}
	for _, edge := range body.Edges() {
		require.Len(t, edge.Faces(), 2)
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	require.Equal(t, 20.0, volume.Value.Base())
	centroid, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(0, 0, 2.5), centroid.Value)
	area, err := body.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-48.07298839110878), area.Bound.Base())

	_, err = body.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 12)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.Greater(t, mesh.Bound().Base(), 0.3)
	require.Less(t, mesh.Bound().Base(), 0.5)
	require.InDelta(t, 15.77350269189626, meshVolume(mesh), 1e-9)
	live := make(map[*decad.Face]struct{}, len(body.Faces()))
	for _, face := range body.Faces() {
		live[face] = struct{}{}
	}
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, live, face)
	}

	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	placedVolume, err := placed.Volume()
	require.NoError(t, err)
	require.InDelta(t, 20, placedVolume.Value.Base(), placedVolume.Bound.Base()+1e-12)
	placedCentroid, err := placed.Centroid()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(7, -3, 13.5), placedCentroid.Value)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
}

func TestSweepTwistNegativeAngleAndAdmission(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, 5)})
	require.NoError(t, err)
	doc := decad.New()
	body, err := doc.Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(-30)))
	require.NoError(t, err)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())

	before := doc.Bodies()
	_, err = doc.Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(70)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	_, err = doc.Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(30)), decad.WithSurfaceResult())
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
}
