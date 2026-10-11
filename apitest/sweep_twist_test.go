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

// This uses Sketch's solved profile and Decad's recorded line and arc. The
// curved span must retain one live wall face per input edge, while its held
// mesh has the extra angular stations needed for a useful volume proof.
func TestSweepTwistCompositeCardinalArc(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-0.1, -0.1, 0.1, 0.1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 1)},
		decad.ArcThrough{Through: r3.NewVec(2, 0, 5), End: r3.NewVec(5, 0, 6)})
	require.NoError(t, err)
	body, err := decad.New().Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(5)))
	require.NoError(t, err)
	require.Len(t, body.Faces(), 10)
	roles := map[string]*decad.Face{}
	for _, face := range body.Faces() {
		require.Len(t, face.Origins(), 1)
		role := face.Origins()[0].Role
		require.NotContains(t, roles, role)
		roles[role] = face
	}
	for span := range 2 {
		for edge := range 4 {
			require.Contains(t, roles, fmt.Sprintf("side(%d,0,%d)", span, edge))
		}
	}
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-0.3541592653589793),
		volume.Bound.Base()+1e-15)
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.Len(t, mesh.Triangles(), 76)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, 0.35100079890664354, meshVolume(mesh), 1e-12)
	for _, face := range mesh.SourceFaces() {
		require.Contains(t, roles, face.Origins()[0].Role)
		require.Same(t, roles[face.Origins()[0].Role], face)
	}
	_, err = body.Tessellate(t.Context(), units.Millimeters(0.01),
		decad.WithVerification(decad.VerifyAll))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	move, err := r3.Translation(r3.NewVec(7, -3, 11))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), move)
	require.NoError(t, err)
	placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, placedMesh.BoundaryVerified())
	require.True(t, placedMesh.VolumeVerified())
	mirrorFrame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	mirror, err := r3.Reflection(mirrorFrame)
	require.NoError(t, err)
	reflected, err := placed.PlacedCopy(t.Context(), mirror)
	require.NoError(t, err)
	reflectedMesh, err := reflected.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, reflectedMesh.BoundaryVerified())
	require.True(t, reflectedMesh.VolumeVerified())
}

func TestSweepTwistCompositeCardinalArcAdmission(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-0.1, -0.1, 0.1, 0.1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 1)},
		decad.ArcThrough{Through: r3.NewVec(2, 0, 5), End: r3.NewVec(5, 0, 6)})
	require.NoError(t, err)
	doc := decad.New()
	negative, err := doc.Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(-5)))
	require.NoError(t, err)
	mesh, err := negative.Tessellate(t.Context(), units.Millimeters(0.5),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	before := doc.Bodies()
	_, err = doc.Sweep(t.Context(), s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(70)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	wideWorld := sketch.NewWorld()
	wideSketch, err := wideWorld.CreateSketch(wideWorld.XY())
	require.NoError(t, err)
	wide := wideSketch.CreateRectangle(-1, -1, 1, 1)
	wideSketch.Fix(wide.A)
	_, err = wideSketch.Solve(t.Context())
	require.NoError(t, err)
	_, err = doc.Sweep(t.Context(), wideSketch, wideSketch.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(5)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Equal(t, before, doc.Bodies())
	largeArc, err := decad.NewPath(r3.Vec{},
		decad.LineTo{End: r3.NewVec(0, 0, 1)},
		decad.ArcThrough{Through: r3.NewVec(40, 0, 81), End: r3.NewVec(100, 0, 101)})
	require.NoError(t, err)
	_, err = doc.Sweep(t.Context(), s, s.Profiles()[0], largeArc,
		decad.WithSweepTwist(units.Degrees(5)))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "boundary tube exceeds")
	require.Equal(t, before, doc.Bodies())
}
