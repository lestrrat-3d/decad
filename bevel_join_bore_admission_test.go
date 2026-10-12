package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This source uses a short fitted tab, not an involute or a complete gear.
// Its six separate copies exercise the retained cap-ring record across a
// count that the former bore switch did not admit.
func bevelBoreTestJoin(t *testing.T, doc *Document) *Body {
	t.Helper()
	blank := bevelFitTestRevolve(t, doc, []Point2{
		{U: 7.5}, {U: 9.5}, {U: 9.5, V: 8.5},
		{U: 9, V: 9}, {U: 7, V: 7}, {U: 7.5, V: 6.5},
	})
	world := sketch.NewWorld()
	plane, err := world.CreateOffsetPlane(world.YZ(), 9.5)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	points := [6]*sketch.Point{
		s.CreatePoint(-9, 0.1), s.CreatePoint(-9, 1.1),
		s.CreatePoint(-9.1, 1.1), s.CreatePoint(-12, 1.1),
		s.CreatePoint(-12, 0.1), s.CreatePoint(-9.1, 0.1),
	}
	center := s.CreatePoint(0, 0.6)
	for _, point := range points {
		s.Fix(point)
	}
	s.Fix(center)
	s.CreateArc(center, points[1], points[0])
	s.CreateLine(points[1], points[2])
	_, err = s.CreateFitSpline(points[2], points[3])
	require.NoError(t, err)
	s.CreateArc(center, points[3], points[4])
	_, err = s.CreateFitSpline(points[4], points[5])
	require.NoError(t, err)
	s.CreateLine(points[5], points[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tooth, err := doc.LoftFromPoint(t.Context(), r3.Vec{}, s, profiles[0])
	require.NoError(t, err)
	toeCone := bevelFitTestRevolve(t, doc, []Point2{
		{U: -2}, {U: -2, V: 16}, {U: 14},
	})
	toeKeeper, err := Cut(t.Context(), tooth, toeCone)
	require.NoError(t, err)
	heelCone := bevelFitTestRevolve(t, doc, []Point2{
		{U: -2}, {U: -2, V: 20}, {U: 18},
	})
	trimmed, err := Intersect(t.Context(), toeKeeper, heelCone)
	require.NoError(t, err)
	tabs := make([]*Body, 6)
	tabs[0] = trimmed
	for i := 1; i < len(tabs); i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1},
			units.Radians(2*math.Pi*float64(i)/float64(len(tabs))))
		require.NoError(t, err)
		tabs[i], err = Copy(t.Context(), trimmed, WithCopyPlacement(turn))
		require.NoError(t, err)
	}
	joined, err := Union(t.Context(), blank, tabs...)
	require.NoError(t, err)
	payload, ok := joined.payload.(facetedPayload)
	require.True(t, ok)
	require.NotNil(t, payload.bevelJoin)
	require.NotNil(t, payload.bevelJoin.capRing)
	require.Len(t, payload.bevelJoin.placed, len(tabs)-1)
	return joined
}

func bevelBoreTestTool(t *testing.T, doc *Document, radius, centerY float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{X: -1, Y: centerY}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	tool, err := doc.Extrude(s, profiles[0], Distance{D: units.Millimeters(12), Dir: Along})
	require.NoError(t, err)
	return tool
}

func bevelBoreTestEuler(mesh *Mesh) int {
	edges := make(map[[2]int]struct{})
	for _, tri := range mesh.Triangles() {
		for i := range 3 {
			a, b := tri[i], tri[(i+1)%3]
			if a > b {
				a, b = b, a
			}
			edges[[2]int{a, b}] = struct{}{}
		}
	}
	return len(mesh.Vertices()) - len(edges) + len(mesh.Triangles())
}

func TestCenteredBoreUsesRetainedBevelJoinGeometryForSixTabs(t *testing.T) {
	doc := New()
	joined := bevelBoreTestJoin(t, doc)
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	call := func(radius, offset float64) (*Body, bool, error) {
		tool := bevelBoreTestTool(t, doc, radius, offset)
		return tryBevelPostJoinBoreCut(t.Context(), meshbool.OpCut, doc, producerID(0), joined, tool)
	}
	for _, radius := range []float64{6.5, 7} {
		_, admitted, err := call(radius, 0)
		require.NoError(t, err)
		require.False(t, admitted)
	}
	_, admitted, err := call(3, 0.1)
	require.NoError(t, err)
	require.False(t, admitted, "a displaced cylinder cannot use the centered rewrite")
	payload := joined.payload.(facetedPayload)
	missingCell := func(group, axis int, radius float64) {
		index := -1
		for i, tri := range payload.tris {
			if payload.src[i] != group {
				continue
			}
			if axis >= 0 && tri[0] != axis && tri[1] != axis && tri[2] != axis {
				continue
			}
			index = i
			break
		}
		require.NotEqual(t, -1, index)
		damaged := payload
		damaged.tris = append(append([][3]int(nil), payload.tris[:index]...), payload.tris[index+1:]...)
		damaged.src = append(append([]int(nil), payload.src[:index]...), payload.src[index+1:]...)
		target := &Body{payload: damaged}
		tool := bevelBoreTestTool(t, doc, radius, 0)
		_, handled, err := tryBevelPostJoinBoreCut(t.Context(), meshbool.OpCut,
			doc, producerID(0), target, tool)
		require.True(t, handled)
		require.ErrorIs(t, err, ErrUnsupported)
	}
	missingCell(1, payload.bevelJoin.capRing.heelAxis, 3)
	missingCell(4, -1, 6.75)
	tooClose := bevelBoreTestTool(t, doc, math.Nextafter(6.5, math.Inf(1)), 0)
	_, err = Cut(t.Context(), joined, tooClose)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, doc.Bodies(), joined, "a refused Cut must leave its target live")
	require.Contains(t, doc.Bodies(), tooClose, "a refused Cut must leave its tool live")
	annularTool := bevelBoreTestTool(t, doc, 3, 0)
	annularBored, err := Cut(t.Context(), joined, annularTool)
	require.NoError(t, err)
	annularMesh, err := annularBored.Tessellate(t.Context(), units.Millimeters(0.1),
		WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, annularMesh.BoundaryVerified())
	require.True(t, annularMesh.VolumeVerified())
	require.Equal(t, 0, bevelBoreTestEuler(annularMesh))
	annularVolume, err := annularBored.Volume()
	require.NoError(t, err)
	require.Less(t, annularVolume.Value.Base()+annularVolume.Bound.Base(),
		joinedVolume.Value.Base()-joinedVolume.Bound.Base())

	toeDoc := New()
	toeJoined := bevelBoreTestJoin(t, toeDoc)
	toeSourceVolume, err := toeJoined.Volume()
	require.NoError(t, err)
	toeTool := bevelBoreTestTool(t, toeDoc, 6.75, 0)
	toeBored, err := Cut(t.Context(), toeJoined, toeTool)
	require.NoError(t, err)
	toeMesh, err := toeBored.Tessellate(t.Context(), units.Millimeters(0.1),
		WithVerification(VerifyAll))
	require.NoError(t, err)
	require.True(t, toeMesh.BoundaryVerified())
	require.True(t, toeMesh.VolumeVerified())
	require.Equal(t, 0, bevelBoreTestEuler(toeMesh))
	toeVolume, err := toeBored.Volume()
	require.NoError(t, err)
	require.Less(t, toeVolume.Value.Base()+toeVolume.Bound.Base(),
		toeSourceVolume.Value.Base()-toeSourceVolume.Bound.Base())
}
