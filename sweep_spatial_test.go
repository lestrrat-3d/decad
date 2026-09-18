package decad_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestSweepSpatialPathTurnsThroughOrthogonalPlanes(t *testing.T) {
	t.Parallel()

	s, profile, path, vertices, joins := orthogonalSweepFixture(t)
	doc := decad.New()
	body, err := doc.Sweep(s, profile, path)
	require.NoError(t, err)

	require.True(t, body.IsSolid())
	require.Len(t, body.Lumps(), 1)
	require.Len(t, body.Shells(), 1)
	require.Len(t, body.Faces(), 14)
	require.Len(t, body.Edges(), 28)
	require.Len(t, body.Vertices(), 16)
	decadtest.HasSurfaceKinds(t, body, map[decad.SurfaceKind]int{
		decad.KindPlane:    10,
		decad.KindCylinder: 4,
	})
	for _, edge := range body.Edges() {
		require.Len(t, edge.Faces(), 2)
	}
	requireSpatialSweepRoles(t, body)
	requireSpatialSweepVertices(t, body, vertices)
	requireSpatialSweepJoinContinuity(t, body, joins[0], 0, 1)
	requireSpatialSweepJoinContinuity(t, body, joins[1], 1, 2)

	wantVolume := units.CubicMillimeters(40 + 20*math.Pi)
	wantArea := units.SquareMillimeters(88 + 40*math.Pi)
	c := (150*math.Pi - 304) / (60*math.Pi + 120)
	wantCentroid := r3.NewVec(
		10,
		c,
		5-c,
	)
	decadtest.MeasuresVolume(t, body, wantVolume)
	decadtest.MeasuresArea(t, body, wantArea)
	decadtest.MeasuresCentroid(t, body, wantCentroid)
	decadtest.MeasuresBounds(t, body, r3.NewVec(-1, -1, 0), r3.NewVec(21, 5, 6))

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Passed())
	bodyReport, err := report.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, bodyReport.Status)
	require.Equal(t, decad.ValidityValid, bodyReport.Validity.Outcome)

	turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(90))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(20, -10, 3))
	require.NoError(t, err)
	motion, err := turn.Then(shift)
	require.NoError(t, err)
	placed, err := body.Placed(motion)
	require.NoError(t, err)

	placedVertices := make([]r3.Vec, len(vertices))
	for i, vertex := range vertices {
		placedVertices[i] = motion.Apply(vertex)
	}
	placedJoins := make([][]r3.Vec, len(joins))
	for i, join := range joins {
		placedJoins[i] = make([]r3.Vec, len(join))
		for j, vertex := range join {
			placedJoins[i][j] = motion.Apply(vertex)
		}
	}
	requireSpatialSweepRoles(t, placed)
	requireSpatialSweepVertices(t, placed, placedVertices)
	requireSpatialSweepJoinContinuity(t, placed, placedJoins[0], 0, 1)
	requireSpatialSweepJoinContinuity(t, placed, placedJoins[1], 1, 2)
	decadtest.MeasuresVolume(t, placed, wantVolume)
	decadtest.MeasuresArea(t, placed, wantArea)
	decadtest.MeasuresCentroid(t, placed, motion.Apply(wantCentroid))
	decadtest.MeasuresBounds(t, placed, r3.NewVec(15, -11, 3), r3.NewVec(21, 11, 9))
	require.Equal(t, []*decad.Body{placed}, doc.Bodies())

	placedReport, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, placedReport.Status)
	require.True(t, placedReport.Passed())
	placedBodyReport, err := placedReport.ForBody(placed)
	require.NoError(t, err)
	require.Equal(t, decad.Sound, placedBodyReport.Status)
	require.Equal(t, decad.ValidityValid, placedBodyReport.Validity.Outcome)
}

func TestSweepSpatialPathPreservesProfileHole(t *testing.T) {
	t.Parallel()

	s, profile := orthogonalSweepProfileWithHole(t)
	path := orthogonalSweepPath(t)
	doc := decad.New()
	body, err := doc.Sweep(s, profile, path)
	require.NoError(t, err)
	require.Len(t, body.Faces(), 17)
	for _, edge := range body.Edges() {
		require.Len(t, edge.Faces(), 2)
	}

	pathLength := 10 + 5*math.Pi
	profileArea := 4 - math.Pi/4
	profilePerimeter := 8 + math.Pi
	decadtest.MeasuresVolume(t, body, units.CubicMillimeters(profileArea*pathLength))
	decadtest.MeasuresArea(t, body, units.SquareMillimeters(2*profileArea+profilePerimeter*pathLength))

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	bodyReport, err := report.ForBody(body)
	require.NoError(t, err)
	require.Equal(t, decad.ValidityValid, bodyReport.Validity.Outcome)
}

func orthogonalSweepFixture(t *testing.T) (*sketch.Sketch, *sketch.Profile, *decad.Path, []r3.Vec, [][]r3.Vec) {
	t.Helper()

	s, profile := orthogonalSweepProfile(t)
	path := orthogonalSweepPath(t)

	joins := [][]r3.Vec{
		{
			r3.NewVec(5, -1, 6),
			r3.NewVec(5, -1, 4),
			r3.NewVec(5, 1, 4),
			r3.NewVec(5, 1, 6),
		},
		{
			r3.NewVec(15, -1, 6),
			r3.NewVec(15, -1, 4),
			r3.NewVec(15, 1, 4),
			r3.NewVec(15, 1, 6),
		},
	}
	vertices := []r3.Vec{
		r3.NewVec(-1, -1, 0),
		r3.NewVec(1, -1, 0),
		r3.NewVec(1, 1, 0),
		r3.NewVec(-1, 1, 0),
	}
	vertices = append(vertices, joins[0]...)
	vertices = append(vertices, joins[1]...)
	vertices = append(vertices,
		r3.NewVec(21, 5, 6),
		r3.NewVec(21, 5, 4),
		r3.NewVec(19, 5, 4),
		r3.NewVec(19, 5, 6),
	)
	return s, profile, path, vertices, joins
}

func orthogonalSweepPath(t *testing.T) *decad.Path {
	t.Helper()

	path, err := decad.NewPath(
		r3.NewVec(0, 0, 0),
		decad.ArcThrough{
			Through: r3.NewVec(2, 0, 4),
			End:     r3.NewVec(5, 0, 5),
		},
		decad.LineTo{End: r3.NewVec(15, 0, 5)},
		decad.ArcThrough{
			Through: r3.NewVec(18, 1, 5),
			End:     r3.NewVec(20, 5, 5),
		},
	)
	require.NoError(t, err)
	return path
}

func orthogonalSweepProfile(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()

	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

func orthogonalSweepProfileWithHole(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()

	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(-1, -1, 1, 1)
	s.Fix(rect.A)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 0.5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	for _, profile := range s.Profiles() {
		if profile.Valid && len(profile.Holes) == 1 {
			return s, profile
		}
	}
	t.Fatal("sketch produced no valid profile with one hole")
	return nil, nil
}

func requireSpatialSweepRoles(t *testing.T, body *decad.Body) {
	t.Helper()

	roles := make([]string, 0, len(body.Faces()))
	for _, face := range body.Faces() {
		origins := face.Origins()
		require.Len(t, origins, 1)
		roles = append(roles, origins[0].Role)
	}
	require.ElementsMatch(t, []string{
		"capStart",
		"capEnd",
		"side(0,0,0)",
		"side(0,0,1)",
		"side(0,0,2)",
		"side(0,0,3)",
		"side(1,0,0)",
		"side(1,0,1)",
		"side(1,0,2)",
		"side(1,0,3)",
		"side(2,0,0)",
		"side(2,0,1)",
		"side(2,0,2)",
		"side(2,0,3)",
	}, roles)
}

func requireSpatialSweepVertices(t *testing.T, body *decad.Body, want []r3.Vec) {
	t.Helper()

	vertices := body.Vertices()
	require.Len(t, vertices, len(want))
	matched := make([]bool, len(want))
	for _, vertex := range vertices {
		position := vertex.Position()
		found := -1
		for i, point := range want {
			if matched[i] || !sweepPositionMatches(position, point) {
				continue
			}
			found = i
			break
		}
		require.NotEqual(t, -1, found, "unexpected sweep vertex %v with bound %s", position.Value, position.Bound)
		matched[found] = true
	}
	require.NotContains(t, matched, false)
}

func requireSpatialSweepJoinContinuity(t *testing.T, body *decad.Body, join []r3.Vec, before, after int) {
	t.Helper()

	joinEdges := 0
	for _, edge := range body.Edges() {
		if !sweepPositionMatchesAny(edge.Start().Position(), join) ||
			!sweepPositionMatchesAny(edge.End().Position(), join) {
			continue
		}
		joinEdges++
		faces := edge.Faces()
		require.Len(t, faces, 2)
		var beforeSuffix, afterSuffix string
		for _, face := range faces {
			origins := face.Origins()
			require.Len(t, origins, 1)
			role := origins[0].Role
			if suffix, ok := strings.CutPrefix(role, fmt.Sprintf("side(%d,", before)); ok {
				beforeSuffix = suffix
			}
			if suffix, ok := strings.CutPrefix(role, fmt.Sprintf("side(%d,", after)); ok {
				afterSuffix = suffix
			}
		}
		require.NotEmpty(t, beforeSuffix)
		require.NotEmpty(t, afterSuffix)
		require.Equal(t, beforeSuffix, afterSuffix)
	}
	require.Equal(t, 4, joinEdges)
}

func sweepPositionMatchesAny(got decad.VecMeasurement, want []r3.Vec) bool {
	for _, point := range want {
		if sweepPositionMatches(got, point) {
			return true
		}
	}
	return false
}

func sweepPositionMatches(got decad.VecMeasurement, want r3.Vec) bool {
	return got.Value.Sub(want).Len() <= got.Bound.Base()+1e-12
}
