package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Untouched geometry keeps its own bound after several Boolean unions.
func TestFacetedUnionKeepsUntouchedVertexAndFaceBounds(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ts, tp := polygonSweepSketch(t, 16, 5)
	trunk, err := decad.Sweep(t.Context(), doc, ts, tp,
		mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 30), r3.NewVec(3, 0, 60)),
		decad.WithMitredJoins(), decad.WithSectionScale(scalars(1, 0.9)...))
	require.NoError(t, err)
	trunkBounds := make(map[r3.Vec]units.Value)
	for _, vertex := range trunk.Vertices() {
		position := vertex.Position()
		trunkBounds[position.Value] = position.Bound
	}

	bs, bp := polygonSweepSketch(t, 16, 1.5)
	golden := math.Pi * (3 - math.Sqrt(5))
	alpha := 40 * math.Pi / 180
	result := trunk
	for k := range 3 {
		phi := golden * float64(k)
		u := r3.NewVec(math.Cos(phi), math.Sin(phi), 0)
		branch, err := decad.Sweep(t.Context(), doc, bs, bp,
			mustPath(t, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 2.5), r3.NewVec(0.8, 0.3, 6),
				r3.NewVec(2.5, 0.6, 9.5)),
			decad.WithMitredJoins(), decad.WithSectionScale(scalars(1.375/1.5, 1.2/1.5, 1/1.5)...))
		require.NoError(t, err)
		direction := u.Scale(math.Sin(alpha)).Add(r3.NewVec(0, 0, math.Cos(alpha)))
		forward := r3.NewVec(0, 0, 1).Cross(direction)
		forward = forward.Scale(1 / forward.Len())
		frame, err := r3.NewFrame(u.Scale(2.6).Add(r3.NewVec(0, 0, 4+1.1*float64(k))),
			forward, direction.Cross(forward))
		require.NoError(t, err)
		placement, err := r3.FromFrame(frame)
		require.NoError(t, err)
		branch, err = decad.Transform(t.Context(), branch, placement)
		require.NoError(t, err)
		result, err = decad.Union(t.Context(), result, branch)
		require.NoError(t, err)
	}

	mesh, err := result.Tessellate(t.Context(), units.Millimeters(1))
	require.NoError(t, err)
	resultBounds := make(map[r3.Vec]units.Value)
	kept, tighter := 0, 0
	for _, vertex := range result.Vertices() {
		position := vertex.Position()
		resultBounds[position.Value] = position.Bound
		want, ok := trunkBounds[position.Value]
		if !ok {
			continue
		}
		require.LessOrEqual(t, position.Bound.Mag(), want.Mag(), "untouched trunk vertex %v", position.Value)
		kept++
		if position.Bound.Mag() < mesh.Bound().Mag() {
			tighter++
		}
	}
	require.Positive(t, kept)
	require.Positive(t, tighter)

	type faceReading struct {
		bound    units.Value
		allTrunk bool
	}
	readings := make(map[*decad.Face]faceReading)
	vertices, triangles, sources := mesh.Vertices(), mesh.Triangles(), mesh.SourceFaces()
	for i, triangle := range triangles {
		face := sources[i]
		reading, exists := readings[face]
		if !exists {
			reading.allTrunk = true
			reading.bound = units.Millimeters(0)
		}
		for _, index := range triangle {
			_, ok := trunkBounds[vertices[index]]
			if !ok {
				reading.allTrunk = false
				continue
			}
			bound, ok := resultBounds[vertices[index]]
			require.True(t, ok, "mesh vertex %v belongs to the body", vertices[index])
			if bound.Mag() > reading.bound.Mag() {
				reading.bound = bound
			}
		}
		readings[face] = reading
	}
	untouchedFaces, tighterFaces := 0, 0
	for face, reading := range readings {
		if !reading.allTrunk {
			continue
		}
		// An untouched trunk face keeps its operand's surface, so its
		// displacement reads through its vertices, not Faceted.Bound.
		require.NotEqual(t, decad.KindFaceted, face.Surface().Kind())
		untouchedFaces++
		if reading.bound.Mag() < mesh.Bound().Mag() {
			tighterFaces++
		}
	}
	require.Positive(t, untouchedFaces)
	require.Positive(t, tighterFaces)

	box, err := result.Bounds()
	require.NoError(t, err)
	require.Positive(t, box.Bound.Mag())
	require.Less(t, box.Bound.Mag(), math.Sqrt(3)*mesh.Bound().Mag(),
		"the box uses its extreme vertices instead of the body-wide bound")
}

// A placement charges rounding at each vertex's own coordinate magnitude.
func TestFacetedPlacementKeepsNearOriginVertexBoundTight(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, 0, 0, 20, 20, 8)
	tool := translated(t, diskBody(t, doc, 0, 0, 2), 10, 10, -10)
	drilled, err := decad.Cut(t.Context(), plate, tool)
	require.NoError(t, err)
	origin, far := r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 8)
	sourceBounds := make(map[r3.Vec]units.Value)
	for _, vertex := range drilled.Vertices() {
		position := vertex.Position()
		sourceBounds[position.Value] = position.Bound
	}
	require.Contains(t, sourceBounds, origin)
	require.Contains(t, sourceBounds, far)
	require.Zero(t, sourceBounds[origin].Mag())
	require.Zero(t, sourceBounds[far].Mag())

	rotation, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1, 2, 3))
	require.NoError(t, err)
	motion, err := rotation.Then(shift)
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), drilled, motion)
	require.NoError(t, err)
	boundAt := func(target r3.Vec) units.Value {
		for _, vertex := range placed.Vertices() {
			position := vertex.Position()
			if position.Value.Sub(target).Len() < 1e-10 {
				return position.Bound
			}
		}
		t.Fatalf("placed vertex near %v is missing", target)
		return units.Millimeters(0)
	}
	nearBound := boundAt(motion.Apply(origin))
	farBound := boundAt(motion.Apply(far))
	require.Positive(t, nearBound.Mag())
	require.Less(t, 2*nearBound.Mag(), farBound.Mag(),
		"the origin vertex pays less rounding than a distant exact corner")
}
