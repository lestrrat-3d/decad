package meshbool

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// prismMesh is the closed prism over the counter-clockwise XY polygon poly
// from z0 to z1, with fan caps about the polygon's vertex mean.
func prismMesh(t *testing.T, poly []r3.Vec, z0, z1 float64) *BoolMesh {
	t.Helper()
	n := len(poly)
	var verts []r3.Vec
	var mean r3.Vec
	for _, p := range poly {
		verts = append(verts, r3.Vec{X: p.X, Y: p.Y, Z: z0}, r3.Vec{X: p.X, Y: p.Y, Z: z1})
		mean = mean.Add(p)
	}
	mean = mean.Scale(1 / float64(n))
	verts = append(verts, r3.Vec{X: mean.X, Y: mean.Y, Z: z0}, r3.Vec{X: mean.X, Y: mean.Y, Z: z1})
	c0, c1 := 2*n, 2*n+1
	var tris [][3]int
	for k := range n {
		a0, a1 := 2*k, 2*k+1
		b0, b1 := 2*((k+1)%n), 2*((k+1)%n)+1
		tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1}, [3]int{c0, b0, a0}, [3]int{c1, a1, b1})
	}
	m, err := PrepContext(t.Context(), verts, tris, nil)
	require.NoError(t, err)
	m.SetVertexBounds(make([]float64, len(verts)))
	return m
}

// facetsOfSide returns the two side facets prismMesh built for polygon edge k.
func facetsOfSide(k int) []int { return []int{4 * k, 4*k + 1} }

// TestNearestWitnessFindsDepthAcrossALongFacet sets a 20 mm prism wall,
// leaning 0.05 in 1 off the plane x = 0 and touching it along z, against a
// box whose face lies in that plane and which is 0.5 mm thin along z. The
// wall's facet runs 2 mm deeper than the bound inside the box across from
// the touch, but its fixed samples lie outside the box and the walk from the
// touch toward its far corners leaves the box before it is deeper than the
// bound. The nearest-point search finds and certifies the depth. With the box
// on the other side of the plane the wall only touches it, and no search
// admits the pair.
func TestNearestWitnessFindsDepthAcrossALongFacet(t *testing.T) {
	t.Parallel()
	const slack = 0.002
	// The edge from (0.05, 1) to (0, 0), polygon edge 3, is the leaning wall.
	wall := prismMesh(t, []r3.Vec{{}, {X: 2}, {X: 2, Y: 1}, {X: 0.05, Y: 1}}, -10, 10)
	testcases := []struct {
		Name string
		Box  []r3.Vec
		Deep bool
	}{
		{Name: "box beside the wall", Box: []r3.Vec{{Y: -1}, {X: 3, Y: -1}, {X: 3, Y: 2}, {Y: 2}}, Deep: true},
		{Name: "box outside the wall", Box: []r3.Vec{{X: -3, Y: -1}, {Y: -1}, {Y: 2}, {X: -3, Y: 2}}, Deep: false},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			box := prismMesh(t, tc.Box, 5, 5.5)
			// The box's face in the plane x = 0 is polygon edge 3 beside the
			// wall and edge 1 outside it.
			side := 3
			if !tc.Deep {
				side = 1
			}
			memo := NewContactMemo(wall, box)
			nc, deferred, err := GatherNearContacts(t.Context(), wall, facetsOfSide(3), box, facetsOfSide(side), slack, memo)
			require.NoError(t, err)
			require.False(t, deferred)
			require.NotEmpty(t, nc.CloseA)
			require.NotEmpty(t, nc.CloseB)

			old, err := refProvenDepthExceedsWithoutNearest(t, wall, box, nc, slack)
			require.NoError(t, err)
			require.False(t, old, "the fixed samples and the span walk find no witness")

			got, err := ProvenDepthExceeds(t.Context(), wall, nc.CloseA, box, nc.CloseB, nc.Spans, slack)
			require.NoError(t, err)
			require.Equal(t, tc.Deep, got)
		})
	}
}

// refProvenDepthExceedsWithoutNearest is ProvenDepthExceeds without its
// nearest-point search.
func refProvenDepthExceedsWithoutNearest(t *testing.T, a, b *BoolMesh, nc NearContacts, slack float64) (bool, error) {
	t.Helper()
	deep, err := DeepWitnessInside(t.Context(), a, nc.CloseA, b, slack)
	if err != nil || deep {
		return deep, err
	}
	deep, err = DeepWitnessInside(t.Context(), b, nc.CloseB, a, slack)
	if err != nil || deep {
		return deep, err
	}
	return SpanWitness(t.Context(), a, b, nc.Spans, slack)
}

// TestNearestFacetWeightsStayOnTheFacet checks that the exact point the
// weights build lies on the closed facet, within float rounding of the
// nearest point, for queries over each corner, edge and the face, and that
// a query the floats cannot place yields no weights.
func TestNearestFacetWeightsStayOnTheFacet(t *testing.T) {
	t.Parallel()
	tri := [3]r3.Vec{{}, {X: 4}, {Y: 2, Z: 1}}
	testcases := []struct {
		Name string
		Q    r3.Vec
		Want r3.Vec
	}{
		{Name: "corner 0", Q: r3.Vec{X: -1, Y: -1}, Want: tri[0]},
		{Name: "corner 1", Q: r3.Vec{X: 6, Y: -1}, Want: tri[1]},
		{Name: "corner 2", Q: r3.Vec{X: -1, Y: 4, Z: 2}, Want: tri[2]},
		{Name: "edge 01", Q: r3.Vec{X: 1.5, Y: -3}, Want: r3.Vec{X: 1.5}},
		{Name: "face", Q: r3.Vec{X: 1, Y: 0.8, Z: 0.4}.Add(r3.Vec{Y: -1, Z: 2}), Want: r3.Vec{X: 1, Y: 0.8, Z: 0.4}},
	}
	x := [3]proof.Xpt{proof.XptOf(tri[0]), proof.XptOf(tri[1]), proof.XptOf(tri[2])}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			inner, outer, ok := nearestFacetWeights(tc.Q, tri)
			require.True(t, ok)
			for _, w := range []*big.Int{inner, outer} {
				require.GreaterOrEqual(t, w.Sign(), 0)
				require.LessOrEqual(t, w.Cmp(nearestWeightDen), 0)
			}
			p := proof.Xlerp(proof.Xlerp(x[0], x[1], inner, nearestWeightDen), x[2], outer, nearestWeightDen).Vec()
			require.InDelta(t, tc.Want.X, p.X, 1e-9)
			require.InDelta(t, tc.Want.Y, p.Y, 1e-9)
			require.InDelta(t, tc.Want.Z, p.Z, 1e-9)
		})
	}
	_, _, ok := nearestFacetWeights(r3.Vec{X: math.NaN()}, tri)
	require.False(t, ok)
}

// TestNearestWitnessReplayAgreesWithPlain runs the leaning wall's two cases
// through the replay path's witness search, under a replay budget, and
// requires the plain search's verdict: whole and through the nearest-point
// step alone.
func TestNearestWitnessReplayAgreesWithPlain(t *testing.T) {
	t.Parallel()
	const slack = 0.002
	wall := prismMesh(t, []r3.Vec{{}, {X: 2}, {X: 2, Y: 1}, {X: 0.05, Y: 1}}, -10, 10)
	testcases := []struct {
		Name string
		Box  []r3.Vec
		Side int
		Deep bool
	}{
		{Name: "box beside the wall", Box: []r3.Vec{{Y: -1}, {X: 3, Y: -1}, {X: 3, Y: 2}, {Y: 2}}, Side: 3, Deep: true},
		{Name: "box outside the wall", Box: []r3.Vec{{X: -3, Y: -1}, {Y: -1}, {Y: 2}, {X: -3, Y: 2}}, Side: 1, Deep: false},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			box := prismMesh(t, tc.Box, 5, 5.5)
			memo := NewContactMemo(wall, box)
			nc, deferred, err := GatherNearContacts(t.Context(), wall, facetsOfSide(3), box, facetsOfSide(tc.Side), slack, memo)
			require.NoError(t, err)
			require.False(t, deferred)

			plain, err := ProvenDepthExceeds(t.Context(), wall, nc.CloseA, box, nc.CloseB, nc.Spans, slack)
			require.NoError(t, err)
			require.Equal(t, tc.Deep, plain)
			plainNearest, err := NearestWitness(t.Context(), wall, nc.CloseA, box, nc.CloseB, slack)
			require.NoError(t, err)
			require.Equal(t, tc.Deep, plainNearest)

			replay, err := provenDepthExceedsShared(t.Context(), wall, nc.CloseA, box, nc.CloseB, nc.Spans, slack,
				proofbound.NewReplayTrialBudget().WorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, plain, replay)
			replayNearest, err := nearestWitnessShared(t.Context(), wall, nc.CloseA, box, nc.CloseB, slack,
				proofbound.NewReplayTrialBudget().WorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, plainNearest, replayNearest)
		})
	}
}
