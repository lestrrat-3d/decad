package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBevelCapRefinementKeepsSharedEdgesAndBoundaryGroups(t *testing.T) {
	cone := &pointConeLimit{apex: big.NewRat(20, 1), slope: big.NewRat(1, 1)}
	record := &pointConeTrimRecord{lower: cone, upper: cone}
	nodes := []sectorPathNode{
		{p: r3.NewVec(10, 5, 0)},
		{p: r3.NewVec(10, 6, 0)},
		{p: r3.NewVec(10, 5, 1)},
		{p: r3.NewVec(10, 10, 10)},
	}
	triangles := [][3]int{{0, 1, 2}, {1, 3, 2}}
	edges := []sectorCapEdge{
		{u: 0, v: 1, group: 2},
		{u: 1, v: 3, group: 2},
		{u: 3, v: 2, group: 3},
		{u: 2, v: 0, group: 3},
	}
	bound := func(points []sectorPathNode, tri [3]int) float64 {
		b, _, _, err := pointConeTriangleBound(
			[3]r3.Vec{points[tri[0]].p, points[tri[1]].p, points[tri[2]].p}, cone)
		require.NoError(t, err)
		return b
	}
	small, large := bound(nodes, triangles[0]), bound(nodes, triangles[1])
	require.Less(t, small, large)
	limit := (small + large) / 2

	refined, cells, perimeter, round, err := bevelRefineCap(t.Context(), nodes,
		triangles, edges, record, limit, bevelCombineCapSourceLimit)
	require.NoError(t, err)
	require.Len(t, cells, 6) // Four children of the marked cell; two of its neighbor.
	require.Len(t, perimeter, 6)
	require.Greater(t, round, 0.0)
	require.Equal(t, []int{2, 2, 2, 3, 3, 3}, []int{
		perimeter[0].group, perimeter[1].group, perimeter[2].group,
		perimeter[3].group, perimeter[4].group, perimeter[5].group,
	})

	counts := map[pointConeEdge]int{}
	for _, tri := range cells {
		for i := range 3 {
			counts[orderedPointConeEdge(tri[i], tri[(i+1)%3])]++
		}
	}
	for _, edge := range perimeter {
		require.Equal(t, 1, counts[orderedPointConeEdge(edge.u, edge.v)])
	}
	for edge, count := range counts {
		if count != 1 {
			require.Equal(t, 2, count, "interior edge %v must be shared", edge)
		}
	}
	require.Greater(t, len(refined), len(nodes))

	before := 0.0
	for _, tri := range cells {
		before = max(before, bound(refined, tri))
	}
	twiceNodes, twiceCells, twicePerimeter, twiceRound, err := bevelRefineCap(t.Context(),
		refined, cells, perimeter, record, 0, 4*bevelCombineCapSourceLimit)
	require.NoError(t, err)
	require.Greater(t, twiceRound, 0.0)
	require.Greater(t, len(twiceCells), len(cells))
	require.LessOrEqual(t, len(twiceCells), 4*len(cells))
	require.Len(t, twicePerimeter, 2*len(perimeter))
	for i, edge := range perimeter {
		require.Equal(t, edge.group, twicePerimeter[2*i].group)
		require.Equal(t, edge.group, twicePerimeter[2*i+1].group)
	}
	after := 0.0
	counts = map[pointConeEdge]int{}
	for _, tri := range twiceCells {
		after = max(after, bound(twiceNodes, tri))
		for i := range 3 {
			counts[orderedPointConeEdge(tri[i], tri[(i+1)%3])]++
		}
	}
	require.Less(t, after, before)
	for _, edge := range twicePerimeter {
		require.Equal(t, 1, counts[orderedPointConeEdge(edge.u, edge.v)])
	}
	for edge, count := range counts {
		if count != 1 {
			require.Equal(t, 2, count, "interior edge %v must be shared", edge)
		}
	}
}

func TestBevelPatternSourceProjectionChargesWholeBodyBounds(t *testing.T) {
	turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(180))
	require.NoError(t, err)
	first := &Body{payload: facetedPayload{
		verts:     []r3.Vec{r3.NewVec(0, 4, -1), r3.NewVec(0, 6, 1)},
		meshBound: 0.1, xform: r3.Identity(),
	}}
	second := &Body{payload: facetedPayload{
		verts:     []r3.Vec{r3.NewVec(0, -6, -1), r3.NewVec(0, -4, 1)},
		meshBound: 0.1, xform: turn,
	}}
	separate, err := bevelPatternSourcesSeparate(t.Context(), []*Body{first, second})
	require.NoError(t, err)
	require.True(t, separate)
	second.payload = facetedPayload{
		verts:     []r3.Vec{r3.NewVec(0, -6, -1), r3.NewVec(0, -4, 1)},
		meshBound: 8, xform: turn,
	}
	separate, err = bevelPatternSourcesSeparate(t.Context(), []*Body{first, second})
	require.NoError(t, err)
	require.False(t, separate)
	second.payload = facetedPayload{
		verts:     []r3.Vec{r3.NewVec(0, -6, -1), r3.NewVec(0, -4, 1)},
		meshBound: math.MaxFloat64, xform: turn,
	}
	separate, err = bevelPatternSourcesSeparate(t.Context(), []*Body{first, second})
	require.NoError(t, err)
	require.False(t, separate)
}

func TestBevelSourceRequiresPositivePublicConeBranch(t *testing.T) {
	blank := &bevelBlankSource{
		toeRoot: Point2{U: 1, V: 1}, ded: Point2{U: 10, V: 10},
		rootCone: Cone{Origin: r3.NewVec(5, 0, 0), Axis: r3.NewVec(1, 0, 0),
			HalfAngle: units.Degrees(45)},
	}
	base := facetedPayload{meshBound: 0.1, verts: []r3.Vec{
		{}, r3.NewVec(4, 4, 0), r3.NewVec(5, 5, 0), r3.NewVec(6, 6, 0),
	}}
	_, ok := bevelImplicitDisplacement(blank, base)
	require.False(t, ok)
	blank.rootCone.Origin.X = 3
	_, ok = bevelImplicitDisplacement(blank, base)
	require.True(t, ok)
}

func TestBevelRootEdgeOrderRequiresOneDirectedInterval(t *testing.T) {
	nodes := []sectorPathNode{
		{angle: 0, rootBoundary: true},
		{angle: 0.3, rootBoundary: true},
		{angle: 0.2, rootBoundary: true},
		{angle: 0.1, rootBoundary: true},
	}
	edges := []sectorCapEdge{
		{u: 0, v: 1, group: 4},
		{u: 1, v: 2, group: -1},
		{u: 2, v: 3, group: -1},
		{u: 3, v: 0, group: -1},
	}
	require.True(t, bevelRootEdgeOrdered(nodes, edges, 0, 0.3))
	nodes[2].angle = 0.05
	require.False(t, bevelRootEdgeOrdered(nodes, edges, 0, 0.3))
}
