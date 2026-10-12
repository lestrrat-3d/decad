package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSectorCapRefinementKeepsSharedEdges(t *testing.T) {
	cone := &pointConeLimit{apex: big.NewRat(20, 1), slope: big.NewRat(1, 1)}
	record := &pointConeTrimRecord{lower: cone, upper: cone}
	nodes := []sectorPathNode{
		{p: r3.NewVec(10, 5, 0), rootBoundary: true, angle: 0},
		{p: r3.NewVec(10, 6, 0), rootBoundary: true, angle: 0.1},
		{p: r3.NewVec(10, 5, 1)},
		{p: r3.NewVec(10, 10, 10)},
	}
	triangles := [][3]int{{0, 1, 2}, {1, 3, 2}}
	edges := []sectorCapEdge{
		{u: 0, v: 1, group: -1},
		{u: 1, v: 3, group: 2},
		{u: 3, v: 2, group: 3},
		{u: 2, v: 0, group: 3},
	}
	before, _, err := sectorConeCapBounds(nodes, triangles, record)
	require.NoError(t, err)
	limit := before * 0.01
	refined, cells, perimeter, round, err := sectorRefineCap(t.Context(), nodes,
		triangles, edges, record, limit, 32)
	require.NoError(t, err)
	require.Greater(t, round, 0.0)
	require.Greater(t, len(cells), len(triangles))
	after, _, err := sectorConeCapBounds(refined, cells, record)
	require.NoError(t, err)
	require.Less(t, after, before)
	require.Equal(t, -1, perimeter[0].group)
	require.Equal(t, -1, perimeter[1].group)
	require.True(t, refined[perimeter[0].v].rootBoundary)
	require.Equal(t, 0.05, refined[perimeter[0].v].angle)

	counts := map[pointConeEdge]int{}
	for _, tri := range cells {
		for i := range 3 {
			counts[orderedPointConeEdge(tri[i], tri[(i+1)%3])]++
		}
	}
	for _, edge := range perimeter {
		require.Equal(t, 1, counts[orderedPointConeEdge(edge.u, edge.v)])
	}
	for _, count := range counts {
		require.LessOrEqual(t, count, 2)
	}
	refusedNodes, refusedCells, refusedEdges, refusedRound, err := sectorRefineCap(t.Context(), nodes,
		triangles, edges, record, limit, 5)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Nil(t, refusedNodes)
	require.Nil(t, refusedCells)
	require.Nil(t, refusedEdges)
	require.Zero(t, refusedRound)
}
