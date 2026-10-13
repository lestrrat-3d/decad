package loftmesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferBoxGridForkReplayMatchesSharedGrid(t *testing.T) {
	const facets, perFacet = 40, 300
	rng := rand.New(rand.NewPCG(53, 59))
	box := func(span float64) [2]r3.Vec {
		x, y := rng.Float64()*8, rng.Float64()*8
		return [2]r3.Vec{{X: x, Y: y, Z: 0.25}, {X: x + rng.Float64()*span, Y: y + rng.Float64()*span, Z: 0.75}}
	}
	boxes := make([][2]r3.Vec, 2000)
	for i := range boxes {
		boxes[i] = box(0.6)
	}
	queries := make([][][2]r3.Vec, facets)
	for f := range queries {
		for range perFacet {
			queries[f] = append(queries[f], box(1.5))
		}
	}
	build := func(maxWork int) *ChamferBoxGrid {
		grid, err := ChamferBoxGridOf(boxes, 0, 1)
		require.NoError(t, err)
		grid.LimitWork(maxWork)
		return grid
	}
	// Measure the unlimited work of the first 25 facets, then cut the budget
	// inside facet 25 so the shared grid refuses there.
	probe := build(1 << 40)
	for f := range 25 {
		for _, q := range queries[f] {
			_, err := probe.candidates(q)
			require.NoError(t, err)
		}
	}
	maxWork := probe.work + 300

	type result struct{ lists [][]int }
	shared := build(maxWork)
	serial := make([]result, facets)
	refusedFacet, refusedQuery := -1, -1
	for f := 0; f < facets && refusedFacet < 0; f++ {
		for qi, q := range queries[f] {
			list, err := shared.candidates(q)
			if err != nil {
				refusedFacet, refusedQuery = f, qi
				break
			}
			serial[f].lists = append(serial[f].lists, list)
		}
	}
	require.GreaterOrEqual(t, refusedFacet, 1)

	shared2 := build(maxWork)
	forks := []*ChamferBoxGrid{shared2.Fork(), shared2.Fork(), shared2.Fork()}
	forked := make([]result, facets)
	logs := make([][]ChamferGridQuery, facets)
	for f := range facets {
		fork := forks[f%len(forks)]
		for _, q := range queries[f] {
			list, err := fork.candidates(q)
			if err != nil {
				list = nil
			}
			forked[f].lists = append(forked[f].lists, list)
		}
		logs[f] = fork.TakeLog()
		require.Len(t, logs[f], perFacet)
		require.Empty(t, fork.TakeLog())
	}
	gotFacet, gotQuery := -1, -1
	for f := range facets {
		if !shared2.Replay(logs[f]) {
			gotFacet = f
			for qi, entry := range logs[f] {
				if !shared2.Replay([]ChamferGridQuery{entry}) {
					gotQuery = qi
					break
				}
			}
			break
		}
		require.Equal(t, serial[f].lists, forked[f].lists, "facet %d", f)
	}
	require.Equal(t, refusedFacet, gotFacet)
	require.Equal(t, refusedQuery, gotQuery)
	// Re-running the refused query on the shared2 grid refuses the same way and
	// leaves the same count the shared grid has.
	_, err := shared2.candidates(queries[gotFacet][gotQuery])
	require.Error(t, err)
	require.Equal(t, shared.work, shared2.work)
}
