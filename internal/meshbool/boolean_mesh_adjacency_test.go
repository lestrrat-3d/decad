package meshbool_test

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

// adjacencyOracle preserves the original map and per-facet append algorithm.
func adjacencyOracle(tris [][3]int) [][]int {
	owner := map[[2]int]int{}
	for i, tri := range tris {
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	adj := make([][]int, len(tris))
	for i, tri := range tris {
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj
}

func adjacencyPairedTriangles(n int) [][3]int {
	tris := make([][3]int, 0, 2*n)
	for i := range n {
		v := 4 * i
		tris = append(tris, [3]int{v, v + 1, v + 2}, [3]int{v + 1, v, v + 3})
	}
	return tris
}

func TestFacetAdjacencyMatchesOriginalOrderAndNilSlices(t *testing.T) {
	cases := map[string][][3]int{
		"empty":       nil,
		"open":        {{0, 1, 2}},
		"one twin":    {{0, 1, 2}, {1, 0, 3}},
		"closed":      {{0, 1, 2}, {1, 0, 3}, {2, 1, 3}, {0, 2, 3}},
		"duplicates":  {{0, 1, 2}, {0, 1, 3}, {1, 0, 4}},
		"degenerate":  {{0, 0, 1}, {1, 0, 0}},
		"many facets": adjacencyPairedTriangles(256),
	}
	for name, tris := range cases {
		t.Run(name, func(t *testing.T) {
			before := append([][3]int(nil), tris...)
			got, err := meshbool.FacetAdjacencyContext(t.Context(), tris)
			require.NoError(t, err)
			require.Equal(t, adjacencyOracle(tris), got)
			require.Equal(t, before, tris)
		})
	}
}

type adjacencyCancelContext struct {
	polls  int
	stopAt int
	err    error
}

func (*adjacencyCancelContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (*adjacencyCancelContext) Done() <-chan struct{}       { return nil }
func (*adjacencyCancelContext) Value(any) any               { return nil }

func (c *adjacencyCancelContext) Err() error {
	c.polls++
	if c.polls == c.stopAt {
		return c.err
	}
	return nil
}

func TestFacetAdjacencyKeepsBothPhaseCancellationPolls(t *testing.T) {
	tris := adjacencyPairedTriangles(256)
	stopped := errors.New("stopped")
	for stopAt := 1; stopAt <= 7; stopAt++ {
		ctx := &adjacencyCancelContext{stopAt: stopAt, err: stopped}
		got, err := meshbool.FacetAdjacencyContext(ctx, tris)
		require.ErrorIs(t, err, stopped)
		require.Nil(t, got)
		require.Equal(t, stopAt, ctx.polls)
	}
}

func TestFacetAdjacencyPreCanceledAvoidsReservation(t *testing.T) {
	tris := make([][3]int, 8192)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	var got [][]int
	var err error
	// AllocsPerRun counts every allocation in the process, so another
	// goroutine's stray allocation can land in one reading. A reservation by
	// FacetAdjacencyContext lands in every reading, so the lowest one decides.
	allocs := math.MaxFloat64
	for range 5 {
		allocs = min(allocs, testing.AllocsPerRun(1, func() {
			got, err = meshbool.FacetAdjacencyContext(ctx, tris)
		}))
		if allocs == 0 {
			break
		}
	}
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
	require.Zero(t, allocs)
}
