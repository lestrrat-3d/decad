package meshbool

import (
	"context"
	"errors"
	"fmt"
	"runtime"
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func poolTriangles(n, offset int) [][3]int {
	tris := make([][3]int, n)
	for i := range tris {
		v := offset + 4*(i/2)
		if i%2 == 0 {
			tris[i] = [3]int{v, v + 1, v + 2}
		} else {
			tris[i] = [3]int{v + 1, v, v + 3}
		}
	}
	return tris
}

func adjacencyScalarDifference(want, got [][]int) string {
	if (want == nil) != (got == nil) || len(want) != len(got) {
		return fmt.Sprintf("outer nil/length mismatch: %t/%d, %t/%d", want == nil, len(want), got == nil, len(got))
	}
	for i := range want {
		if (want[i] == nil) != (got[i] == nil) || len(want[i]) != len(got[i]) {
			return fmt.Sprintf("row %d nil/length mismatch", i)
		}
		for j := range want[i] {
			if want[i][j] != got[i][j] {
				return fmt.Sprintf("row %d neighbor %d: %d != %d", i, j, want[i][j], got[i][j])
			}
		}
	}
	return ""
}

func TestFacetOwnerPoolScalarEquivalence(t *testing.T) {
	for _, n := range []int{0, 1, 2, 255, 256, 257, 8191, 8192, 8193, 2, 8192, 1} {
		tris := poolTriangles(n, n*10)
		before := append([][3]int(nil), tris...)
		want, err := unpooledFacetAdjacency(t.Context(), tris)
		require.NoError(t, err)
		for pass := range 3 {
			if pass == 2 {
				runtime.GC()
			}
			got, err := FacetAdjacencyContext(t.Context(), tris)
			require.NoError(t, err)
			require.Empty(t, adjacencyScalarDifference(want, got), "size %d pass %d", n, pass)
			for i := range tris {
				for j := range 3 {
					require.Equal(t, before[i][j], tris[i][j])
				}
			}
		}
	}
	for _, tris := range [][][3]int{
		{{0, 1, 2}, {1, 0, 3}, {2, 1, 3}, {0, 2, 3}},
		{{0, 1, 2}, {0, 1, 3}, {1, 0, 4}},
		{{0, 0, 1}, {1, 0, 0}},
	} {
		want, err := unpooledFacetAdjacency(t.Context(), tris)
		require.NoError(t, err)
		got, err := FacetAdjacencyContext(t.Context(), tris)
		require.NoError(t, err)
		require.Empty(t, adjacencyScalarDifference(want, got))
	}
}

func TestFacetOwnerPoolBudgetAndReset(t *testing.T) {
	var pool sync.Pool
	for _, n := range []int{-1, 0, 8193, int(^uint(0) >> 1)} {
		require.Nil(t, acquireFacetOwners(&pool, n))
	}
	for _, n := range []int{1, 8192, 2, 8191, 1} {
		s := acquireFacetOwners(&pool, n)
		require.NotNil(t, s)
		require.Empty(t, s.owners)
		require.GreaterOrEqual(t, s.maxRequestedEntries, 3*n)
		require.LessOrEqual(t, s.maxRequestedEntries, 24576)
		s.owners[[2]int{100, 200}] = 17
		releaseFacetOwners(&pool, s)
		// Get may miss; inspect only an object now owned by this test again.
		if item := pool.Get(); item != nil {
			s = item.(*facetOwnerScratch)
			require.Empty(t, s.owners)
			require.GreaterOrEqual(t, s.maxRequestedEntries, 3)
			require.LessOrEqual(t, s.maxRequestedEntries, 24576)
			releaseFacetOwners(&pool, s)
		}
	}
}

type poolPollContext struct{ calls, stop int }

func (*poolPollContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (*poolPollContext) Done() <-chan struct{}       { return nil }
func (*poolPollContext) Value(any) any               { return nil }
func (c *poolPollContext) Err() error {
	c.calls++
	if c.stop > 0 && c.calls >= c.stop {
		return context.Canceled
	}
	return nil
}

func TestFacetOwnerPoolCancellation(t *testing.T) {
	tris := poolTriangles(8192, 0)
	counter := &poolPollContext{}
	_, err := unpooledFacetAdjacency(counter, tris)
	require.NoError(t, err)
	require.Less(t, counter.calls, 100)
	for stop := 1; stop <= counter.calls+1; stop++ {
		a, b := &poolPollContext{stop: stop}, &poolPollContext{stop: stop}
		want, wantErr := unpooledFacetAdjacency(a, tris)
		got, gotErr := FacetAdjacencyContext(b, tris)
		require.Equal(t, a.calls, b.calls)
		require.Equal(t, errors.Is(wantErr, context.Canceled), errors.Is(gotErr, context.Canceled))
		require.Empty(t, adjacencyScalarDifference(want, got))
		other := poolTriangles(12, stop*100000)
		expected, err := unpooledFacetAdjacency(t.Context(), other)
		require.NoError(t, err)
		actual, err := FacetAdjacencyContext(t.Context(), other)
		require.NoError(t, err)
		require.Empty(t, adjacencyScalarDifference(expected, actual))
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	var got [][]int
	allocs := testing.AllocsPerRun(1, func() { got, err = FacetAdjacencyContext(ctx, tris) })
	require.Zero(t, allocs)
	require.Nil(t, got)
	require.ErrorIs(t, err, context.Canceled)
}

func TestFacetOwnerPoolConcurrentOwnership(t *testing.T) {
	for _, workers := range []int{2, 4, 8} {
		var wg sync.WaitGroup
		failures := make(chan string, workers)
		ctx := t.Context()
		for worker := range workers {
			wg.Go(func() {
				tris := poolTriangles(512, worker*10000)
				want, err := unpooledFacetAdjacency(ctx, tris)
				if err != nil {
					failures <- "oracle error"
					return
				}
				held, err := FacetAdjacencyContext(ctx, tris)
				if err != nil {
					failures <- "initial error"
					return
				}
				for pass := range 8 {
					got, err := FacetAdjacencyContext(ctx, tris)
					if err != nil {
						failures <- "repeat error"
						return
					}
					if diff := adjacencyScalarDifference(want, got); diff != "" {
						failures <- diff
						return
					}
					got[0][0] = -1
					if diff := adjacencyScalarDifference(want, held); diff != "" {
						failures <- diff
						return
					}
					_, err = FacetAdjacencyContext(&poolPollContext{stop: 2 + pass}, tris)
					if err != nil && !errors.Is(err, context.Canceled) {
						failures <- "cancel error"
						return
					}
				}
			})
		}
		wg.Wait()
		close(failures)
		for failure := range failures {
			require.Empty(t, failure)
		}
	}
}

func unpooledFacetAdjacency(ctx context.Context, tris [][3]int) ([][]int, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	owner := make(map[[2]int]int, 3*len(tris))
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			owner[[2]int{tri[k], tri[(k+1)%3]}] = i
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	adj := make([][]int, len(tris))
	var neighbors []int
	for i, tri := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		for k := range 3 {
			if twin, ok := owner[[2]int{tri[(k+1)%3], tri[k]}]; ok && twin != i {
				if adj[i] == nil {
					if neighbors == nil {
						if err := ctx.Err(); err != nil {
							return nil, err
						}
						neighbors = make([]int, 3*len(tris))
					}
					adj[i] = neighbors[3*i : 3*i : 3*i+3]
				}
				adj[i] = append(adj[i], twin)
			}
		}
	}
	return adj, nil
}
