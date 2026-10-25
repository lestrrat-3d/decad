package facetproof_test

import (
	"context"
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func requireClosedEdgesMatchCounts(t *testing.T, tris [][3]int) {
	t.Helper()
	counts := map[[2]int]int{}
	for _, tri := range tris {
		for edge := range 3 {
			counts[[2]int{tri[edge], tri[(edge+1)%3]}]++
		}
	}
	closed := true
	for edge, count := range counts {
		if count != 1 || counts[[2]int{edge[1], edge[0]}] != 1 {
			closed = false
			break
		}
	}
	neighbors, err := facetproof.ClosedFacetEdges(t.Context(), tris)
	if !closed {
		require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
		require.ErrorContains(t, err, "the held boundary does not close")
		return
	}
	require.NoError(t, err)
	got, err := facetproof.ClosedFacetAdjacency(t.Context(), tris, neighbors)
	require.NoError(t, err)
	want, err := meshbool.FacetAdjacencyContext(t.Context(), tris)
	require.NoError(t, err)
	require.Len(t, got, len(want))
	for i := range want {
		require.Equal(t, want[i] == nil, got[i] == nil, "facet %d", i)
		require.Len(t, got[i], len(want[i]), "facet %d", i)
		require.Equal(t, cap(want[i]), cap(got[i]), "facet %d", i)
		for edge := range want[i] {
			require.Equal(t, want[i][edge], got[i][edge], "facet %d edge %d", i, edge)
		}
	}
}

func TestClosedFacetEdgesMatchDirectedCounts(t *testing.T) {
	// Exhaust all one- and two-facet meshes on three vertex IDs, including
	// self-edges, repeated directions and opposite edges in one facet.
	for a := range 27 {
		first := [3]int{a / 9, (a / 3) % 3, a % 3}
		requireClosedEdgesMatchCounts(t, [][3]int{first})
		for b := range 27 {
			second := [3]int{b / 9, (b / 3) % 3, b % 3}
			requireClosedEdgesMatchCounts(t, [][3]int{first, second})
		}
	}
	rng := rand.New(rand.NewSource(904))
	for range 1000 {
		tris := make([][3]int, 3+rng.Intn(6))
		for i := range tris {
			for edge := range 3 {
				tris[i][edge] = rng.Intn(5) - 2
			}
		}
		requireClosedEdgesMatchCounts(t, tris)
	}
}

func TestClosedFacetEdgesPreserveLocalEdgeOrder(t *testing.T) {
	rng := rand.New(rand.NewSource(64))
	for _, model := range auditCases(t) {
		tris := append([][3]int(nil), model.tris...)
		for range 3 {
			rng.Shuffle(len(tris), func(i, j int) { tris[i], tris[j] = tris[j], tris[i] })
			for i, tri := range tris {
				shift := rng.Intn(3)
				tris[i] = [3]int{tri[shift], tri[(shift+1)%3], tri[(shift+2)%3]}
			}
			requireClosedEdgesMatchCounts(t, tris)
		}
		// Reversing every facet changes orientation but keeps paired edges.
		for i, tri := range tris {
			tris[i] = [3]int{tri[0], tri[2], tri[1]}
		}
		requireClosedEdgesMatchCounts(t, tris)
	}
}

func TestAuditClosurePrecedesCoordinateAndVolumeErrors(t *testing.T) {
	for _, workers := range []int{1, 4} {
		verts := []r3.Vec{{X: math.NaN()}, {X: 1}}
		closed := [][3]int{{0, 0, 1}}
		require.Panics(t, func() {
			_, _ = facetproof.AuditFacetedMeshWorkers(t.Context(), verts, closed, workers)
		})
		verts[0].X = 0
		_, err := facetproof.AuditFacetedMeshWorkers(t.Context(), verts, closed, workers)
		require.ErrorContains(t, err, "a result shell encloses no volume")

		verts[0].X = math.NaN()
		_, err = facetproof.AuditFacetedMeshWorkers(t.Context(), verts, [][3]int{{0, 0, 0}}, workers)
		require.ErrorContains(t, err, "the held boundary does not close")
	}
}

func TestClosedFacetEdgesCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := facetproof.ClosedFacetEdges(ctx, [][3]int{{0, 1, 2}})
	require.ErrorIs(t, err, context.Canceled)
	_, err = facetproof.ClosedFacetAdjacency(ctx, nil, nil)
	require.ErrorIs(t, err, context.Canceled)

	// Even an invalid early edge must not hide cancellation during the scan.
	tris := make([][3]int, 1024)
	scan, stop := context.WithCancel(t.Context())
	defer stop()
	poll := &cancelAuditContext{Context: scan, cancel: stop}
	_, err = facetproof.ClosedFacetEdges(poll, tris)
	require.ErrorIs(t, err, context.Canceled)

	rows, stopRows := context.WithCancel(t.Context())
	defer stopRows()
	poll = &cancelAuditContext{Context: rows, cancel: stopRows}
	_, err = facetproof.ClosedFacetAdjacency(poll, tris, make([]int, 3*len(tris)))
	require.ErrorIs(t, err, context.Canceled)
}

type cancelAuditContext struct {
	context.Context //nolint:containedctx // A test context that cancels during a bounded polling loop.
	cancel          context.CancelFunc
	polls           int
}

func (c *cancelAuditContext) Err() error {
	c.polls++
	if c.polls == 3 {
		c.cancel()
	}
	return c.Context.Err()
}
