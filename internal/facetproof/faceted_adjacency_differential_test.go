package facetproof_test

import (
	"context"
	"fmt"
	"maps"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/stretchr/testify/require"
)

// pollCountContext counts Err calls and cancels at call cancelAt (never when
// cancelAt is zero), so two runs can be compared poll for poll.
type pollCountContext struct {
	context.Context //nolint:containedctx // A test context that counts and cancels its polls.
	cancel          context.CancelFunc
	cancelAt        int
	polls           int
}

func (c *pollCountContext) Err() error {
	c.polls++
	if c.polls == c.cancelAt {
		c.cancel()
	}
	return c.Context.Err()
}

type closedEdgesRun struct {
	neighbors []int
	isNil     bool
	err       string
	polls     int
}

func runClosedEdges(t *testing.T, edges func(context.Context, [][3]int) ([]int, error), tris [][3]int,
	cancelAt int) closedEdgesRun {
	t.Helper()
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	poll := &pollCountContext{Context: ctx, cancel: cancel, cancelAt: cancelAt}
	neighbors, err := edges(poll, tris)
	run := closedEdgesRun{neighbors: neighbors, isNil: neighbors == nil, polls: poll.polls}
	if err != nil {
		run.err = err.Error()
	}
	return run
}

func requireSameClosedEdges(t *testing.T, name string, want, got closedEdgesRun) {
	t.Helper()
	require.Equal(t, want.err, got.err, "%s: error", name)
	require.Equal(t, want.polls, got.polls, "%s: polls", name)
	require.Equal(t, want.isNil, got.isNil, "%s: nil neighbors", name)
	require.Equal(t, len(want.neighbors), len(got.neighbors), "%s: neighbors", name)
	for i := range want.neighbors {
		if want.neighbors[i] != got.neighbors[i] {
			require.Failf(t, "neighbor differs", "%s: slot %d: want %d, got %d", name, i,
				want.neighbors[i], got.neighbors[i])
		}
	}
}

// closedEdgeVariants derives reordered, open, duplicated, non-manifold,
// degenerate and out-of-range forms of a closed mesh.
func closedEdgeVariants(rng *rand.Rand, name string, tris [][3]int, vertices int) map[string][][3]int {
	clone := func() [][3]int { return append([][3]int(nil), tris...) }
	out := map[string][][3]int{name: tris}
	shuffled := clone()
	rng.Shuffle(len(shuffled), func(i, j int) { shuffled[i], shuffled[j] = shuffled[j], shuffled[i] })
	for i, t := range shuffled {
		k := rng.Intn(3)
		shuffled[i] = [3]int{t[k], t[(k+1)%3], t[(k+2)%3]}
	}
	out[name+"/shuffled"] = shuffled
	open := clone()
	i := rng.Intn(len(open))
	out[name+"/open"] = append(open[:i], open[i+1:]...)
	out[name+"/duplicate"] = append(clone(), tris[rng.Intn(len(tris))])
	t := tris[rng.Intn(len(tris))]
	out[name+"/nonmanifold"] = append(clone(), [3]int{t[1], t[0], rng.Intn(vertices)}, [3]int{t[0], t[1], rng.Intn(vertices)})
	flipped := clone()
	flipped[0] = [3]int{flipped[0][0], flipped[0][2], flipped[0][1]}
	out[name+"/flipped"] = flipped
	// A closed degenerate facet: one self-edge and an opposite pair.
	out[name+"/selfedge"] = append(clone(), [3]int{t[0], t[0], t[1]})
	out[name+"/twoselfedges"] = append(clone(), [3]int{t[0], t[0], t[0]})
	limit := 3 * (len(tris) + 1)
	for _, bad := range []int{-1, limit - 1, limit, 1 << 40} {
		far := append(clone(), [3]int{bad, t[1], t[0]})
		out[fmt.Sprintf("%s/corner%d", name, bad)] = far
	}
	return out
}

func TestClosedFacetEdgesMatchMapReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewSource(2647))
	cases := map[string][][3]int{}
	for _, model := range auditCases(t) {
		maps.Copy(cases, closedEdgeVariants(rng, model.name, model.tris, len(model.verts)))
	}
	for k := range 3000 {
		tris := make([][3]int, 1+rng.Intn(8))
		for i := range tris {
			for edge := range 3 {
				tris[i][edge] = rng.Intn(6) - 1
			}
		}
		cases[fmt.Sprintf("random-%d", k)] = tris
	}
	cases["empty"] = nil
	outcomes := map[string]int{}
	for name, tris := range cases {
		want := runClosedEdges(t, facetproof.ReferenceClosedFacetEdges, tris, 0)
		requireSameClosedEdges(t, name, want, runClosedEdges(t, facetproof.ClosedFacetEdges, tris, 0))
		outcomes[want.err]++
		// Cancelling at each poll must stop both at the same poll.
		for cancelAt := 1; cancelAt <= want.polls; cancelAt++ {
			requireSameClosedEdges(t, fmt.Sprintf("%s@%d", name, cancelAt),
				runClosedEdges(t, facetproof.ReferenceClosedFacetEdges, tris, cancelAt),
				runClosedEdges(t, facetproof.ClosedFacetEdges, tris, cancelAt))
		}
	}
	t.Logf("outcomes: %v", outcomes)
	require.Greater(t, outcomes[""], 50)
	require.Greater(t, outcomes["decad: boolean operation failed: the held boundary does not close"], 50)
}

func TestGroupedVertexRangeBoundsScratchByCorners(t *testing.T) {
	t.Parallel()
	tris := [][3]int{{0, 1, 2}, {2, 1, 5}}
	vertices, ok := facetproof.GroupedVertexRange(tris)
	require.True(t, ok)
	require.Equal(t, 6, vertices)
	// An index below 3·len(tris) groups even where the mesh skips vertices;
	// one at or past it, or a negative one, keeps the map.
	vertices, ok = facetproof.GroupedVertexRange([][3]int{{0, 1, 2}, {2, 1, 5}, {0, 0, 0}, {8, 0, 0}})
	require.True(t, ok)
	require.Equal(t, 9, vertices)
	_, ok = facetproof.GroupedVertexRange([][3]int{{0, 1, 2}, {2, 1, 6}})
	require.False(t, ok)
	_, ok = facetproof.GroupedVertexRange([][3]int{{0, -1, 2}, {2, 1, 3}})
	require.False(t, ok)
	vertices, ok = facetproof.GroupedVertexRange(nil)
	require.True(t, ok)
	require.Zero(t, vertices)
}
