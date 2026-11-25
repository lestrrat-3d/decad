package loftmesh

import (
	"context"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// retryGrid is an n by n cell grid over the XY plane, lifted by a small
// smooth bump so its triangles are a graph over XY but not coplanar. Cell
// (x, y) holds triangles {v00, v10, v11} and {v00, v11, v01}, in that order.
type retryGrid struct {
	n     int
	verts []r3.Vec
	tris  [][3]int
}

func newRetryGrid(n int) *retryGrid {
	g := &retryGrid{n: n}
	for y := 0; y <= n; y++ {
		for x := 0; x <= n; x++ {
			fx, fy := float64(x), float64(y)
			g.verts = append(g.verts, r3.Vec{X: fx, Y: fy, Z: 0.01 * (fx*fy - 0.5*fx*fx)})
		}
	}
	for y := range n {
		for x := range n {
			v00 := g.vertex(x, y)
			v10, v01, v11 := g.vertex(x+1, y), g.vertex(x, y+1), g.vertex(x+1, y+1)
			g.tris = append(g.tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	return g
}

func (g *retryGrid) vertex(x, y int) int { return y*(g.n+1) + x }

// move sets the projected position of grid vertex (x, y), keeping its lift.
func (g *retryGrid) move(x, y int, px, py float64) {
	i := g.vertex(x, y)
	g.verts[i].X, g.verts[i].Y = px, py
}

func (g *retryGrid) patch() ProjectedPatch {
	indices := make([]int, len(g.tris))
	for i := range indices {
		indices[i] = i
	}
	return ProjectedPatch{Triangles: indices, Axes: [2]int{0, 1}}
}

// invertedByCross lists the triangles whose projected cross product over XY is
// not positive, by plain float arithmetic on the small integers of the grid.
func (g *retryGrid) invertedByCross() map[int]struct{} {
	out := map[int]struct{}{}
	for i, tri := range g.tris {
		a, b, c := g.verts[tri[0]], g.verts[tri[1]], g.verts[tri[2]]
		if (b.X-a.X)*(c.Y-a.Y)-(b.Y-a.Y)*(c.X-a.X) <= 0 {
			out[i] = struct{}{}
		}
	}
	return out
}

// ringExpectation returns the triangles a one-ring retry keeps: those sharing
// no vertex with an inverted triangle, in index order.
func (g *retryGrid) ringExpectation() []int {
	inverted := g.invertedByCross()
	ring := map[int]struct{}{}
	for i := range inverted {
		for _, v := range g.tris[i] {
			ring[v] = struct{}{}
		}
	}
	var keep []int
	for i, tri := range g.tris {
		_, a := ring[tri[0]]
		_, b := ring[tri[1]]
		_, c := ring[tri[2]]
		if !a && !b && !c {
			keep = append(keep, i)
		}
	}
	return keep
}

// retryOutcome is what the audit's per-patch decision reports for a grid.
type retryOutcome struct {
	old     ProjectedPatchCondition
	reduced ProjectedPatch
	ok      bool
}

func retryDecide(ctx context.Context, g *retryGrid) retryOutcome {
	remaining := uint64(100_000_000)
	old := projectedPatchProofLimited(proofbound.NewWorkBudget(ctx), g.verts, g.tris, g.patch(), &remaining)
	remaining = 100_000_000
	reduced, ok := provenProjectedPatch(proofbound.NewWorkBudget(ctx), g.verts, g.tris, g.patch(), &remaining)
	return retryOutcome{old: old, reduced: reduced, ok: ok}
}

// requireRetryInvariants checks what holds for every outcome: an accepted
// patch proves on its own by the public proof, keeps only members of the
// input in order, and a patch the first proof accepts comes back unchanged.
func requireRetryInvariants(t *testing.T, g *retryGrid, got retryOutcome) {
	t.Helper()
	patch := g.patch()
	if got.old == ProjectedPatchHolds {
		require.True(t, got.ok)
		require.Equal(t, patch, got.reduced)
		return
	}
	if got.old != ProjectedPatchOrientation {
		require.False(t, got.ok)
		return
	}
	if !got.ok {
		return
	}
	require.Equal(t, ProjectedPatchHolds,
		ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, got.reduced))
	require.Equal(t, patch.Axes, got.reduced.Axes)
	require.True(t, slices.IsSorted(got.reduced.Triangles))
	require.Less(t, len(got.reduced.Triangles), len(patch.Triangles))
}

// retryAuditVerdict runs the audit with the patch and without any patch.
func retryAuditVerdict(t *testing.T, g *retryGrid, ceiling uint64) (LoftAuditWork, error, error) {
	t.Helper()
	patches := []ProjectedPatch{g.patch()}
	work, withPatch := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, nil,
		loftGenericShortcuts, ceiling, len(g.tris), false, patches...)
	_, without := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), g.verts, g.tris, nil,
		loftGenericShortcuts, ceiling, len(g.tris), false)
	return work, withPatch, without
}
