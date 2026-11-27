package meshbool

// CI: isolated-tests

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestConformMemoMatchesTwoScan holds the memoized pass to the two-scan pass:
// same split flag, triangles, sources, bounds, xbeta, error and number of
// budget steps, on meshes with twin edges, repeated edges, interior points and
// degenerate facets. Some seed must split.
//
// Leg shown to fail: charging a twin use no steps (dropping the StepN call in
// edgeInteriorHitsMemo) changes the step count of every seed that has a twin.
func TestConformMemoMatchesTwoScan(t *testing.T) {
	t.Parallel()
	splits, refused := 0, 0
	for seed := range uint64(400) {
		base := twinEdgeMesh(rand.New(rand.NewPCG(seed, 31)), 10+int(seed%30))
		want := runConformPass(t, base, 0, twoScanPass(t))
		got := runConformPass(t, base, 0, memoPass(t))
		requireSameRunf(t, want, got, "seed %d", seed)
		switch {
		case want.err != nil:
			refused++
		case want.split:
			splits++
		}
	}
	require.Positive(t, splits, "no seed split a facet")
	t.Logf("%d of 400 seeds split, %d refused by the exact triangulation", splits, refused)
}

// TestConformMemoRefusesAtSameStep refuses the budget at every step count up
// to the pass's total and requires both passes to stop at the same point with
// the same state, so a refusal lands where it did before.
//
// Leg shown to fail: charging a twin use one step too few moves the refusal
// of any step limit that falls inside a twin charge.
func TestConformMemoRefusesAtSameStep(t *testing.T) {
	t.Parallel()
	refusals := 0
	for seed := range uint64(30) {
		base := twinEdgeMesh(rand.New(rand.NewPCG(seed, 37)), 12)
		total := runConformPass(t, base, 0, twoScanPass(t)).steps
		for limit := uint64(1); limit <= total+1; limit++ {
			want := runConformPass(t, base, limit, twoScanPass(t))
			got := runConformPass(t, base, limit, memoPass(t))
			requireSameRunf(t, want, got, "seed %d limit %d", seed, limit)
			if want.err == errConformStep {
				refusals++
			}
		}
	}
	require.Positive(t, refusals, "no limit refused the pass")
}

// TestConformMemoWalksEachUndirectedEdgeOnce checks the saving the memo exists
// for: on a closed tetrahedron, whose six edges each have a twin, the point
// tree is walked six times for twelve directed edges.
func TestConformMemoWalksEachUndirectedEdgeOnce(t *testing.T) {
	t.Parallel()
	base := approxMesh{
		tris:   [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}},
		src:    make([]int, 4),
		bounds: make([]float64, 4),
	}
	for _, p := range []r3.Vec{{}, {X: 4}, {Y: 4}, {Z: 4}} {
		base.verts = append(base.verts, proof.XptOf(p))
		base.xbeta = append(base.xbeta, 0)
	}
	budget := proofbound.NewWorkBudget(t.Context())
	scan, err := newConformScanApprox(budget, base.verts, base.rounded())
	require.NoError(t, err)
	memo := newConformEdgeMemo(len(base.tris))
	walks := 0
	for _, tri := range base.tris {
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			_, seen := memo.steps.Get(min(a, b), max(a, b))
			_, err := scan.edgeInteriorHitsMemo(budget, memo, a, b, tri)
			require.NoError(t, err)
			if !seen {
				walks++
			}
		}
	}
	require.Equal(t, 6, walks)
}
