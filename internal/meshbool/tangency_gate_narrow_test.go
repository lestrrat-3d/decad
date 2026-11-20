package meshbool

import (
	"context"
	"math"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// narrowGrid is how many quads the narrowing test's gridPrism sides have
// along each edge.
const narrowGrid = 6

// sideFacets is the facets of side s of a narrowGrid gridPrism, which holds
// 2·narrowGrid² consecutive facets.
func sideFacets(s int) []int {
	out := make([]int, 2*narrowGrid*narrowGrid)
	for k := range out {
		out[k] = s*len(out) + k
	}
	return out
}

// treeEdgePair is two meshes of 129 facets each, so a gather over all of them
// queries its box tree. Facet 0 of each comes within slack 1-2^-53 of the
// other by BoxesWithin's rounding, while the tree's widened box leaves it out;
// the other facets sit far apart.
func treeEdgePair(t *testing.T) (*BoolMesh, *BoolMesh) {
	t.Helper()
	build := func(x float64, far float64) *BoolMesh {
		verts := []r3.Vec{r3.NewVec(x, 0, 0), r3.NewVec(x, 1, 0), r3.NewVec(x, 0, 1)}
		tris := [][3]int{{0, 1, 2}}
		for k := range 128 {
			base := len(verts)
			fx := far + 3*float64(k)
			verts = append(verts, r3.NewVec(fx, 0, 0), r3.NewVec(fx+1, 0, 0), r3.NewVec(fx, 1, 0))
			tris = append(tris, [3]int{base, base + 1, base + 2})
		}
		bm, err := PrepContext(t.Context(), verts, tris, nil)
		require.NoError(t, err)
		return bm
	}
	return build(-1, 1000), build(-0x1p-53+0x1p-104, -5000)
}

// TestNarrowNearContactsMatchesTightGather holds narrowNearContacts to the
// gather it replaces. After the same broad gather on a fresh memo, it returns
// the contacts GatherNearContacts returns at the tighter slack, in the same
// order, polls the context as many times, leaves the memo as the gather
// leaves it, and stops at the same poll with the same error when the context
// is cancelled there. The broad gather's log does not change what it returns
// or how it polls.
//
// Legs shown to fail: dropping the box-tree test (the tree-edge case),
// dropping the BoxesWithin test, skipping the first queued pair, walking the
// log backwards, dropping the TriTriDistanceExceeds measurement, and dropping
// the final poll.
func TestNarrowNearContactsMatchesTightGather(t *testing.T) {
	t.Parallel()
	const n = narrowGrid
	gap := func(t *testing.T, shift r3.Vec) (*BoolMesh, *BoolMesh) {
		t.Helper()
		_, a := gridPrism(t, r3.Vec{}, r3.NewVec(1, 1, 1), n, func(int) float64 { return 0.1 })
		_, b := gridPrism(t, shift, r3.NewVec(1, 1, 1).Add(shift), n, func(int) float64 { return 0.1 })
		return a, b
	}
	nearA, nearB := gap(t, r3.NewVec(0.3, 0.2, 1.03))
	closeA, closeB := gap(t, r3.NewVec(0.3, 0.2, 1.015))
	crossA, crossB := gap(t, r3.NewVec(0.5, 0.5, 0.5))
	edgeA, edgeB := treeEdgePair(t)
	broadSlack := 0.2 * (1 + 1e-9)
	type narrowCase struct {
		name     string
		a, b     *BoolMesh
		fis, fjs []int
		broad    float64
		tight    []float64
	}
	tights := []float64{0, 0.01, 0.02 * (1 + 1e-9), 0.05, broadSlack}
	cases := []narrowCase{
		{name: "near-miss sides, scanned", a: nearA, b: nearB, fis: sideFacets(5), fjs: sideFacets(4),
			broad: broadSlack, tight: tights},
		{name: "near-miss prisms, tree", a: nearA, b: nearB, fis: AllFacets(nearA), fjs: AllFacets(nearB),
			broad: broadSlack, tight: tights},
		{name: "closer near-miss prisms, tree", a: closeA, b: closeB, fis: AllFacets(closeA), fjs: AllFacets(closeB),
			broad: broadSlack, tight: tights},
		{name: "crossing prisms, tree", a: crossA, b: crossB, fis: AllFacets(crossA), fjs: AllFacets(crossB),
			broad: broadSlack, tight: tights},
		{name: "crossing sides, scanned", a: crossA, b: crossB, fis: sideFacets(5), fjs: sideFacets(0),
			broad: broadSlack, tight: tights},
		{name: "a pair the tight tree leaves out", a: edgeA, b: edgeB, fis: AllFacets(edgeA), fjs: AllFacets(edgeB),
			broad: 2, tight: []float64{1 - 0x1p-53}},
	}
	type outcome struct {
		nc    NearContacts
		polls int64
		err   error
	}
	// old runs the broad gather and then the tight one; neu runs the logged
	// broad gather and then narrowNearContacts. Both count polls from the
	// start of the tight step and cancel at the cancelAt-th poll overall.
	run := func(t *testing.T, c narrowCase, tight float64, cancelAt int64, logged bool) (outcome, outcome, *ContactMemo) {
		t.Helper()
		ctx := &gatePollCounter{Context: WithContactWorkers(t.Context(), 1), cancelAt: cancelAt, limit: math.MaxInt64}
		memo := NewContactMemo(c.a, c.b)
		var log []gatheredPair
		var broad NearContacts
		var deferred bool
		var err error
		if logged {
			broad, deferred, err = gatherNearContactsLog(ctx, c.a, c.fis, c.b, c.fjs, c.broad, memo,
				NearContactRunner(c.broad), ContactBatchSize, &log)
		} else {
			broad, deferred, err = GatherNearContacts(ctx, c.a, c.fis, c.b, c.fjs, c.broad, memo)
		}
		require.False(t, deferred)
		first := outcome{nc: broad, polls: ctx.polls.Load(), err: err}
		if err != nil {
			return first, outcome{}, memo
		}
		var nc NearContacts
		if logged {
			nc, deferred, err = narrowNearContacts(ctx, c.a, c.fis, c.b, c.fjs, tight, memo, broad, log)
		} else {
			nc, deferred, err = GatherNearContacts(ctx, c.a, c.fis, c.b, c.fjs, tight, memo)
		}
		require.False(t, deferred)
		return first, outcome{nc: nc, polls: ctx.polls.Load() - first.polls, err: err}, memo
	}
	same := func(t *testing.T, want, got outcome, what string) {
		t.Helper()
		require.Equal(t, want.err, got.err, what)
		require.Equal(t, want.polls, got.polls, "%s polls", what)
		require.Equal(t, want.nc.CloseA, got.nc.CloseA, "%s CloseA", what)
		require.Equal(t, want.nc.CloseB, got.nc.CloseB, "%s CloseB", what)
		require.Len(t, got.nc.Spans, len(want.nc.Spans), "%s span count", what)
		for k := range want.nc.Spans {
			require.True(t, reflect.DeepEqual(want.nc.Spans[k], got.nc.Spans[k]), "%s span %d differs", what, k)
		}
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			for _, tight := range c.tight {
				wantBroad, want, wantMemo := run(t, c, tight, 0, false)
				gotBroad, got, gotMemo := run(t, c, tight, 0, true)
				require.NoError(t, want.err)
				same(t, wantBroad, gotBroad, "broad")
				same(t, want, got, "tight")
				require.NotEmpty(t, wantBroad.nc.CloseA)
				require.Len(t, gotMemo.Values, len(wantMemo.Values))
				require.Len(t, gotMemo.Sparse, len(wantMemo.Sparse))
				require.Equal(t, wantMemo.Dense, gotMemo.Dense)
				for _, at := range []int64{1, (want.polls + 1) / 2, want.polls} {
					cancelAt := wantBroad.polls + at
					_, want, _ := run(t, c, tight, cancelAt, false)
					_, got, _ := run(t, c, tight, cancelAt, true)
					require.ErrorIs(t, want.err, context.Canceled)
					same(t, want, got, "cancelled")
				}
			}
		})
	}
	t.Run("the tree-edge pair is left out of the tight contacts", func(t *testing.T) {
		t.Parallel()
		c := cases[len(cases)-1]
		broad, tight, _ := run(t, c, c.tight[0], 0, false)
		require.Equal(t, []int{0}, broad.nc.CloseA, "the broad gather keeps the edge pair")
		require.Empty(t, tight.nc.CloseA, "the tight gather's tree leaves the edge pair out")
		require.True(t, BoxesWithin(c.a.Boxes[0], c.b.Boxes[0], c.tight[0]))
	})
}
