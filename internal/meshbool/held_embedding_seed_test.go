package meshbool

import (
	"context"
	"math"
	"math/big"
	"runtime"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldSeedCase builds a tilted grid of 4,232 facets whose vertices all move,
// plus extra facets that make the first scan fail, named by kind:
//
//	"clean"     one far, unrelated, unmoved facet, so the scan is not a self scan;
//	"one"       "clean" plus an unmoved facet piercing the grid;
//	"many"      "one" plus a second unmoved piercing facet and a moved one;
//	"collapsed" "many" plus a moved collapsed facet beside the moved piercing
//	            facet's failing pair, and another far away;
//	"collapsed-only" "clean" plus the far collapsed facet.
//
// Every vertex is movable, so repair may move any of them.
func heldSeedCase(t *testing.T, kind heldSeedKind) HeldRounding {
	t.Helper()
	const side = 46
	tilt, err := r3.NewFrame(r3.NewVec(0.25, -0.5, 0.125),
		r3.NewVec(math.Cos(0.3), math.Sin(0.3), 0.1), r3.NewVec(-math.Sin(0.3), math.Cos(0.3), 0.2))
	require.NoError(t, err)
	var h HeldRounding
	for y := range side + 1 {
		for x := range side + 1 {
			h.Verts = append(h.Verts, tilt.ToWorld(r3.Vec{X: float64(x) / 3, Y: float64(y) / 3}))
		}
	}
	for y := range side {
		for x := range side {
			a := y*(side+1) + x
			b := a + side + 1
			h.Tris = append(h.Tris, [3]int{a, a + 1, b}, [3]int{a + 1, b + 1, b})
		}
	}
	h.Moved = make([]bool, len(h.Verts), len(h.Verts)+32)
	for i := range h.Moved {
		h.Moved[i] = true
	}
	// add appends one facet at the given grid-space corners; moved says whether
	// its vertices are moved, so whether it is a query facet.
	add := func(moved bool, corners ...r3.Vec) {
		n := len(h.Verts)
		for _, c := range corners {
			h.Verts = append(h.Verts, tilt.ToWorld(c))
			h.Moved = append(h.Moved, moved)
		}
		h.Tris = append(h.Tris, [3]int{n, n + 1, n + 2})
	}
	// pierce adds a facet through the grid cell at (cx, cy).
	pierce := func(moved bool, cx, cy int) {
		x, y := float64(cx)/3, float64(cy)/3
		add(moved, r3.Vec{X: x + 0.05, Y: y + 0.05, Z: -1}, r3.Vec{X: x + 0.25, Y: y + 0.05, Z: 1},
			r3.Vec{X: x + 0.05, Y: y + 0.25, Z: 1})
	}
	// collapse adds a facet with two equal corners at the cell (cx, cy), lifted
	// above the grid so that only its own collapse makes it unusual.
	collapse := func(cx, cy int) {
		x, y := float64(cx)/3, float64(cy)/3
		add(true, r3.Vec{X: x + 0.1, Y: y + 0.1, Z: 0.5}, r3.Vec{X: x + 0.1, Y: y + 0.1, Z: 0.5},
			r3.Vec{X: x + 0.2, Y: y + 0.1, Z: 0.5})
	}
	add(false, r3.Vec{X: 500}, r3.Vec{X: 501}, r3.Vec{X: 500, Y: 1})
	switch kind {
	case seedClean:
	case seedOne:
		pierce(false, 1, 1)
	case seedMany:
		pierce(false, 1, 1)
		pierce(false, 15, 38)
		pierce(true, 30, 30)
	case seedCollapsed:
		pierce(false, 1, 1)
		pierce(false, 15, 38)
		pierce(true, 30, 30)
		collapse(31, 30)
		add(true, r3.Vec{X: 700}, r3.Vec{X: 700}, r3.Vec{X: 701})
	case seedCollapsedOnly:
		add(true, r3.Vec{X: 700}, r3.Vec{X: 700}, r3.Vec{X: 701})
	default:
		t.Fatalf("unknown case %q", kind)
	}
	h.Movable = make([]bool, len(h.Verts))
	h.Exact = make([]proof.Xpt, len(h.Verts))
	w := new(big.Int).Lsh(big.NewInt(1), 61)
	scaled := func(f float64, extra *big.Rat) *big.Int {
		r := new(big.Rat).SetFloat64(f)
		if extra != nil {
			r.Add(r, extra)
		}
		return new(big.Int).Set(r.Mul(r, new(big.Rat).SetInt(w)).Num())
	}
	for i, p := range h.Verts {
		h.Movable[i] = true
		// The exact point sits a few units of 2^-60 off the held one in y, so
		// the float box around it has corners repair can try.
		off := new(big.Rat).SetFrac(big.NewInt(int64(i%5-2)), new(big.Int).Lsh(big.NewInt(1), 60))
		h.Exact[i] = proof.Xpt{X: scaled(p.X, nil), Y: scaled(p.Y, off), Z: scaled(p.Z, nil), W: new(big.Int).Set(w)}
	}
	return h
}

type heldSeedKind string

const (
	seedClean         heldSeedKind = "clean"
	seedOne           heldSeedKind = "one"
	seedMany          heldSeedKind = "many"
	seedCollapsed     heldSeedKind = "collapsed"
	seedCollapsedOnly heldSeedKind = "collapsed-only"
)

var heldSeedKinds = []heldSeedKind{seedClean, seedOne, seedMany, seedCollapsed, seedCollapsedOnly}

func heldSeedQuery(h HeldRounding) []int {
	var query []int
	for i, tri := range h.Tris {
		if h.Moved[tri[0]] || h.Moved[tri[1]] || h.Moved[tri[2]] {
			query = append(query, i)
		}
	}
	return query
}

func requireSameInts(t *testing.T, what string, want, got []int) {
	t.Helper()
	if slices.Equal(want, got) {
		return
	}
	first := -1
	for k := range min(len(want), len(got)) {
		if want[k] != got[k] {
			first = k
			break
		}
	}
	require.Failf(t, what+" differs", "want %d entries, got %d, first difference at index %d", len(want), len(got), first)
}

func requireSameVerts(t *testing.T, want, got []r3.Vec) {
	t.Helper()
	require.Len(t, got, len(want))
	moved := 0
	first := -1
	for i := range want {
		if want[i] != got[i] {
			if first < 0 {
				first = i
			}
			moved++
		}
	}
	require.Zero(t, moved, "%d vertices differ, first is vertex %d", moved, first)
}

// TestHeldSeedMatchesOffenders checks that the first sweep built from the
// parallel scan's failing pairs lists the same vertices and failing facets as
// scanning every query facet with offenders does.
func TestHeldSeedMatchesOffenders(t *testing.T) {
	if runtime.GOMAXPROCS(0) < 2 {
		t.Skip("the parallel scan needs more than one processor")
	}
	for _, kind := range heldSeedKinds {
		t.Run(string(kind), func(t *testing.T) {
			h := heldSeedCase(t, kind)
			query := heldSeedQuery(h)
			require.GreaterOrEqual(t, len(query), 4096)
			require.Less(t, len(query), len(h.Tris), "not a self scan")

			seeded := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
			seeded.collect = true
			clean, err := seeded.parallelClean(t.Context(), query)
			require.NoError(t, err)
			require.Equal(t, kind == seedClean, clean)
			require.Equal(t, kind != seedClean, seeded.seed != nil)
			require.False(t, seeded.collect, "the collecting mode is one scan long")
			if kind == seedClean {
				return
			}
			require.NoError(t, seeded.buildGrid())
			seeded.prepareRepair()
			seeded.pairs = map[uint64]uint64{}
			gotVerts, gotFailing, err := seeded.seededOffenders(query)
			require.NoError(t, err)

			ref := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
			require.NoError(t, ref.buildGrid())
			ref.prepareRepair()
			ref.pairs = map[uint64]uint64{}
			wantVerts, wantFailing, err := ref.offenders(query)
			require.NoError(t, err)

			t.Logf("%d failing facets, %d vertices to try", len(wantFailing), len(wantVerts))
			require.NotEmpty(t, wantFailing, "the case fails the first scan")
			requireSameInts(t, "vertices", wantVerts, gotVerts)
			requireSameInts(t, "failing facets", wantFailing, gotFailing)
		})
	}
}

// TestHeldSeedKeepsRepairResult checks that the repair seeded from the
// parallel scan gives the result of the memo-free repair, which scans every
// query facet itself: the same vertices, moved count and failing facets.
func TestHeldSeedKeepsRepairResult(t *testing.T) {
	if runtime.GOMAXPROCS(0) < 2 {
		t.Skip("the parallel scan needs more than one processor")
	}
	for _, kind := range heldSeedKinds {
		t.Run(string(kind), func(t *testing.T) {
			h := heldSeedCase(t, kind)
			plain, memo := h, h
			plain.Verts, memo.Verts = slices.Clone(h.Verts), slices.Clone(h.Verts)
			wantMoved, wantFailing, err := repairHeldEmbedding(t.Context(), plain, false)
			require.NoError(t, err)
			gotMoved, gotFailing, err := repairHeldEmbedding(t.Context(), memo, true)
			require.NoError(t, err)
			t.Logf("%d facets, %d moved vertices, %d failing facets", len(h.Tris), gotMoved, len(gotFailing))
			require.Equal(t, wantMoved, gotMoved, "moved count")
			requireSameInts(t, "failing facets", wantFailing, gotFailing)
			requireSameVerts(t, plain.Verts, memo.Verts)
			if kind != seedClean {
				changed := 0
				for i := range h.Verts {
					if h.Verts[i] != memo.Verts[i] {
						changed++
					}
				}
				require.Equal(t, gotMoved, changed, "the moved count is the number of changed vertices")
			}
		})
	}
}

// pollCancelCtx reports cancellation from its n-th Err call on. It is safe for
// the parallel scan's workers.
type pollCancelCtx struct {
	context.Context //nolint:containedctx // a test wrapper counting Err calls
	calls           atomic.Int64
	cancelAt        int64
}

func (c *pollCancelCtx) Err() error {
	if n := c.calls.Add(1); c.cancelAt > 0 && n >= c.cancelAt {
		return context.Canceled
	}
	return nil
}

// TestHeldSeedCancellation checks that a cancellation landing at an early
// poll of the seeded repair gives the context error, as on the memo-free path.
func TestHeldSeedCancellation(t *testing.T) {
	if runtime.GOMAXPROCS(0) < 2 {
		t.Skip("the parallel scan needs more than one processor")
	}
	for _, kind := range []heldSeedKind{seedOne, seedCollapsed} {
		t.Run(string(kind), func(t *testing.T) {
			h := heldSeedCase(t, kind)
			run := func(memo bool, cancelAt int64) (int64, error) {
				c := &pollCancelCtx{Context: t.Context(), cancelAt: cancelAt}
				run := h
				run.Verts = slices.Clone(h.Verts)
				_, _, err := repairHeldEmbedding(c, run, memo)
				return c.calls.Load(), err
			}
			for _, memo := range []bool{false, true} {
				total, err := run(memo, 0)
				require.NoError(t, err)
				require.Greater(t, total, int64(8), "memo=%v polls the context", memo)
				for _, at := range []int64{1, 2, total / 8, total / 4} {
					_, err := run(memo, max(at, 1))
					require.ErrorIs(t, err, context.Canceled, "memo=%v cancelled at poll %d of %d", memo, at, total)
				}
			}
		})
	}
}
