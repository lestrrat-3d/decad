package loftmesh

import (
	"context"
	"errors"
	"math"
	"math/rand/v2"
	"runtime"
	"slices"
	"sync/atomic"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// stepCountingContext counts Err calls and fails from call failFrom on;
// failFrom 0 never fails.
type stepCountingContext struct {
	calls    int
	failFrom int
}

func (c *stepCountingContext) Deadline() (time.Time, bool) { return time.Time{}, false }
func (c *stepCountingContext) Done() <-chan struct{}       { return nil }
func (c *stepCountingContext) Value(any) any               { return nil }

func (c *stepCountingContext) Err() error {
	c.calls++
	if c.failFrom != 0 && c.calls >= c.failFrom {
		return context.Canceled
	}
	return nil
}

func TestFlatWalkRowMatchesVisitRow(t *testing.T) {
	boxes := randomBoxes(11, 900, 0.35)
	tree := newTreeScan(boxes, allMembers(len(boxes)))
	for i := 0; i < len(boxes); i += 5 {
		var want []int32
		work, _, err := tree.visitRow(unlimitedBudget(t), i, math.MaxUint64,
			func(j int) { want = append(want, int32(j)) })
		require.NoError(t, err)
		for _, limit := range []uint64{0, 1, 17, work / 2, max(work, 1) - 1, work, math.MaxUint64} {
			var wantPart []int32
			wantWork, wantExceeded, err := tree.visitRow(unlimitedBudget(t), i, limit,
				func(j int) { wantPart = append(wantPart, int32(j)) })
			require.NoError(t, err)
			gotPart, gotWork, gotExceeded := tree.walkRowAppend(i, limit, nil)
			require.Equal(t, wantWork, gotWork, "row %d limit %d", i, limit)
			require.Equal(t, wantExceeded, gotExceeded, "row %d limit %d", i, limit)
			require.Equal(t, wantPart, gotPart, "row %d limit %d", i, limit)
		}
	}
}

func TestFlatPairWalkMatchesVisit(t *testing.T) {
	boxes := randomBoxes(12, 700, 0.3)
	tree := newTreeScan(boxes, allMembers(len(boxes)))
	type pair struct{ i, j int }
	var all []pair
	total, _, err := tree.visit(unlimitedBudget(t), math.MaxUint64, func(i, j int) { all = append(all, pair{i, j}) })
	require.NoError(t, err)
	for _, limit := range []uint64{0, 1, 17, total / 2, total - 1, total, math.MaxUint64} {
		var want, got []pair
		wantWork, wantExceeded, err := tree.visit(unlimitedBudget(t), limit,
			func(i, j int) { want = append(want, pair{i, j}) })
		require.NoError(t, err)
		w := pairWalker{t: &tree, limit: limit, fn: func(i, j int) { got = append(got, pair{i, j}) }}
		gotExceeded := w.walk(0, 0)
		require.Equal(t, wantWork, w.scanned, "limit %d", limit)
		require.Equal(t, wantExceeded, gotExceeded, "limit %d", limit)
		require.Equal(t, want, got, "limit %d", limit)
	}
	require.NotEmpty(t, all)
}

func TestSortRowByBitmapMatchesSort(t *testing.T) {
	const f = 5000
	words := (f + 63) / 64
	rng := rand.New(rand.NewPCG(13, 1))
	for trial := range 300 {
		n := rng.IntN(120)
		span := 1 + rng.IntN(f)
		row := make([]int32, 0, n)
		seen := map[int32]struct{}{}
		base := rng.IntN(f - span + 1)
		for len(row) < min(n, span) {
			j := int32(base + rng.IntN(span))
			if _, dup := seen[j]; !dup {
				seen[j] = struct{}{}
				row = append(row, j)
			}
		}
		if trial%5 == 0 && len(row) > 1 {
			row = append(row, row[rng.IntN(len(row))]) // an injected duplicate
		}
		want := slices.Clone(row)
		slices.Sort(want)
		require.Equal(t, want, sortRowByBitmap(slices.Clone(row), words), "trial %d", trial)
	}
	// A bitmap taken from the pool is zeroed again after a duplicate row.
	require.Equal(t, []int32{3, 3, 9}, sortRowByBitmap([]int32{9, 3, 3}, words))
	require.Equal(t, []int32{2, 3, 9}, sortRowByBitmap([]int32{9, 3, 2}, words))
}

func TestWalkLoftRowsBitmapMatchesSort(t *testing.T) {
	boxes := randomBoxes(14, 1500, 0.3)
	members := allMembers(len(boxes))
	tree := newTreeScan(boxes, members)
	for _, workers := range []int{1, 4} {
		out := make([]loftRowResult, 400)
		require.NoError(t, walkLoftRows(unlimitedBudget(t), tree, members, 100, 500, math.MaxUint64, true, workers, out))
		for k, r := range out {
			var want []int32
			work, exceeded := tree.walkRow(members[100+k], math.MaxUint64, func(j int) { want = append(want, int32(j)) })
			slices.Sort(want)
			require.Equal(t, work, r.work)
			require.Equal(t, exceeded, r.exceeded)
			require.Equal(t, len(want), r.count)
			require.Equal(t, want, slices.Clone(r.row))
		}
	}
}

// refParallelPairs is the serial replay parallelLoftPairs ran before its
// workers tallied: one Step per pair up to the first failure.
func refParallelPairs(budget *proofbound.WorkBudget, cands loftCandidates, results []uint8, failAt int,
	failErr error, work *LoftAuditWork) (int, int, error) {
	i := 0
	for k := 0; k < len(cands.js) && k <= failAt; k++ {
		for k >= cands.starts[i+1] {
			i++
		}
		if err := budget.Step(); err != nil {
			return -1, -1, err
		}
		switch results[k] {
		case pairSkipped:
			work.Skips++
		case pairEdgeCertified:
			work.EdgeCerts++
		case pairVertexCertified:
			work.VertexCerts++
		default:
			work.Classifications++
		}
		if k == failAt {
			return cands.baseRow + i, int(cands.js[k]), failErr
		}
	}
	return -1, -1, nil
}

func TestParallelLoftPairsTalliesMatchReplay(t *testing.T) {
	const rows, baseRow = 400, 7
	rng := rand.New(rand.NewPCG(15, 1))
	cands := loftCandidates{baseRow: baseRow, starts: make([]int, 1, rows+1)}
	for range rows {
		for range rng.IntN(9) { // empty rows included
			cands.js = append(cands.js, int32(rng.IntN(1000)))
		}
		cands.starts = append(cands.starts, len(cands.js))
	}
	n := len(cands.js)
	outcome := func(i, j int) uint8 { return uint8(1 + (i*31+j*17)%4) }
	errRefused := errors.New("refused")
	results := make([]uint8, n)
	for k := range n {
		results[k] = outcome(baseRow+rowOf(cands, k), int(cands.js[k]))
	}
	type result struct {
		failI, failJ int
		err          error
		work         LoftAuditWork
		errCalls     int
	}
	budgets := map[string]func(ctx context.Context) *proofbound.WorkBudget{
		"poll": proofbound.NewPollWorkBudget,
		"fn":   proofbound.NewWorkBudget,
	}
	// A failing pair is named by value, so a repeated pair fails at each
	// occurrence and the lowest index is the one that counts.
	for name, newBudget := range budgets {
		for _, failAt := range []int{-1, 0, 1, n / 3, n / 2, n - 1} {
			failI, failJ := -1, -1
			if failAt >= 0 {
				failI, failJ = baseRow+rowOf(cands, failAt), int(cands.js[failAt])
			}
			classify := func(i, j int) (uint8, error) {
				if i == failI && j == failJ {
					return outcome(i, j), errRefused
				}
				return outcome(i, j), nil
			}
			refFail := n
			for k := range n {
				if _, err := classify(baseRow+rowOf(cands, k), int(cands.js[k])); err != nil {
					refFail = k
					break
				}
			}
			// Cancel at the first, second or no poll point.
			for _, failFrom := range []int{0, 1, 2} {
				for _, workers := range []int{1, 2, 3, 4} {
					wantCtx := &stepCountingContext{failFrom: failFrom}
					gotCtx := &stepCountingContext{failFrom: failFrom}
					var want, got result
					want.failI, want.failJ, want.err = refParallelPairs(newBudget(wantCtx), cands, results, refFail,
						errRefused, &want.work)
					got.failI, got.failJ, got.err = parallelLoftPairsWorkers(newBudget(gotCtx), workers, cands,
						classify, &got.work)
					want.errCalls, got.errCalls = wantCtx.calls, gotCtx.calls
					require.Equal(t, want, got, "%s fail %d ctx %d workers %d", name, failAt, failFrom, workers)
				}
			}
		}
	}
}

func TestParallelLoftPairsSharesWorkersAndReleasesAfterFailure(t *testing.T) {
	prior := runtime.GOMAXPROCS(2)
	defer runtime.GOMAXPROCS(prior)
	_, held := orderedwork.ReserveWorkers(2)
	defer func() { orderedwork.ReleaseWorkers(held) }()
	cands := loftCandidates{starts: []int{0, 2, 4}, js: []int32{1, 2, 2, 3}}
	failure := errors.New("pair 0,2 refused")
	var calls, active atomic.Int32
	classify := func(i, j int) (uint8, error) {
		calls.Add(1)
		active.Add(1)
		defer active.Add(-1)
		if i == 0 && j == 2 {
			return pairClassified, failure
		}
		return pairClassified, nil
	}
	var work LoftAuditWork
	i, j, err := parallelLoftPairsWorkers(proofbound.NewWorkBudget(t.Context()), 4,
		cands, classify, &work)
	require.ErrorIs(t, err, failure)
	require.Equal(t, 0, i)
	require.Equal(t, 2, j)
	require.Equal(t, 2, work.Classifications)
	require.Equal(t, int32(2), calls.Load(), "the saturated audit has one worker")
	require.Zero(t, active.Load(), "the classifier joined before return")

	orderedwork.ReleaseWorkers(held)
	held = 0
	work = LoftAuditWork{}
	i, j, err = parallelLoftPairsWorkers(proofbound.NewWorkBudget(t.Context()), 4,
		cands, classify, &work)
	require.ErrorIs(t, err, failure)
	require.Equal(t, 0, i)
	require.Equal(t, 2, j)
	require.Equal(t, 2, work.Classifications)
	require.Zero(t, active.Load())
	workers, reserved := orderedwork.ReserveWorkers(2)
	orderedwork.ReleaseWorkers(reserved)
	require.Equal(t, 2, workers, "a failed audit released its extra worker")
}

func rowOf(c loftCandidates, k int) int {
	row := 0
	for k >= c.starts[row+1] {
		row++
	}
	return row
}

// gridMesh is an n-by-n planar grid of triangles; with pierce set, one more
// triangle crosses it.
func gridMesh(n int, pierce bool) ([]r3.Vec, [][3]int) {
	var verts []r3.Vec
	for y := 0; y <= n; y++ {
		for x := 0; x <= n; x++ {
			verts = append(verts, r3.NewVec(float64(x), float64(y), 0))
		}
	}
	var tris [][3]int
	for y := range n {
		for x := range n {
			a := y*(n+1) + x
			tris = append(tris, [3]int{a, a + 1, a + n + 2}, [3]int{a, a + n + 2, a + n + 1})
		}
	}
	if pierce {
		c := float64(n) / 2
		verts = append(verts, r3.NewVec(c+0.3, c+0.2, -1), r3.NewVec(c+0.6, c+0.2, 1), r3.NewVec(c+0.4, c+0.7, 1))
		tris = append(tris, [3]int{len(verts) - 3, len(verts) - 2, len(verts) - 1})
	}
	return verts, tris
}

type auditRun struct {
	err, workErr error
	work         LoftAuditWork
	errCalls     int
}

func TestLoftCrossingAuditPollBudgetMatchesNewWorkBudget(t *testing.T) {
	for _, pierce := range []bool{false, true} {
		verts, tris := gridMesh(14, pierce)
		var refWork LoftAuditWork
		{
			ctx := &stepCountingContext{}
			var err error
			refWork, err = LoftCrossingAuditWork(proofbound.NewWorkBudget(ctx), verts, tris, loftGenericShortcuts)
			require.Equal(t, pierce, err != nil)
		}
		for _, failFrom := range []int{0, 1, 2, 3} {
			for _, scans := range []uint64{0, 100, 5000, 1 << 30} {
				run := func(newBudget func(context.Context) *proofbound.WorkBudget) auditRun {
					ctx := &stepCountingContext{failFrom: failFrom}
					r := auditRun{}
					r.err = LoftCrossingAuditBounded(newBudget(ctx), verts, tris, MaxLoftAuditTriangles, scans)
					ctx2 := &stepCountingContext{failFrom: failFrom}
					r.work, r.workErr = LoftCrossingAuditWork(newBudget(ctx2), verts, tris, loftGenericShortcuts)
					r.errCalls = ctx.calls + ctx2.calls
					return r
				}
				want, got := run(proofbound.NewWorkBudget), run(proofbound.NewPollWorkBudget)
				require.Equal(t, want, got, "pierce %v ctx %d scans %d", pierce, failFrom, scans)
			}
		}
		require.NotZero(t, refWork.Candidates)
	}
}
