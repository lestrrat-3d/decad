package loftmesh

import (
	"errors"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// randomBoxes returns n boxes with centres in [0,10]x[0,10]x[0,1] and
// half-extents up to maxHalf.
func randomBoxes(seed uint64, n int, maxHalf float64) [][2]r3.Vec {
	rng := rand.New(rand.NewPCG(seed, 2))
	boxes := make([][2]r3.Vec, n)
	for i := range boxes {
		c := r3.NewVec(10*rng.Float64(), 10*rng.Float64(), rng.Float64())
		h := r3.NewVec(maxHalf*rng.Float64(), maxHalf*rng.Float64(), maxHalf*rng.Float64())
		boxes[i] = [2]r3.Vec{c.Sub(h), c.Add(h)}
	}
	return boxes
}

func allMembers(n int) []int {
	members := make([]int, n)
	for i := range members {
		members[i] = i
	}
	return members
}

// countingBudget wraps a wall budget and counts successful steps.
func countingBudget(limit uint64, steps *uint64) *proofbound.WorkBudget {
	inner := proofbound.NewWallWorkBudget(limit)
	return &proofbound.WorkBudget{
		StepFn: func() error {
			err := inner.Step()
			if err == nil {
				*steps++
			}
			return err
		},
		ErrFn: inner.Err,
	}
}

func unlimitedBudget(t *testing.T) *proofbound.WorkBudget {
	return proofbound.NewWorkBudget(t.Context())
}

type rowScanner interface {
	loftRowScan
}

func TestWalkRowMatchesVisitRow(t *testing.T) {
	boxes := randomBoxes(1, 1200, 0.35)
	members := allMembers(len(boxes))
	family := make([]int, len(boxes))
	for i := range family {
		family[i] = i%3 - 1 // -1, 0, 1: one unproven family and two proven
	}
	scans := map[string]rowScanner{
		"tree":    newTreeScan(boxes, members),
		"grouped": newGroupedTreeScan(boxes, members, family),
	}
	for name, scan := range scans {
		t.Run(name, func(t *testing.T) {
			for i := 0; i < len(boxes); i += 7 {
				var want []int
				work, exceeded, err := scan.visitRow(unlimitedBudget(t), i, math.MaxUint64,
					func(j int) { want = append(want, j) })
				require.NoError(t, err)
				require.False(t, exceeded)
				for _, limit := range []uint64{0, 1, 17, work / 2, max(work, 1) - 1, work, math.MaxUint64} {
					var wantPart, gotPart []int
					wantWork, wantExceeded, err := scan.visitRow(unlimitedBudget(t), i, limit,
						func(j int) { wantPart = append(wantPart, j) })
					require.NoError(t, err)
					gotWork, gotExceeded := scan.walkRow(i, limit, func(j int) { gotPart = append(gotPart, j) })
					require.Equal(t, wantWork, gotWork)
					require.Equal(t, wantExceeded, gotExceeded)
					require.Equal(t, wantPart, gotPart)
				}
				var got []int
				gotWork, _ := scan.walkRow(i, math.MaxUint64, func(j int) { got = append(got, j) })
				require.Equal(t, work, gotWork)
				require.Equal(t, want, got)
			}
		})
	}
}

func TestCountParallelMatchesVisit(t *testing.T) {
	boxes := randomBoxes(3, 1500, 0.3)
	tree := newTreeScan(boxes, allMembers(len(boxes)))
	f := len(boxes)
	full, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, f, keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	total := full.scanned
	require.Greater(t, total, uint64(1000))
	keeps := map[string]func(i, j int) bool{
		"every": keepEveryPair,
		"mod3":  func(i, j int) bool { return (i+j)%3 != 0 },
	}
	limits := []uint64{0, 5, total / 3, total - 1, total, math.MaxUint64}
	budgets := []uint64{0, 1, total / 2, total, total + 1, math.MaxUint64}
	for name, keep := range keeps {
		t.Run(name, func(t *testing.T) {
			for _, limit := range limits {
				for _, k := range budgets {
					var wantSteps, gotSteps uint64
					want, wantErr := sweepCandidateCountsWorkers(countingBudget(k, &wantSteps), tree, f, keep, limit, 1)
					got, gotErr := tree.countParallel(countingBudget(k, &gotSteps), f, keep, limit, 4)
					require.Equal(t, want, got, "limit %d budget %d", limit, k)
					require.Equal(t, wantErr, gotErr, "limit %d budget %d", limit, k)
					require.Equal(t, wantSteps, gotSteps, "limit %d budget %d", limit, k)
				}
			}
		})
	}
}

func TestCountParallelGate(t *testing.T) {
	boxes := randomBoxes(4, loftTreeParallelMin, 0.1)
	tree := newTreeScan(boxes, allMembers(len(boxes)))
	f := len(boxes)
	want, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, f, keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	got, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, f, keepEveryPair, math.MaxUint64, 4)
	require.NoError(t, err)
	require.Equal(t, want, got)
}

func TestPairTasksPartitionWalk(t *testing.T) {
	boxes := randomBoxes(5, 1500, 0.3)
	tree := newTreeScan(boxes, allMembers(len(boxes)))
	tasks, used := tree.pairTasks(256)
	require.GreaterOrEqual(t, len(tasks), 256)
	total := used
	for _, task := range tasks {
		w := pairWalker{t: &tree, limit: math.MaxUint64, fn: func(int, int) {}}
		require.False(t, w.walk(task[0], task[1]))
		total += w.scanned
	}
	scanned, exceeded, err := tree.visit(unlimitedBudget(t), math.MaxUint64, func(int, int) {})
	require.NoError(t, err)
	require.False(t, exceeded)
	require.Equal(t, scanned, total)
}

func TestPreflightLoftRowsParallelMatchesSerial(t *testing.T) {
	boxes := randomBoxes(6, 1500, 0.3)
	members := allMembers(len(boxes))
	tree := newTreeScan(boxes, members)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, len(boxes), keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	var rowSteps uint64
	require.NoError(t, preflightLoftRowsWorkers(countingBudget(math.MaxUint64, &rowSteps), tree, members,
		sc.starts, math.MaxUint64, 1))
	total := rowSteps
	tampered := append([]int(nil), sc.starts...)
	for i := 500; i < len(tampered); i++ {
		tampered[i]++
	}
	for name, starts := range map[string][]int{"clean": sc.starts, "tampered": tampered} {
		t.Run(name, func(t *testing.T) {
			for _, ceiling := range []uint64{total / 2, total - 1, total, math.MaxUint64} {
				for _, k := range []uint64{0, 1, total / 2, total, total + 1, math.MaxUint64} {
					var wantSteps, gotSteps uint64
					wantErr := preflightLoftRowsWorkers(countingBudget(k, &wantSteps), tree, members, starts,
						ceiling, 1)
					gotErr := preflightLoftRowsWorkers(countingBudget(k, &gotSteps), tree, members, starts,
						ceiling, 4)
					require.Equal(t, wantErr, gotErr, "ceiling %d budget %d", ceiling, k)
					require.Equal(t, wantSteps, gotSteps, "ceiling %d budget %d", ceiling, k)
				}
			}
		})
	}
	gotErr := preflightLoftRowsWorkers(unlimitedBudget(t), tree, members, tampered, math.MaxUint64, 4)
	require.Error(t, gotErr)
	require.False(t, errors.Is(gotErr, proofbound.ErrWallWorkBudget))
}

func TestLoftRowChunkEnd(t *testing.T) {
	n := 3 * loftRowChunkRows
	members := allMembers(n)
	starts := make([]int, n+1)
	for i := range n {
		starts[i+1] = starts[i] + 1
	}
	require.Equal(t, loftRowChunkRows, loftRowChunkEnd(members, starts, 0))
	require.Equal(t, n, loftRowChunkEnd(members, starts, n-3))
	// A row heavier than a batch still forms a chunk of its own.
	starts = make([]int, n+1)
	for i := range n {
		starts[i+1] = starts[i] + loftAuditCandidateBatchSize
	}
	require.Equal(t, 1, loftRowChunkEnd(members, starts, 0))
	require.Equal(t, 4, loftRowChunkEnd(members, starts, 3))
}

func TestStreamLoftPairsParallelMatchesSerial(t *testing.T) {
	// More members than one chunk, with few partners per row, so the stream
	// crosses chunk and batch boundaries cheaply.
	n := loftRowChunkRows + 700
	boxes := randomBoxes(7, n, 0.08)
	for i := range boxes { // spread over a wider area to keep rows short
		for k := range boxes[i] {
			boxes[i][k].X *= 5
			boxes[i][k].Y *= 5
		}
	}
	members := allMembers(n)
	tree := newTreeScan(boxes, members)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, n, keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	require.Greater(t, sc.count, uint64(1000))
	type pair struct{ i, j int }
	type result struct {
		failI, failJ int
		err          error
		pairs        []pair
		work         LoftAuditWork
		steps        uint64
	}
	run := func(batchSize, failAt, workers int, ceiling uint64) result {
		var r result
		budget := countingBudget(math.MaxUint64, &r.steps)
		testPair := func(i, j int) error {
			r.pairs = append(r.pairs, pair{i, j})
			if len(r.pairs) == failAt {
				return errors.New("refused")
			}
			return nil
		}
		r.failI, r.failJ, r.err = streamLoftPairsWorkers(budget, tree, members, sc.starts, sc.count, ceiling,
			batchSize, nil, boxes, nil, LoftAuditShortcuts{}, &r.work, testPair, workers)
		return r
	}
	for _, batchSize := range []int{1_000, 100_000} {
		for _, failAt := range []int{0, 1, int(sc.count) / 2, int(sc.count)} {
			want := run(batchSize, failAt, 1, math.MaxUint64)
			got := run(batchSize, failAt, 4, math.MaxUint64)
			require.Equal(t, want, got, "batch %d fail %d", batchSize, failAt)
		}
	}
	for _, ceiling := range []uint64{sc.scanned / 2, sc.scanned - 1} {
		want := run(1_000, 0, 1, ceiling)
		got := run(1_000, 0, 4, ceiling)
		require.Error(t, want.err)
		require.Equal(t, want, got)
	}
}
