package revolvemesh

import (
	"context"
	"errors"
	"math/rand"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayContactTreeCandidateParity(t *testing.T) {
	rng := rand.New(rand.NewSource(920))
	for trial := range 20 {
		data := make([]RevolveAuditTri, 90)
		tris := make([][3]int, len(data))
		for i := range data {
			x, y, z := rng.Float64()*20-10, rng.Float64()*20-10, rng.Float64()*20-10
			dx, dy, dz := rng.Float64()*3, rng.Float64()*3, rng.Float64()*3
			data[i].Box = [2]r3.Vec{{X: x, Y: y, Z: z}, {X: x + dx, Y: y + dy, Z: z + dz}}
			tris[i] = [3]int{rng.Intn(50), rng.Intn(50), rng.Intn(50)}
		}
		margin := rng.Float64()
		want, err := revolveContactCandidates(proofbound.NewWorkBudget(t.Context()), data, tris, margin)
		if err != nil {
			t.Fatal(err)
		}
		got, _, err := replayContactCandidates(t.Context(), data, tris, margin, proofbound.MaxFacetPairTestsPerCall, proofbound.NewReplayTrialBudget())
		if err != nil {
			t.Fatal(err)
		}
		for i := range want {
			if !slices.Equal(got[i], want[i]) {
				t.Fatalf("trial %d row %d differs", trial, i)
			}
		}
	}
}

func TestReplayBoxCandidatesCountOnlyOverlappingPairs(t *testing.T) {
	data := []RevolveAuditTri{
		{Box: [2]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 1}}},
		{Box: [2]r3.Vec{{X: .5, Y: .5, Z: .5}, {X: 1.5, Y: 1.5, Z: 1.5}}},
		{Box: [2]r3.Vec{{X: 10, Y: 10, Z: 10}, {X: 11, Y: 11, Z: 11}}},
	}
	order := []int32{0, 1, 2}
	rows := make([][]int32, len(data))

	got, err := replayBoxCandidates(t.Context(), data, order, rows, 0, 1, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	if got != 1 || !slices.Equal(rows[0], []int32{1}) {
		t.Fatalf("exact candidate count = %d, row 0 = %v", got, rows[0])
	}

	rows = make([][]int32, len(data))
	got, err = replayBoxCandidates(t.Context(), data, order, rows, 0, 0, proofbound.NewReplayTrialBudget())
	if err == nil || got != 0 {
		t.Fatalf("one-short candidate count = %d, error = %v", got, err)
	}
}

func TestReplayContactCandidateCeilingExcludesDisjointBoxes(t *testing.T) {
	const n = 4_001
	data := make([]RevolveAuditTri, n)
	tris := make([][3]int, n)
	for i := range n {
		y := float64(i * 2)
		data[i].Box = [2]r3.Vec{{X: 0, Y: y, Z: 0}, {X: 1, Y: y + 1, Z: 1}}
		tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
	}
	rows, pairs, err := replayContactCandidates(t.Context(), data, tris, 0,
		proofbound.MaxFacetPairTestsPerCall, proofbound.NewReplayTrialBudget())
	if err != nil {
		t.Fatal(err)
	}
	if pairs != 0 || revolveRowsLen(rows) != 0 {
		t.Fatalf("disjoint boxes emitted %d candidates in %d pairs", revolveRowsLen(rows), pairs)
	}
}

func TestReplayContactCandidatesBudget(t *testing.T) {
	const n = 12
	data := make([]RevolveAuditTri, n)
	tris := make([][3]int, n)
	for i := range n {
		x := float64(i) * .6
		data[i].Box = [2]r3.Vec{{X: x}, {X: x + 1, Y: 1, Z: 1}}
		tris[i] = [3]int{3 * i, 3*i + 1, 3*i + 2}
	}
	run := func(ctx context.Context, m *proofbound.ReplayBudget) error {
		rows, pairs, e := replayContactCandidates(ctx, data, tris, 0, proofbound.MaxFacetPairTestsPerCall, m)
		if e != nil {
			if rows != nil {
				t.Fatal("partial rows returned")
			}
			return e
		}
		if pairs == 0 || int(pairs) != revolveRowsLen(rows) {
			t.Fatal("candidate count mismatch")
		}
		return nil
	}
	base := proofbound.NewReplayTrialBudget()
	if e := run(t.Context(), base); e != nil {
		t.Fatal(e)
	}
	w, r := base.Usage()
	z := base.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run(mode, func(t *testing.T) {
			m := proofbound.NewReplayTrialBudget()
			ww, rr, zz := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-z
			switch mode {
			case replayBudgetTestWork:
				ww++
			case replayBudgetTestRecords:
				rr++
			case replayBudgetTestBytes:
				zz++
			}
			if e := m.SpendWork(t.Context(), ww); e != nil {
				t.Fatal(e)
			}
			if e := m.ReserveRecords(t.Context(), rr); e != nil {
				t.Fatal(e)
			}
			if e := m.ReserveBytes(t.Context(), zz); e != nil {
				t.Fatal(e)
			}
			e := run(t.Context(), m)
			if mode == replayBudgetTestExact && e != nil || mode != replayBudgetTestExact && !errors.Is(e, decaderr.ErrUnsupported) {
				t.Fatalf("error=%v", e)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if e := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(e, context.Canceled) {
		t.Fatalf("cancel=%v", e)
	}
}
