package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayFaceBounds(t *testing.T) {
	tris := [][3]int{{0, 1, 2}, {2, 1, 3}, {0, 2, 3}}
	source := []int{2, 1, 2}
	faces := []int{1, 2}
	trim := []float64{.01, .02}
	axial := []float64{.001, .002}
	store := []float64{1e-4, 2e-4, 3e-4, 4e-4}
	want, err := FaceBounds(tris, source, map[int]float64{1: trim[0], 2: trim[1]}, map[int]float64{1: axial[0], 2: axial[1]}, store, .0001)
	if err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) ([]float64, error) {
		return ReplayFaceBounds(ctx, tris, source, faces, trim, axial, store, .0001, b)
	}
	baseline := proofbound.NewReplayTrialBudget()
	got, err := run(context.Background(), baseline)
	if err != nil || len(got) != 2 || got[0] != want[1] || got[1] != want[2] {
		t.Fatalf("bound mismatch error=%v", err)
	}
	w, r := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{{replayTestCaseExact, w, r, bytes, true}, {replayTestCaseWorkShort, w - 1, r, bytes, false}, {replayTestCaseRecordsShort, w, r - 1, bytes, false}, {replayTestCaseBytesShort, w, r, bytes - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			m, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			out, e := run(context.Background(), m)
			if tc.pass && (e != nil || len(out) != 2) {
				t.Fatalf("exact error=%v", e)
			}
			if !tc.pass && (e == nil || out != nil) {
				t.Fatalf("rollback error=%v", e)
			}
		})
	}
	faces[1] = 1
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("duplicate face accepted")
	}
	faces[1] = 2
	source[0] = 3
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("unknown face accepted")
	}
	source[0] = 2
	tris[0][0] = 99
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("invalid index accepted")
	}
	tris[0][0] = 0
	store[0] = math.Inf(1)
	if out, e := run(context.Background(), proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("nonfinite store accepted")
	}
	store[0] = 1e-4
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &replayCloseCancel{Context: lateBase, cancel: lateCancel}
	if out, e := run(late, proofbound.NewReplayTrialBudget()); e == nil || out != nil || late.polls.Load() < 20 {
		t.Fatalf("late cancellation polls=%d error=%v", late.polls.Load(), e)
	}
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if out, e := run(ctx, proofbound.NewReplayTrialBudget()); e == nil || out != nil {
		t.Fatal("cancelled audit succeeded")
	}
}
