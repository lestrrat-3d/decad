package tessellation

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestReplayOrientationSign(t *testing.T) {
	vertices := []r3.Vec{{}, {X: 1}, {Y: 1}, {Z: 1}}
	triangles := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) (int, error) {
		return ReplayOrientationSign(ctx, vertices, triangles, b)
	}
	base := proofbound.NewReplayTrialBudget()
	got, err := run(context.Background(), base)
	want := OrientationSign(vertices, triangles, r3.Vec{})
	if err != nil || got != want || want == 0 {
		t.Fatalf("got=%d want=%d error=%v", got, want, err)
	}
	w, r := base.Usage()
	bytes := base.Bytes()
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
			got, e := run(context.Background(), m)
			if tc.pass && (e != nil || got != want) {
				t.Fatalf("exact=%d error=%v", got, e)
			}
			if !tc.pass && (e == nil || got != 0) {
				t.Fatalf("rollback=%d error=%v", got, e)
			}
		})
	}
	for i := range triangles {
		triangles[i][1], triangles[i][2] = triangles[i][2], triangles[i][1]
	}
	got, err = run(context.Background(), proofbound.NewReplayTrialBudget())
	if err != nil || got != -want {
		t.Fatalf("reverse=%d error=%v", got, err)
	}
	vertices[3].Z = 0
	got, err = run(context.Background(), proofbound.NewReplayTrialBudget())
	if err != nil || got != 0 {
		t.Fatalf("flat=%d error=%v", got, err)
	}
	vertices[3].Z = math.Inf(1)
	if _, err := run(context.Background(), proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("nonfinite vertex accepted")
	}
	vertices[3].Z = 1
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &replayCloseCancel{Context: lateBase, cancel: lateCancel}
	if sign, err := run(late, proofbound.NewReplayTrialBudget()); err == nil || sign != 0 || late.polls.Load() < 20 {
		t.Fatalf("late cancellation=%d polls=%d error=%v", sign, late.polls.Load(), err)
	}
}
