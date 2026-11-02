package tessellation

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayRequireVertexLinks(t *testing.T) {
	tetra := [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3}}
	separate := append(append([][3]int(nil), tetra...), [][3]int{{4, 5, 6}, {4, 6, 7}, {4, 7, 5}, {5, 7, 6}}...)
	pinched := append(append([][3]int(nil), tetra...), [][3]int{{0, 4, 5}, {0, 5, 6}, {0, 6, 4}, {4, 6, 5}}...)
	for _, tc := range []struct {
		vertices  int
		triangles [][3]int
	}{{4, tetra}, {8, separate}, {7, pinched}, {5, [][3]int{{0, 1, 2}, {0, 2, 3}}}} {
		want := RequireVertexLinks(context.Background(), tc.vertices, tc.triangles)
		got := ReplayRequireVertexLinks(context.Background(), tc.vertices, tc.triangles, proofbound.NewReplayTrialBudget())
		if (want == nil) != (got == nil) {
			t.Fatalf("vertices=%d triangles=%d parity differs", tc.vertices, len(tc.triangles))
		}
	}
	const ring = 600
	large := make([][3]int, 0, ring*2)
	for i := range ring {
		a, b := 2+i, 2+(i+1)%ring
		large = append(large, [3]int{0, a, b}, [3]int{1, b, a})
	}
	wantLarge := RequireVertexLinks(context.Background(), ring+2, large)
	gotLarge := ReplayRequireVertexLinks(context.Background(), ring+2, large, proofbound.NewReplayTrialBudget())
	if wantLarge != nil || gotLarge != nil {
		t.Fatalf("large vertex link ordinary=%v replay=%v", wantLarge, gotLarge)
	}
	base := proofbound.NewReplayTrialBudget()
	if err := ReplayRequireVertexLinks(context.Background(), 4, tetra, base); err != nil {
		t.Fatal(err)
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
			if e := ReplayRequireVertexLinks(context.Background(), 4, tetra, m); (e == nil) != tc.pass {
				t.Fatalf("error=%v", e)
			}
		})
	}
	if err := ReplayRequireVertexLinks(context.Background(), 4, [][3]int{{0, 1, 4}}, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("invalid index accepted")
	}
	lateBase, lateCancel := context.WithCancel(context.Background())
	defer lateCancel()
	late := &replayCloseCancel{Context: lateBase, cancel: lateCancel}
	if err := ReplayRequireVertexLinks(late, 4, tetra, proofbound.NewReplayTrialBudget()); err == nil || late.polls.Load() < 20 {
		t.Fatalf("late cancellation polls=%d error=%v", late.polls.Load(), err)
	}
}
