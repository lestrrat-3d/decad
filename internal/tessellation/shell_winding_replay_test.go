package tessellation_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

func TestReplayRequireShellWindingParityAndLimits(t *testing.T) {
	vertices, outward := windingCube()
	for _, tc := range []struct {
		name      string
		triangles [][3]int
		void      bool
		wantError bool
	}{
		{"outer", outward, false, false},
		{"void", inverted(outward), true, false},
		{"wrong outer", inverted(outward), false, true},
		{"open shell", inverted(outward)[:len(outward)-1], false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter := proofbound.NewReplayTrialBudget()
			err := tessellation.ReplayRequireShellWinding(t.Context(), vertices, tc.triangles,
				shellIndex(len(tc.triangles), 0), []bool{tc.void}, meter)
			if (err != nil) != tc.wantError {
				t.Fatalf("replay refusal=%v want=%v", err, tc.wantError)
			}
			ordinary := tessellation.RequireShellWinding(t.Context(), vertices, tc.triangles,
				shellIndex(len(tc.triangles), 0), []bool{tc.void})
			if (ordinary != nil) != (err != nil) {
				t.Fatalf("ordinary refusal=%v replay refusal=%v", ordinary, err)
			}
		})
	}
	trial := proofbound.NewReplayTrialBudget()
	indices := shellIndex(len(outward), 0)
	if err := tessellation.ReplayRequireShellWinding(t.Context(), vertices, outward, indices,
		[]bool{false}, trial); err != nil {
		t.Fatal(err)
	}
	work, records := trial.Usage()
	bytes := trial.Bytes()
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{"exact", work, records, bytes, true},
		{"work short", work - 1, records, bytes, false},
		{"records short", work, records - 1, bytes, false},
		{"bytes short", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			meter, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			if err != nil {
				t.Fatal(err)
			}
			err = tessellation.ReplayRequireShellWinding(t.Context(), vertices, outward, indices,
				[]bool{false}, meter)
			if (err == nil) != tc.want {
				t.Fatalf("limit refusal=%v want admission=%t", err, tc.want)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := tessellation.ReplayRequireShellWinding(ctx, vertices, outward, indices,
		[]bool{false}, proofbound.NewReplayTrialBudget()); err == nil {
		t.Fatal("cancelled shell audit was admitted")
	}
}
