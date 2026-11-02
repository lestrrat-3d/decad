package stackedrecord

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestNameOwnWallsReplayParityAndLimits(t *testing.T) {
	col := WallColumn{Column: Column{Start: 12, Region: 34}} //nolint:modernize // Named fields leave the remaining fields zero.
	makeFaces := func() []*topo.Face {
		return []*topo.Face{{Origins: []provenance.Ref{{Role: "side(0,1)"}, {Role: "side(2,3)"}}}}
	}
	want := makeFaces()
	if err := (OwnPlan{}).NameWalls(t.Context(), col, want, provenance.Producer(0)); err != nil {
		t.Fatal(err)
	}
	run := func(ctx context.Context, budget *proofbound.ReplayBudget) ([]*topo.Face, error) {
		got := makeFaces()
		if err := nameOwnWallsReplay(ctx, col, got, budget); err != nil {
			return nil, err
		}
		return got, nil
	}
	meter := proofbound.NewReplayTrialBudget()
	got, err := run(t.Context(), meter)
	if err != nil {
		t.Fatal(err)
	}
	for i := range got[0].Origins {
		if got[0].Origins[i].Role != want[0].Origins[i].Role {
			t.Fatalf("role %d differs", i)
		}
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{replayTestCaseExact, work, records, bytes, true},
		{replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false},
		{replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, e := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
			if e != nil {
				t.Fatal(e)
			}
			out, e := run(t.Context(), budget)
			if tc.pass && (e != nil || out == nil) || !tc.pass && (e == nil || out != nil) {
				t.Fatalf("out=%t err=%v", out != nil, e)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter = proofbound.NewReplayTrialBudget()
	if out, err := run(ctx, meter); err == nil || out != nil {
		t.Fatalf("cancelled out=%t err=%v", out != nil, err)
	}
}
