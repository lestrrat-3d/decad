package topo

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayFaceBoundLookupChargesFullScan(t *testing.T) {
	ctx := t.Context()
	meter := proofbound.NewReplayTrialBudget()
	table, err := NewFaceBoundTable(ctx, 3, meter)
	if err != nil {
		t.Fatal(err)
	}
	faces := [3]*Face{{}, {}, {}}
	for i, face := range faces {
		if err := table.Set(ctx, face, float64(i), meter); err != nil {
			t.Fatal(err)
		}
	}
	for i, face := range faces {
		var calls int
		value, found, err := table.LookupWork(face, func() error { calls++; return nil })
		if err != nil || !found || value != float64(i) || calls != 4 {
			t.Fatalf("face %d: found=%t value=%v calls=%d error=%v", i, found, value, calls, err)
		}
	}
	stop := errors.New("budget exhausted")
	calls := 0
	_, _, err = table.LookupWork(faces[0], func() error {
		calls++
		if calls == 4 {
			return stop
		}
		return nil
	})
	if !errors.Is(err, stop) || calls != 4 {
		t.Fatalf("one-short lookup: calls=%d error=%v", calls, err)
	}
}
