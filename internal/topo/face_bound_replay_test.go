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

func TestReplayFaceBoundChargesIgnoreInsertionOrder(t *testing.T) {
	ctx := t.Context()
	faces := [3]*Face{{}, {}, {}}
	orders := [][3]int{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}
	for _, order := range orders {
		meter := proofbound.NewBrakeReplayBudget()
		table, err := NewFaceBoundTable(ctx, len(faces), meter)
		if err != nil {
			t.Fatal(err)
		}
		for _, i := range order {
			if err := table.Set(ctx, faces[i], float64(i), meter); err != nil {
				t.Fatal(err)
			}
		}
		for _, i := range []int{0, 0, 0, 1, 1, 2} {
			before, records := meter.Usage()
			bytes := meter.Bytes()
			value, found, err := table.Lookup(ctx, faces[i], meter)
			after, gotRecords := meter.Usage()
			if err != nil || !found || value != float64(i) || after-before != 4 {
				t.Fatalf("lookup order=%v face=%d work=%d found=%t value=%v err=%v", order, i, after-before, found, value, err)
			}
			if records != gotRecords || bytes != meter.Bytes() {
				t.Fatal("lookup allocated records or bytes")
			}
			before = after
			if err := table.Set(ctx, faces[i], float64(i), meter); err != nil {
				t.Fatal(err)
			}
			after, gotRecords = meter.Usage()
			if after-before != 4 || records != gotRecords || bytes != meter.Bytes() {
				t.Fatalf("update order=%v face=%d work=%d records=%d bytes=%d", order, i, after-before, gotRecords-records, meter.Bytes()-bytes)
			}
		}
	}
}
