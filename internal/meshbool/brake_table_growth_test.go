package meshbool

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeReplayTableGrowthLimits(t *testing.T) {
	ctx := t.Context()
	setup := func() (*proofbound.ReplayBudget, *replayTable[int, int]) {
		meter := proofbound.NewBrakeReplayBudget()
		table, err := newReplayGrowingTable[int, int](ctx, 32, meter, func(v int) uint64 { return uint64(v) })
		if err != nil {
			t.Fatal(err)
		}
		for i := range 16 {
			if err := table.Set(ctx, i, i); err != nil {
				t.Fatal(err)
			}
		}
		return meter, table
	}
	measure, table := setup()
	w0, r0 := measure.Usage()
	b0 := measure.Bytes()
	if err := table.Set(ctx, 16, 16); err != nil {
		t.Fatal(err)
	}
	w1, r1 := measure.Usage()
	costWork, costRecords, costBytes := w1-w0, r1-r0, measure.Bytes()-b0
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			meter, table := setup()
			work, records := meter.Usage()
			w := proofbound.BrakeReplayWork - work - costWork
			r := proofbound.BrakeReplayRecords - records - costRecords
			b := proofbound.BrakeReplayBytes - meter.Bytes() - costBytes
			switch short {
			case brakeTestWork:
				w++
			case brakeTestRecords:
				r++
			case brakeTestBytes:
				b++
			}
			if err := meter.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveBytes(ctx, b); err != nil {
				t.Fatal(err)
			}
			err := table.Set(ctx, 16, 16)
			if short == brakeTestExact {
				if err != nil || table.count != 17 {
					t.Fatalf("exact growth: count=%d error=%v", table.count, err)
				}
			} else {
				if !errors.Is(err, decaderr.ErrUnsupported) || table.count != 16 {
					t.Fatalf("one-short growth changed count: count=%d error=%v", table.count, err)
				}
				for _, entry := range table.slots {
					if entry.used && entry.key == 16 {
						t.Fatal("one-short growth inserted the new key")
					}
				}
			}
		})
	}
}
