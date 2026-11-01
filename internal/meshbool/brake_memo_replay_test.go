package meshbool

import (
	"context"
	"errors"
	"math/big"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func TestBrakeReplayAllocationShellLimits(t *testing.T) {
	cases := []struct {
		name string
		size uintptr
		make func(*proofbound.ReplayBudget) error
	}{
		{"memo", unsafe.Sizeof(ContactMemo{}), func(b *proofbound.ReplayBudget) error {
			_, err := NewReplayContactMemo(t.Context(), &BoolMesh{}, &BoolMesh{}, b)
			return err
		}},
		{"keys", unsafe.Sizeof(ReplayKeys{}), func(b *proofbound.ReplayBudget) error { _, err := NewReplayKeysBounded(t.Context(), b); return err }},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			for _, short := range []bool{false, true} {
				b := proofbound.NewBrakeReplayBudget()
				remaining := 2*uint64(c.size) + 128
				if short {
					remaining--
				}
				if err := b.ReserveBytes(t.Context(), proofbound.BrakeReplayBytes-remaining); err != nil {
					t.Fatal(err)
				}
				err := c.make(b)
				if short && !errors.Is(err, decaderr.ErrUnsupported) {
					t.Fatalf("one-short: %v", err)
				}
				if !short && err != nil {
					t.Fatalf("exact: %v", err)
				}
			}
		})
	}
}

func TestBrakeProductScratchAllocationEnvelope(t *testing.T) {
	ctx := t.Context()
	const size = uint64(72)
	const required = 2*size + 128
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := required
		if short {
			remaining--
		}
		if err := meter.ReserveBytes(ctx, proofbound.BrakeReplayBytes-remaining); err != nil {
			t.Fatal(err)
		}
		budget := replayProductScratch{ReplayBudget: meter}
		err := budget.ReserveBytes(ctx, size)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short product allocation: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact product allocation: %v", err)
		}
	}
}

func TestBrakeSmallProductBudgetLimits(t *testing.T) {
	ctx := t.Context()
	values := [4]*big.Int{big.NewInt(-31), big.NewInt(7), big.NewInt(11), big.NewInt(3)}
	run := func(meter *proofbound.ReplayBudget) error {
		arena := &proof.ReplayProductArena{}
		budget := &replayProductBudget{ReplayBudget: meter, arena: arena, scratch: replayProductScratch{meter}}
		got, err := budget.ProductDifference(ctx, values[0], values[1], values[2], values[3])
		if err == nil && got.Int64() != -250 {
			t.Fatal("small product value changed")
		}
		return err
	}
	meter := proofbound.NewBrakeReplayBudget()
	if err := run(meter); err != nil {
		t.Fatal(err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			m := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			switch short {
			case brakeTestWork:
				w++
			case brakeTestRecords:
				r++
			case brakeTestBytes:
				b++
			}
			if err := m.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(ctx, b); err != nil {
				t.Fatal(err)
			}
			err := run(m)
			if short == brakeTestExact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short: %v", err)
			}
		})
	}
}

func TestBrakeAppendWorkChargesNewElements(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := uint64(3)
		if short {
			remaining--
		}
		if err := meter.SpendWork(ctx, proofbound.BrakeReplayWork-remaining); err != nil {
			t.Fatal(err)
		}
		backing := []int{-1, -1, -1}
		out, err := appendWork(backing[:0], []int{1, 2, 3}, meter.WorkBudget(ctx))
		if short {
			if !errors.Is(err, decaderr.ErrUnsupported) || backing[0] != -1 || backing[1] != -1 || backing[2] != -1 {
				t.Fatalf("one-short append changed storage or returned %v", err)
			}
		} else if err != nil || len(out) != 3 || out[0] != 1 || out[1] != 2 || out[2] != 3 {
			t.Fatalf("exact append failed: %v", err)
		}
	}
}

func TestBrakeAppendReplayChargesInsertion(t *testing.T) {
	ctx := t.Context()
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := uint64(1)
		if short {
			remaining = 0
		}
		if err := meter.SpendWork(ctx, proofbound.BrakeReplayWork-remaining); err != nil {
			t.Fatal(err)
		}
		budget, err := NewReplayPairCutsBudget(meter)
		if err != nil {
			t.Fatal(err)
		}
		backing := []int{-1}
		out, err := appendReplay(ctx, backing[:0], 7, budget)
		if short {
			if !errors.Is(err, decaderr.ErrUnsupported) || backing[0] != -1 {
				t.Fatalf("one-short insertion changed storage or returned %v", err)
			}
		} else if err != nil || len(out) != 1 || out[0] != 7 {
			t.Fatalf("exact insertion failed: %v", err)
		}
	}
}

func TestBrakeLargeProductRecordLimits(t *testing.T) {
	ctx := t.Context()
	large := new(big.Int).Lsh(big.NewInt(1), 200)
	one, zero := big.NewInt(1), big.NewInt(0)
	run := func(meter *proofbound.ReplayBudget) error {
		arena := &proof.ReplayProductArena{}
		budget := &replayProductBudget{ReplayBudget: meter, arena: arena, scratch: replayProductScratch{meter}}
		got, err := budget.ProductDifference(ctx, large, one, zero, zero)
		if err == nil && got.Cmp(large) != 0 {
			t.Fatal("large product value changed")
		}
		return err
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := run(measure); err != nil {
		t.Fatal(err)
	}
	_, records := measure.Usage()
	if records != 3 {
		t.Fatalf("large product charged %d records, expected three", records)
	}
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := records
		if short {
			remaining--
		}
		if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords-remaining); err != nil {
			t.Fatal(err)
		}
		err := run(meter)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact: %v", err)
		}
	}
}

func TestBrakeNearContactsFixedLimits(t *testing.T) {
	ctx := t.Context()
	measure := proofbound.NewBrakeReplayBudget()
	if err := reserveBrakeNearContactsFixed(ctx, measure); err != nil {
		t.Fatal(err)
	}
	work, records := measure.Usage()
	bytes := measure.Bytes()
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			meter := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
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
			err := reserveBrakeNearContactsFixed(ctx, meter)
			if short == brakeTestExact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short: %v", err)
			}
		})
	}
}

func TestBrakeNearContactBatchRecordLimits(t *testing.T) {
	ctx := t.Context()
	run := func(meter *proofbound.ReplayBudget) error {
		meshA, meshB := &BoolMesh{}, &BoolMesh{}
		_, _, err := gatherNearContactsSharedReplay(ctx, meshA, nil, meshB, nil, 0, nil, nil,
			ContactBatchSize, meter.WorkBudget(ctx), meter)
		return err
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := reserveBrakeNearContactsFixed(ctx, measure); err != nil {
		t.Fatal(err)
	}
	_, before := measure.Usage()
	if err := run(measure); err != nil {
		t.Fatal(err)
	}
	_, after := measure.Usage()
	cost := after - before
	if cost < 4*ContactBatchSize {
		t.Fatalf("batch reserved only %d records", cost)
	}
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		if err := reserveBrakeNearContactsFixed(ctx, meter); err != nil {
			t.Fatal(err)
		}
		_, used := meter.Usage()
		remaining := cost
		if short {
			remaining--
		}
		if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords-used-remaining); err != nil {
			t.Fatal(err)
		}
		err := run(meter)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short batch: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact batch: %v", err)
		}
	}
}

func TestBrakeStitchFixedLimits(t *testing.T) {
	ctx := t.Context()
	canceled, cancel := context.WithCancel(ctx)
	cancel()
	if err := reserveBrakeStitchFixed(canceled, proofbound.NewBrakeReplayBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("stitch cancellation: %v", err)
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := reserveBrakeStitchFixed(ctx, measure); err != nil {
		t.Fatal(err)
	}
	work, records := measure.Usage()
	bytes := measure.Bytes()
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			meter := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
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
			err := reserveBrakeStitchFixed(ctx, meter)
			if short == brakeTestExact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short: %v", err)
			}
		})
	}
}

func TestBrakeReplayGapRecordLimits(t *testing.T) {
	ctx := t.Context()
	run := func(meter *proofbound.ReplayBudget) error {
		_, bounds, err := replayGapBounds(ctx, meter.WorkBudget(ctx), nil, nil, nil, 3)
		if err == nil && len(bounds) != 3 {
			t.Fatal("gap bounds length changed")
		}
		return err
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := run(measure); err != nil {
		t.Fatal(err)
	}
	_, records := measure.Usage()
	if records != 4 {
		t.Fatalf("gap bounds charged %d records, expected four", records)
	}
	for _, short := range []bool{false, true} {
		meter := proofbound.NewBrakeReplayBudget()
		remaining := records
		if short {
			remaining--
		}
		if err := meter.ReserveRecords(ctx, proofbound.BrakeReplayRecords-remaining); err != nil {
			t.Fatal(err)
		}
		err := run(meter)
		if short && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short gaps: %v", err)
		}
		if !short && err != nil {
			t.Fatalf("exact gaps: %v", err)
		}
	}
}

func TestBrakeFaceGroupingBudgetLimits(t *testing.T) {
	ctx := t.Context()
	face := &topo.Face{}
	mesh := &topo.Mesh{Source: []*topo.Face{face, face, face, face, face}, FaceBound: map[*topo.Face]float64{face: 0}}
	boxes := make([][2]r3.Vec, len(mesh.Source))
	run := func(meter *proofbound.ReplayBudget) error {
		groups, err := brakefacesOfMesh(ctx, meter.WorkBudget(ctx), mesh, boxes)
		if err == nil && (len(groups) != 1 || groups[0].source != face || len(groups[0].facets) != len(mesh.Source)) {
			t.Fatal("face grouping changed")
		}
		return err
	}
	measure := proofbound.NewBrakeReplayBudget()
	if err := run(measure); err != nil {
		t.Fatal(err)
	}
	work, records := measure.Usage()
	bytes := measure.Bytes()
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			meter := proofbound.NewBrakeReplayBudget()
			w, r, b := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
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
			err := run(meter)
			if short == brakeTestExact && err != nil {
				t.Fatalf("exact: %v", err)
			}
			if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("one-short: %v", err)
			}
		})
	}
}
