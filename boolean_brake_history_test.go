package decad

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestBrakeHistoryReaderBudgetAndLargeInput(t *testing.T) {
	ctx := t.Context()
	doc, target, _ := brakeGalleryFixture(t, ctx)
	meter := proofbound.NewBrakeReplayBudget()
	kind, inputs, ok, err := brakeBooleanProducer(ctx, doc, &target.n, meter)
	if err != nil || !ok || kind != meshbool.OpCut || len(inputs) != 2 {
		t.Fatalf("producer: ok=%t inputs=%d error=%v", ok, len(inputs), err)
	}
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, short := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(short, func(t *testing.T) {
			m := proofbound.NewBrakeReplayBudget()
			w, r, z := proofbound.BrakeReplayWork-work, proofbound.BrakeReplayRecords-records, proofbound.BrakeReplayBytes-bytes
			switch short {
			case brakeTestWork:
				w++
			case brakeTestRecords:
				r++
			case brakeTestBytes:
				z++
			}
			if err := m.SpendWork(ctx, w); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveRecords(ctx, r); err != nil {
				t.Fatal(err)
			}
			if err := m.ReserveBytes(ctx, z); err != nil {
				t.Fatal(err)
			}
			_, got, accepted, err := brakeBooleanProducer(ctx, doc, &target.n, m)
			if short == brakeTestExact {
				if err != nil || !accepted || len(got) != 2 || got[0] != inputs[0] || got[1] != inputs[1] {
					t.Fatalf("exact producer: ok=%t error=%v", accepted, err)
				}
			} else if !errors.Is(err, ErrUnsupported) || accepted || got != nil {
				t.Fatalf("one-short %s: ok=%t error=%v", short, accepted, err)
			}
		})
	}
	doc.n.LiveMu.Lock()
	op := doc.producers[target]
	original := op.inputs
	op.inputs = make([]OperationInput, 10000)
	doc.n.LiveMu.Unlock()
	defer func() { doc.n.LiveMu.Lock(); op.inputs = original; doc.n.LiveMu.Unlock() }()
	m := proofbound.NewBrakeReplayBudget()
	_, got, accepted, err := brakeBooleanProducer(ctx, doc, &target.n, m)
	w, r := m.Usage()
	if err != nil || accepted || got != nil || w != 2 || r != 0 || m.Bytes() != 0 {
		t.Fatalf("large input: ok=%t work=%d records=%d bytes=%d error=%v", accepted, w, r, m.Bytes(), err)
	}
}
