package proof_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayDyadicFloatError(t *testing.T) {
	for _, v := range []float64{0, 1, -1, math.SmallestNonzeroFloat64, math.MaxFloat64, 1e-100, 1e100} {
		for _, h := range []float64{0, math.SmallestNonzeroFloat64, -1, 1e100} {
			b := proofbound.NewReplayTrialBudget()
			d := proof.DyOfFinite(v)
			got, err := proof.ReplayDyadicFloatError(t.Context(), d, h, b)
			if err != nil {
				t.Fatal(err)
			}
			want, err := proof.ReplayRationalFloatError(t.Context(), d.Rat(), h, proofbound.NewReplayTrialBudget())
			if err != nil {
				t.Fatal(err)
			}
			if math.Float64bits(got) != math.Float64bits(want) {
				t.Fatalf("v=%g h=%g got=%g want=%g", v, h, got, want)
			}
		}
	}
	d := proof.DyOfFinite(3.3)
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		_, err := proof.ReplayDyadicFloatError(ctx, d, 1.2, b)
		return err
	}
	base := proofbound.NewReplayTrialBudget()
	if err := run(t.Context(), base); err != nil {
		t.Fatal(err)
	}
	work, records := base.Usage()
	bytes := base.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		b := proofbound.NewReplayTrialBudget()
		w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
		switch mode {
		case replayBudgetTestWork:
			w++
		case replayBudgetTestRecords:
			r++
		case replayBudgetTestBytes:
			z++
		}
		if err := b.SpendWork(t.Context(), w); err != nil {
			t.Fatal(err)
		}
		if err := b.ReserveRecords(t.Context(), r); err != nil {
			t.Fatal(err)
		}
		if err := b.ReserveBytes(t.Context(), z); err != nil {
			t.Fatal(err)
		}
		err := run(t.Context(), b)
		if mode == replayBudgetTestExact && err != nil || mode != replayBudgetTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("%s: %v", mode, err)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if err := run(ctx, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
		t.Fatalf("cancel: %v", err)
	}
}
