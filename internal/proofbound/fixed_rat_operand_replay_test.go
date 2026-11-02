package proofbound

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
)

func TestReplayFixedRatOperandMatchesExistingMeter(t *testing.T) {
	q := big.NewRat(-17, 13)
	other := big.NewInt(23)
	for _, tc := range []struct {
		name             string
		numerator, first bool
		op               byte
		shift            uint
	}{
		{"numerator product", true, true, '*', 0}, {"denominator quotient", false, false, '/', 0},
		{"denominator remainder", false, false, '%', 0}, {"numerator remainder", true, true, '%', 0},
		{"numerator copy", true, true, 'c', 0}, {"denominator copy", false, true, 'c', 0},
		{"numerator shift", true, true, 'l', 9},
	} {
		t.Run(tc.name, func(t *testing.T) {
			part := q.Denom()
			if tc.numerator {
				part = q.Num()
			}
			a, b := part, other
			if !tc.first {
				a, b = other, part
			}
			if tc.op == 'c' || tc.op == 'l' {
				b = nil
			}
			oldBudget := NewReplayTrialBudget()
			want, err := replayFixedIntOp(t.Context(), a, b, tc.op, tc.shift, oldBudget)
			if err != nil {
				t.Fatal(err)
			}
			run := func(ctx context.Context, budget *ReplayBudget) (*big.Int, error) {
				return proof.ReplayFixedRatInt(ctx, q, tc.numerator, other, tc.first, tc.op, tc.shift, budget)
			}
			budget := NewReplayTrialBudget()
			got, err := run(t.Context(), budget)
			if err != nil || got.Cmp(want) != 0 {
				t.Fatalf("value mismatch: %v", err)
			}
			ow, or := oldBudget.Usage()
			w, r := budget.Usage()
			z := budget.Bytes()
			if w != ow || r != or || z != oldBudget.Bytes() {
				t.Fatalf("charge mismatch %d/%d/%d versus %d/%d/%d", w, r, z, ow, or, oldBudget.Bytes())
			}
			for i := range 4 {
				ww, rr, zz := w+1, r+1, z+1
				switch i {
				case 1:
					ww--
				case 2:
					rr--
				case 3:
					zz--
				}
				limited, e := NewReplayBudgetLimits(ww, rr, zz)
				if e != nil {
					t.Fatal(e)
				}
				if e := limited.ReserveBundle(t.Context(), 1, 1, 1); e != nil {
					t.Fatal(e)
				}
				value, e := run(t.Context(), limited)
				if i == 0 {
					if e != nil || value.Cmp(want) != 0 {
						t.Fatalf("exact limit: %v", e)
					}
				} else if e == nil || value != nil {
					t.Fatalf("one-short %d succeeded", i)
				}
			}
			canceled, cancel := context.WithCancel(t.Context())
			cancel()
			if _, err = run(canceled, NewReplayTrialBudget()); !errors.Is(err, context.Canceled) {
				t.Fatalf("cancellation: %v", err)
			}
		})
	}
}

func TestReplayFixedRatQuotientAndCopy(t *testing.T) {
	q := big.NewRat(-17, 13)
	first := NewReplayTrialBudget()
	want, err := replayFixedIntOp(t.Context(), q.Num(), q.Denom(), '/', 0, first)
	if err != nil {
		t.Fatal(err)
	}
	second := NewReplayTrialBudget()
	got, err := proof.ReplayFixedRatQuo(t.Context(), q, second)
	if err != nil || got.Cmp(want) != 0 {
		t.Fatalf("quotient: %v", err)
	}
	w, r := first.Usage()
	x, y := second.Usage()
	if w != x || r != y || first.Bytes() != second.Bytes() {
		t.Fatal("quotient meter differs")
	}
	first = NewReplayTrialBudget()
	wantRat, err := proof.ReplayReducedFraction(t.Context(), q.Num(), q.Denom(), first)
	if err != nil {
		t.Fatal(err)
	}
	second = NewReplayTrialBudget()
	gotRat, err := proof.ReplayCopyRat(t.Context(), q, second)
	if err != nil || gotRat.Cmp(wantRat) != 0 {
		t.Fatalf("copy: %v", err)
	}
	w, r = first.Usage()
	x, y = second.Usage()
	if w != x || r != y || first.Bytes() != second.Bytes() {
		t.Fatal("copy meter differs")
	}
	gotRat.SetInt64(0)
	if q.Sign() == 0 {
		t.Fatal("copy aliases source")
	}
}
