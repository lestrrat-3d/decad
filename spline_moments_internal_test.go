package decad

import (
	"fmt"
	"math/big"
	"testing"
)

func literalBernsteinMonomial(values []*big.Rat) ratPoly {
	degree := len(values) - 1
	if degree < 0 {
		return nil
	}
	result := make([]*big.Rat, degree+1)
	for i := range result {
		result[i] = new(big.Rat)
	}
	for i, value := range values {
		if value.Sign() == 0 {
			continue
		}
		term := make([]*big.Rat, i+1)
		for k := range term {
			term[k] = new(big.Rat)
		}
		term[i].SetInt64(1)
		for range degree - i {
			next := make([]*big.Rat, len(term)+1)
			for k := range next {
				next[k] = new(big.Rat)
			}
			for k, coefficient := range term {
				next[k].Add(next[k], coefficient)
				next[k+1].Sub(next[k+1], coefficient)
			}
			term = next
		}
		scale := new(big.Rat).Mul(value, binomialRat(degree, i))
		for k, coefficient := range term {
			result[k].Add(result[k], new(big.Rat).Mul(coefficient, scale))
		}
	}
	return trimTestRatPoly(result)
}

func trimTestRatPoly(p ratPoly) ratPoly {
	for len(p) > 0 && p[len(p)-1].Sign() == 0 {
		p = p[:len(p)-1]
	}
	return p
}

func TestRPFromBernsteinMatchesLiteralExpansion(t *testing.T) {
	cases := []struct {
		name   string
		values []*big.Rat
	}{
		{name: "degree-zero", values: []*big.Rat{big.NewRat(-7, 3)}},
		{name: "degree-one", values: []*big.Rat{big.NewRat(5, 7), big.NewRat(-2, 9)}},
		{name: "zero-controls", values: []*big.Rat{big.NewRat(0, 1), big.NewRat(-3, 5), big.NewRat(0, 1)}},
		{name: "higher-odd-rationals", values: []*big.Rat{
			big.NewRat(-5, 7), big.NewRat(0, 1), big.NewRat(11, 13),
			big.NewRat(-17, 19), big.NewRat(23, 29), big.NewRat(-31, 37),
		}},
		{name: "negative-values", values: []*big.Rat{
			big.NewRat(-1, 3), big.NewRat(-5, 11), big.NewRat(-7, 13),
			big.NewRat(-17, 23),
		}},
		{name: "no-input", values: nil},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := rpFromBernstein(tc.values)
			want := literalBernsteinMonomial(tc.values)
			if len(got) != len(want) {
				t.Fatalf("coefficient count: got %d, want %d", len(got), len(want))
			}
			for i := range want {
				if got[i].Cmp(want[i]) != 0 {
					t.Errorf("coefficient %d: got %s, want %s", i, got[i], want[i])
				}
			}
		})
	}
}

func BenchmarkExactFreeformMomentsDegreeAndSpans(b *testing.B) {
	for _, degree := range []int{1, 3, 8, 16, 32} {
		for _, spanCount := range []int{1, 4} {
			name := fmt.Sprintf("degree=%d/spans=%d", degree, spanCount)
			spans := benchmarkMomentSpans(degree, spanCount)
			b.Run(name, func(b *testing.B) {
				b.ReportAllocs()
				for range b.N {
					_ = exactFreeformMoments(spans, false, momentSecondOrder)
				}
			})
		}
	}
}

func benchmarkMomentSpans(degree, count int) []bezierSpan {
	spans := make([]bezierSpan, count)
	for spanIndex := range spans {
		span := make(bezierSpan, degree+1)
		for i := range span {
			u := big.NewRat(int64(i*i+3*i+spanIndex+1), int64(2*i+3))
			v := big.NewRat(int64(i*i*i-2*i+spanIndex+2), int64(3*i+5))
			span[i] = ratPoint{u: u, v: v}
		}
		spans[spanIndex] = span
	}
	return spans
}
