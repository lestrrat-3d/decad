package freeform

import (
	"fmt"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/stretchr/testify/require"
)

func literalBernsteinMonomial(values []*big.Rat) polynomial.RatPoly {
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
		scale := new(big.Rat).Mul(value, BinomialRat(degree, i))
		for k, coefficient := range term {
			result[k].Add(result[k], new(big.Rat).Mul(coefficient, scale))
		}
	}
	return trimTestRatPoly(result)
}

func trimTestRatPoly(p polynomial.RatPoly) polynomial.RatPoly {
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
			got := RpFromBernstein(tc.values)
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
					_ = ExactFreeformMoments(spans, false, MomentSecondOrder)
				}
			})
		}
	}
}

func benchmarkMomentSpans(degree, count int) []survey2d.BezierSpan {
	spans := make([]survey2d.BezierSpan, count)
	for spanIndex := range spans {
		span := make(survey2d.BezierSpan, degree+1)
		for i := range span {
			u := big.NewRat(int64(i*i+3*i+spanIndex+1), int64(2*i+3))
			v := big.NewRat(int64(i*i*i-2*i+spanIndex+2), int64(3*i+5))
			span[i] = survey2d.RatPoint{U: u, V: v}
		}
		spans[spanIndex] = span
	}
	return spans
}

// TestFreeformThirdMomentsParabolicRegion integrates the region between the
// parabola v = u² and the chord v = 2u over u ∈ [0, 2]: the parabola is the
// quadratic Bézier (0,0), (1,0), (2,4) walked forward, the chord a line
// walked back. The expected ∫u^p·v^q dA = ∫₀² u^p·((2u)^(q+1) − u^(2q+2))/(q+1) du
// is integrated here column by column. A reversed span negates every term.
//
// Shown-to-fail: dropping the ¼ of ∫u³ dA, or integrating ∫v³ dA against du
// instead of dv, separates that term from its value.
func TestFreeformThirdMomentsParabolicRegion(t *testing.T) {
	point := func(u, v int64) survey2d.RatPoint { return survey2d.RatPoint{U: big.NewRat(u, 1), V: big.NewRat(v, 1)} }
	parabola := []survey2d.BezierSpan{{point(0, 0), point(1, 0), point(2, 4)}}
	curve := FreeformThirdMoments(parabola, false)
	chord := PolyThirdMoments(
		polynomial.RatPoly{big.NewRat(2, 1), big.NewRat(-2, 1)},
		polynomial.RatPoly{big.NewRat(4, 1), big.NewRat(-4, 1)},
	)
	for i, pq := range [4][2]int{{3, 0}, {2, 1}, {1, 2}, {0, 3}} {
		p, q := pq[0], pq[1]
		// ∫₀² u^p·(2^(q+1)·u^(q+1) − u^(2q+2)) du / (q+1)
		upper := new(big.Rat).SetInt64(1 << (q + 1))
		upper.Mul(upper, new(big.Rat).SetInt64(1<<(p+q+2)))
		upper.Quo(upper, big.NewRat(int64(p+q+2), 1))
		lower := new(big.Rat).SetInt64(1 << (p + 2*q + 3))
		lower.Quo(lower, big.NewRat(int64(p+2*q+3), 1))
		want := new(big.Rat).Sub(upper, lower)
		want.Quo(want, big.NewRat(int64(q+1), 1))
		got := new(big.Rat).Add(curve[i], chord[i])
		require.Zero(t, got.Cmp(want), "∫u^%d·v^%d dA: got %s, want %s", p, q, got, want)
	}
	reversed := FreeformThirdMoments(parabola, true)
	for i := range curve {
		require.Zero(t, new(big.Rat).Neg(curve[i]).Cmp(reversed[i]), "term %d: a reversed span must negate", i)
	}
}
