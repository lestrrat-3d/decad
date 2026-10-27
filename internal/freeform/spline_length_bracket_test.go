package freeform_test

import (
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func floatSpan(coords ...float64) freeform.BezierSpan {
	span := make(freeform.BezierSpan, 0, len(coords)/2)
	for i := 0; i+1 < len(coords); i += 2 {
		span = append(span, freeform.RatPoint{
			U: new(big.Rat).SetFloat64(coords[i]),
			V: new(big.Rat).SetFloat64(coords[i+1]),
		})
	}
	return span
}

// wideRat is a random value of about 2^magnitude whose denominator is
// odd · 2^denPow, the shape a converted fit-spline control point takes once
// Boehm insertion and the closed-spline identity have divided float records.
func wideRat(r *rand.Rand, magnitude, denPow uint, odd int64) *big.Rat {
	num := new(big.Int)
	for range (magnitude+denPow)/64 + 2 {
		num.Lsh(num, 64)
		num.Or(num, new(big.Int).SetUint64(r.Uint64()))
	}
	num.Rsh(num, uint(num.BitLen())-(magnitude+denPow))
	if r.IntN(2) == 0 {
		num.Neg(num)
	}
	den := new(big.Int).Lsh(big.NewInt(odd), denPow)
	return new(big.Rat).SetFrac(num, den)
}

func wideSpan(r *rand.Rand, n int, magnitude, denPow uint, odd int64) freeform.BezierSpan {
	span := make(freeform.BezierSpan, n)
	for i := range span {
		span[i] = freeform.RatPoint{U: wideRat(r, magnitude, denPow, odd), V: wideRat(r, magnitude, denPow, odd)}
	}
	return span
}

func randomSpan(r *rand.Rand) freeform.BezierSpan {
	n := 2 + r.IntN(7)
	span := make(freeform.BezierSpan, n)
	scale := math.Ldexp(1, r.IntN(121)-60)
	odd := []int64{1, 3, 5, 6, 7, 9}[r.IntN(6)]
	coord := func() *big.Rat {
		out := new(big.Rat).SetFloat64((r.Float64()*2 - 1) * scale)
		if r.IntN(3) == 0 {
			// A second float of a far smaller scale stretches the dyadic
			// denominator past one float's precision.
			out.Add(out, new(big.Rat).SetFloat64(math.Ldexp(r.Float64(), r.IntN(200)-240)))
		}
		return out.Quo(out, new(big.Rat).SetInt64(odd))
	}
	for i := range span {
		span[i] = freeform.RatPoint{U: coord(), V: coord()}
	}
	if r.IntN(5) == 0 {
		// A repeated control point makes one control-polygon leg zero.
		j := r.IntN(n - 1)
		span[j+1] = span[j]
	}
	return span
}

// TestSpanLengthBracketMatchesReference holds the fixed-depth bracket to the
// reference copy in spline_length_reference_test.go bit for bit, on spans
// chosen to reach every branch the integer form has: collapsed nets, zero
// legs, degree 1, near-straight and looping cubics, both ends of the float
// range, odd denominators and very wide dyadic ones. Under the race detector
// it draws 40 random spans instead of 200 and splits them at most five times.
func TestSpanLengthBracketMatchesReference(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(11, 12))
	type namedSpan struct {
		name string
		span freeform.BezierSpan
	}
	spans := []namedSpan{
		{"ordinary cubic", floatSpan(0, 0, 1, 2, 3, 2, 4, 0)},
		{"collapsed cubic", floatSpan(1.5, -2, 1.5, -2, 1.5, -2, 1.5, -2)},
		{"collapsed line", floatSpan(7, 7, 7, 7)},
		{"line", floatSpan(0, 0, 3, 4)},
		{"near-straight cubic", floatSpan(0, 0, 1, 1e-12, 2, -1e-12, 3, 0)},
		{"closed loop", floatSpan(0, 0, 10, 10, -10, 10, 0, 0)},
		{"cusp", floatSpan(0, 0, 1, 1, 0, 1, 1, 0)},
		{"repeated inner control", floatSpan(0, 0, 1, 2, 1, 2, 4, 0)},
		{"near-duplicate pair", floatSpan(0, 0, 1, 1, math.Nextafter(1, 2), 1, 3, 0)},
		{"huge", floatSpan(0, 0, 1e300, 2e300, 3e300, 2e300, 4e300, 0)},
		{"past MaxFloat64", floatSpan(-1.5e308, 0, 1.7e308, 1.7e308, -1.7e308, -1.7e308, 1.5e308, 0)},
		{"tiny", floatSpan(0, 0, 1e-300, 2e-300, 3e-300, 2e-300, 4e-300, 0)},
		{"subnormal", floatSpan(0, 0, 5e-324, 1e-323, 1.5e-323, 1e-323, 2e-323, 0)},
		{"huge and tiny", floatSpan(5e-324, 0, 1e200, 1e-200, -1e200, 3, 1e-300, 1e150)},
		{"thirds", freeform.BezierSpan{
			{U: big.NewRat(1, 3), V: big.NewRat(2, 7)},
			{U: big.NewRat(5, 6), V: big.NewRat(-1, 9)},
			{U: big.NewRat(11, 3), V: big.NewRat(4, 21)},
			{U: big.NewRat(13, 5), V: big.NewRat(0, 1)},
		}},
		{"fit-spline denominators", wideSpan(r, 4, 7, 166, 3)},
		{"fit-spline denominators, odd 9", wideSpan(r, 4, 7, 168, 9)},
		{"wide dyadic denominators", wideSpan(r, 4, 2, 600, 1)},
		{"quintic", wideSpan(r, 6, 3, 40, 5)},
		{"degree 31", wideSpan(r, 32, 4, 60, 3)},
	}
	named := len(spans)
	random := 200
	if raceDetector {
		random = 40
	}
	for i := range random {
		spans = append(spans, namedSpan{fmt.Sprintf("random %d", i), randomSpan(r)})
	}
	for k, tc := range spans {
		depths := []int{0, 1, 2, 3, 5, freeform.FreeformLengthDepth}
		switch {
		case len(tc.span) > 8:
			depths = []int{0, 1, 3, 6}
		case raceDetector && k >= named:
			depths = []int{0, 1, 2, 3, 5}
		}
		for _, depth := range depths {
			wantLo, wantHi := refSpanLengthBracket(tc.span, depth)
			gotLo, gotHi := freeform.SpanLengthBracketUncached(tc.span, depth)
			require.Equal(t, math.Float64bits(wantLo), math.Float64bits(gotLo), "%s depth %d lower %v %v", tc.name, depth, wantLo, gotLo)
			require.Equal(t, math.Float64bits(wantHi), math.Float64bits(gotHi), "%s depth %d upper %v %v", tc.name, depth, wantHi, gotHi)
		}
	}
}

// BenchmarkSpanLengthBracket measures one uncached production-depth bracket.
// The fit-spline case is the shape loft record validation brackets: a cubic
// whose control coordinates carry about 170-bit numerators over an
// odd · 2^166 denominator.
func BenchmarkSpanLengthBracket(b *testing.B) {
	r := rand.New(rand.NewPCG(5, 6))
	cases := []struct {
		name string
		span freeform.BezierSpan
	}{
		{"integer cubic", floatSpan(0, 0, 1, 2, 3, 2, 4, 0)},
		{"fit-spline cubic", wideSpan(r, 4, 7, 166, 3)},
		{"degree 7", wideSpan(r, 8, 4, 60, 3)},
	}
	for _, tc := range cases {
		b.Run(tc.name, func(b *testing.B) {
			b.ReportAllocs()
			for b.Loop() {
				lo, hi := freeform.SpanLengthBracketUncached(tc.span, freeform.FreeformLengthDepth)
				if !(lo <= hi) {
					b.Fatalf("inverted bracket [%v, %v]", lo, hi)
				}
			}
		})
	}
}
