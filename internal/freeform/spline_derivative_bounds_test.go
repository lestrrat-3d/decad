package freeform_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func TestBernsteinSquaredNormIntegralMatchesPairwiseBetaIntegral(t *testing.T) {
	t.Parallel()
	point := func(un, ud, vn, vd int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: big.NewRat(vn, vd)}
	}
	cases := [][]freeform.RatPoint{
		nil,
		{point(-1, 3, 2, 5)},
		{point(1, 2, -2, 3), point(7, 11, 3, 13)},
		{point(2, 3, -1, 7), point(-5, 11, 4, 9), point(1, 17, 3, 5), point(-7, 19, -2, 13)},
	}
	for _, g := range cases {
		got, err := freeform.BernsteinSquaredNormIntegral(nil, g)
		require.NoError(t, err)
		if len(g) == 0 {
			require.Zero(t, got.Sign())
			continue
		}
		q := len(g) - 1
		factorials := make([]*big.Int, 2*q+2)
		factorials[0] = big.NewInt(1)
		for k := 1; k < len(factorials); k++ {
			factorials[k] = new(big.Int).Mul(factorials[k-1], big.NewInt(int64(k)))
		}
		want := new(big.Rat)
		for i, a := range g {
			for j, b := range g {
				dot := new(big.Rat).Add(
					new(big.Rat).Mul(a.U, b.U),
					new(big.Rat).Mul(a.V, b.V),
				)
				weight := new(big.Rat).SetFrac(
					new(big.Int).Mul(factorials[i+j], factorials[2*q-i-j]),
					factorials[2*q+1],
				)
				want.Add(want, dot.Mul(dot, weight))
			}
		}
		require.Zero(t, got.Cmp(want))
	}
}

func TestSpanHodographGapSquaredMatchesRationalHull(t *testing.T) {
	t.Parallel()
	point := func(un, ud, vn, vd int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: big.NewRat(vn, vd)}
	}
	cases := []freeform.BezierSpan{
		nil,
		{point(3, 7, -2, 5)},
		{point(1, 2, -2, 3), point(7, 11, 3, 13)},
		{point(2, 3, -1, 7), point(-5, 11, 4, 9), point(1, 17, 3, 5), point(-7, 19, -2, 13)},
	}
	for _, span := range cases {
		got, err := freeform.SpanHodographGapSquared(nil, span)
		require.NoError(t, err)
		if len(span) < 2 {
			require.Zero(t, got.Sign())
			continue
		}
		last := span[len(span)-1]
		du := new(big.Rat).Sub(last.U, span[0].U)
		dv := new(big.Rat).Sub(last.V, span[0].V)
		degree := big.NewRat(int64(len(span)-1), 1)
		want := new(big.Rat)
		for i := 0; i+1 < len(span); i++ {
			hu := new(big.Rat).Sub(span[i+1].U, span[i].U)
			hu.Mul(hu, degree).Sub(hu, du)
			hv := new(big.Rat).Sub(span[i+1].V, span[i].V)
			hv.Mul(hv, degree).Sub(hv, dv)
			squared := new(big.Rat).Add(new(big.Rat).Mul(hu, hu), new(big.Rat).Mul(hv, hv))
			if squared.Cmp(want) > 0 {
				want.Set(squared)
			}
		}
		require.Zero(t, got.Cmp(want))
	}
}

// zigzagHuggingSpan is a cubic whose control points all lie on its own chord
// segment [(0,0),(1,0)], two of them clustered at u = 1/1000. Its sagitta is
// exactly 0, but the curve's position at parameter t runs far behind the
// chord point at the same t, so its parameter-matched deviation is large.
func zigzagHuggingSpan() freeform.BezierSpan {
	point := func(un, ud int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: new(big.Rat)}
	}
	return freeform.BezierSpan{point(0, 1), point(1, 1000), point(1, 1000), point(1, 1)}
}

// denseMatchedDeviation samples max_t |C(t) − (C(0) + t·(C(1) − C(0)))| on
// samples+1 parameters, evaluating C by float de Casteljau.
func denseMatchedDeviation(span freeform.BezierSpan, samples int) float64 {
	control := make([][2]float64, len(span))
	for i, p := range span {
		control[i][0], _ = p.U.Float64()
		control[i][1], _ = p.V.Float64()
	}
	start, end := control[0], control[len(control)-1]
	worst := 0.0
	work := make([][2]float64, len(control))
	for step := range samples + 1 {
		at := float64(step) / float64(samples)
		copy(work, control)
		for n := len(work); n > 1; n-- {
			for i := range n - 1 {
				work[i][0] = (1-at)*work[i][0] + at*work[i+1][0]
				work[i][1] = (1-at)*work[i][1] + at*work[i+1][1]
			}
		}
		chordU := start[0] + at*(end[0]-start[0])
		chordV := start[1] + at*(end[1]-start[1])
		worst = math.Max(worst, math.Hypot(work[0][0]-chordU, work[0][1]-chordV))
	}
	return worst
}

// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses pins why the loft's
// free-form walk bisects on the matched departure as well as the sagitta
// (agent-docs/loft-gear-bounds-design.md §5): on zigzagHuggingSpan the
// sagitta is exactly 0 while the sampled parameter-matched deviation is above
// 0.3, and SpanMatchedDeltaUpper encloses that deviation.
//
// Shown to fail: returning SpanSagittaUpper from SpanMatchedDeltaUpper fails
// the enclosure assertion, since that reading is 0 here.
func TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses(t *testing.T) {
	t.Parallel()
	span := zigzagHuggingSpan()

	sagitta, err := freeform.SpanSagittaUpper(nil, span)
	require.NoError(t, err)
	require.Zero(t, sagitta, "every control point lies on the chord segment")

	dense := denseMatchedDeviation(span, 200_000)
	require.Greater(t, dense, 0.3, "the fixture must carry a large parameter-matched deviation")

	matched, err := freeform.SpanMatchedDeltaUpper(nil, span)
	require.NoError(t, err)
	require.GreaterOrEqual(t, matched, dense, "the matched bound must enclose what the sagitta misses")
}
