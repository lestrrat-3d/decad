package freeform

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// RefineSpanChain divides every span into factor equal local-parameter parts.
// The resulting curve is identical to the input chain, including its span
// joins. A loft uses this after checking the resulting count against its
// station share, so both paired curves can use the same span-uniform slots.
func RefineSpanChain(spans []BezierSpan, factor int, work *FreeformWork) ([]BezierSpan, error) {
	if factor < 1 || len(spans) == 0 {
		return nil, fmt.Errorf(`%w: a refined Bézier chain needs spans and a positive factor`, decaderr.ErrDegenerate)
	}
	if factor > MaxChordsPerWalk || len(spans) > MaxChordsPerWalk/factor {
		return nil, ErrTooManyChords
	}
	for _, span := range spans {
		if len(span) == 0 {
			return nil, fmt.Errorf(`%w: a Bézier span needs control points`, decaderr.ErrDegenerate)
		}
	}
	if factor == 1 {
		return spans, nil
	}
	out := make([]BezierSpan, 0, len(spans)*factor)
	for _, span := range spans {
		rest := span
		for remaining := factor; remaining > 1; remaining-- {
			t := big.NewRat(1, int64(remaining))
			if err := work.Step(refineSplitCharge(rest, t)); err != nil {
				return nil, err
			}
			u, v := make([]*big.Rat, len(rest)), make([]*big.Rat, len(rest))
			for i, point := range rest {
				u[i], v[i] = point.U, point.V
			}
			leftU, rightU := BernsteinSplit(u, t)
			leftV, rightV := BernsteinSplit(v, t)
			left, right := make(BezierSpan, len(rest)), make(BezierSpan, len(rest))
			for i := range rest {
				left[i] = RatPoint{U: leftU[i], V: leftV[i]}
				right[i] = RatPoint{U: rightU[i], V: rightV[i]}
			}
			out = append(out, left)
			rest = right
		}
		out = append(out, rest)
	}
	return out, nil
}

// refineSplitCharge bounds one rational de Casteljau split before it runs.
// Each blend uses two products and one sum per coordinate. The width includes
// every input numerator and denominator plus the split parameter, multiplied
// by the degree to cover the growth across successive interpolation levels.
func refineSplitCharge(span BezierSpan, t *big.Rat) uint64 {
	n := uint64(len(span))
	bits := t.Denom().BitLen() + t.Num().BitLen()
	for _, point := range span {
		bits = max(bits, point.U.Num().BitLen()+point.U.Denom().BitLen(),
			point.V.Num().BitLen()+point.V.Denom().BitLen())
	}
	return CostMul(CostMul(6, CostMul(n, n-1)), WidthUnits(bits*len(span)))
}
