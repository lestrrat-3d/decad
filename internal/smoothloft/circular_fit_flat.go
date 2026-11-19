package smoothloft

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

type circularInterval struct{ lo, hi *big.Rat }

// circularFlatHalves preserves the three recorded line joins while matching
// the lower and upper halves of their common line to separate source arcs.
func circularFlatHalves(edges [][]twistSpan, fit int) ([][]twistSpan, [][]circularInterval, bool) {
	flat := [3]twistSpan{}
	for i := range flat {
		s := edges[(fit+1+i)%4]
		if len(s) != 1 {
			return nil, nil, false
		}
		flat[i] = s[0]
	}
	fitSpans := edges[fit]
	if len(fitSpans) == 0 || len(fitSpans)%2 != 0 {
		return nil, nil, false
	}
	lo, hi := flat[0][0], flat[2][3]
	if lo.U.Cmp(hi.U) != 0 || lo.V.Cmp(hi.V) >= 0 {
		return nil, nil, false
	}
	for i, span := range flat {
		if span[0].U.Cmp(lo.U) != 0 || span[3].U.Cmp(lo.U) != 0 || span[0].V.Cmp(span[3].V) >= 0 ||
			(i > 0 && !twistSame(flat[i-1][3], span[0])) {
			return nil, nil, false
		}
	}
	mid := freeform.RatPoint{U: lo.U, V: div(add(lo.V, hi.V), rint(2))}
	groups := make([][]twistSpan, 4)
	intervals := make([][]circularInterval, 4)
	appendPart := func(edge int, a, b freeform.RatPoint, base, width *big.Rat) {
		groups[edge] = append(groups[edge], twistLineSpan(a, b))
		intervals[edge] = append(intervals[edge], circularInterval{div(sub(a.V, base), width), div(sub(b.V, base), width)})
	}
	width := sub(mid.V, lo.V)
	for _, span := range flat {
		a, b := span[0], span[3]
		if a.V.Cmp(mid.V) < 0 {
			end := b
			if b.V.Cmp(mid.V) > 0 {
				end = mid
			}
			appendPart(3, a, end, lo.V, width)
		}
		if b.V.Cmp(mid.V) > 0 {
			start := a
			if a.V.Cmp(mid.V) < 0 {
				start = mid
			}
			appendPart(0, start, b, mid.V, width)
		}
	}
	n := len(fitSpans) / 2
	groups[1], groups[2] = fitSpans[:n], fitSpans[n:]
	for edge := 1; edge <= 2; edge++ {
		for i := range groups[edge] {
			intervals[edge] = append(intervals[edge], circularInterval{big.NewRat(int64(i), int64(n)), big.NewRat(int64(i+1), int64(n))})
		}
	}
	return groups, intervals, true
}

// circularFlatQuarter maps the complete flat to one quarter and divides the
// spline's span-index parameter into three equal intervals for the others.
// Every recorded line join and spline span end remains a subpatch boundary.
func circularFlatQuarter(edges [][]twistSpan, fit int, knots []*big.Rat) ([][]twistSpan, [][]circularInterval, bool) {
	_, _, ok := circularFlatHalves(edges, fit)
	if !ok {
		return nil, nil, false
	}
	groups := make([][]twistSpan, 4)
	intervals := make([][]circularInterval, 4)
	loFlat, hiFlat := edges[(fit+1)%4][0][0].V, edges[(fit+3)%4][0][3].V
	for i := 1; i <= 3; i++ {
		span := edges[(fit+i)%4][0]
		groups[0] = append(groups[0], span)
		intervals[0] = append(intervals[0], circularInterval{
			div(sub(span[0].V, loFlat), sub(hiFlat, loFlat)),
			div(sub(span[3].V, loFlat), sub(hiFlat, loFlat)),
		})
	}
	fitSpans := edges[fit]
	n := len(fitSpans)
	if knots != nil && len(knots) != n+1 {
		return nil, nil, false
	}
	breaks := [4]*big.Rat{rint(0), big.NewRat(1, 3), big.NewRat(2, 3), rint(1)}
	if knots != nil && n%3 == 0 {
		breaks[1], breaks[2] = knots[n/3], knots[2*n/3]
	}
	for q := range 3 {
		qlo, qhi := breaks[q], breaks[q+1]
		for i, span := range fitSpans {
			spanLo, spanHi := big.NewRat(int64(i), int64(n)), big.NewRat(int64(i+1), int64(n))
			if knots != nil {
				spanLo, spanHi = knots[i], knots[i+1]
			}
			lo, hi := spanLo, spanHi
			if lo.Cmp(qlo) < 0 {
				lo = qlo
			}
			if hi.Cmp(qhi) > 0 {
				hi = qhi
			}
			if lo.Cmp(hi) >= 0 {
				continue
			}
			a := div(sub(lo, spanLo), sub(spanHi, spanLo))
			b := div(sub(hi, spanLo), sub(spanHi, spanLo))
			x, y := make([]*big.Rat, 4), make([]*big.Rat, 4)
			for j, point := range span {
				x[j], y[j] = point.U, point.V
			}
			x, y = polynomial.BernsteinRestrict(x, a, b), polynomial.BernsteinRestrict(y, a, b)
			var restricted twistSpan
			for j := range restricted {
				restricted[j] = freeform.RatPoint{U: x[j], V: y[j]}
			}
			groups[q+1] = append(groups[q+1], restricted)
			intervals[q+1] = append(intervals[q+1], circularInterval{
				div(sub(lo, qlo), sub(qhi, qlo)), div(sub(hi, qlo), sub(qhi, qlo)),
			})
		}
	}
	return groups, intervals, true
}

// circularEmergingFlat pairs the flat with a point and the complete spline
// with four exact quarter intervals, preserving all authenticated knots.
func circularEmergingFlat(edges [][]twistSpan, fit int, knots []*big.Rat) ([][]twistSpan, [][]circularInterval, bool) {
	if len(edges[fit]) == 0 {
		return nil, nil, false
	}
	for i := 1; i <= 3; i++ {
		line := edges[(fit+i)%4]
		if len(line) != 1 || line[0][0].U.Cmp(line[0][3].U) != 0 ||
			line[0][0].V.Cmp(line[0][3].V) >= 0 ||
			(i > 1 && !twistSame(edges[(fit+i-1)%4][0][3], line[0][0])) {
			return nil, nil, false
		}
	}
	groups := make([][]twistSpan, 5)
	intervals := make([][]circularInterval, 5)
	loFlat, hiFlat := edges[(fit+1)%4][0][0].V, edges[(fit+3)%4][0][3].V
	for i := 1; i <= 3; i++ {
		span := edges[(fit+i)%4][0]
		groups[0] = append(groups[0], span)
		intervals[0] = append(intervals[0], circularInterval{
			div(sub(span[0].V, loFlat), sub(hiFlat, loFlat)),
			div(sub(span[3].V, loFlat), sub(hiFlat, loFlat)),
		})
	}
	spans := edges[fit]
	n := len(spans)
	if knots != nil && len(knots) != n+1 {
		return nil, nil, false
	}
	breaks := [5]*big.Rat{rint(0), big.NewRat(1, 4), big.NewRat(1, 2), big.NewRat(3, 4), rint(1)}
	if knots != nil && n%4 == 0 {
		for i := 1; i < 4; i++ {
			breaks[i] = knots[i*n/4]
		}
	}
	for q := range 4 {
		for i, span := range spans {
			spanLo, spanHi := big.NewRat(int64(i), int64(n)), big.NewRat(int64(i+1), int64(n))
			if knots != nil {
				spanLo, spanHi = knots[i], knots[i+1]
			}
			lo, hi := spanLo, spanHi
			if lo.Cmp(breaks[q]) < 0 {
				lo = breaks[q]
			}
			if hi.Cmp(breaks[q+1]) > 0 {
				hi = breaks[q+1]
			}
			if lo.Cmp(hi) >= 0 {
				continue
			}
			a, b := div(sub(lo, spanLo), sub(spanHi, spanLo)), div(sub(hi, spanLo), sub(spanHi, spanLo))
			x, y := make([]*big.Rat, 4), make([]*big.Rat, 4)
			for j, point := range span {
				x[j], y[j] = point.U, point.V
			}
			x, y = polynomial.BernsteinRestrict(x, a, b), polynomial.BernsteinRestrict(y, a, b)
			var restricted twistSpan
			for j := range restricted {
				restricted[j] = freeform.RatPoint{U: x[j], V: y[j]}
			}
			groups[q+1] = append(groups[q+1], restricted)
			intervals[q+1] = append(intervals[q+1], circularInterval{
				div(sub(lo, breaks[q]), sub(breaks[q+1], breaks[q])),
				div(sub(hi, breaks[q]), sub(breaks[q+1], breaks[q])),
			})
		}
	}
	return groups, intervals, true
}
