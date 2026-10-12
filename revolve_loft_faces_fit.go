package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/sketch"
)

const (
	revolveLoftFacesInside  = -1
	revolveLoftFacesCrossed = 0
	revolveLoftFacesOutside = 1
)

// A signed side applies to the entire true mapped Bezier span in every band.
// For a crossing, Q_s has a strict sign and the root's X movement has a
// strict positive Bernstein bound. These are exact rational predicates.
func revolveLoftFacesFitSide(trim *loftFacesTrimRecord, m, b *big.Rat,
	segmentIndex int, segment fitSplineSeg) (*revolveLoftFacesCrossing, int, bool) {
	if _, ok := trim.source.source.entities[segmentIndex].(*sketch.FitSpline); !ok ||
		!((segment.TStart == 0 && segment.TEnd == 1) ||
			(segment.TStart == 1 && segment.TEnd == 0)) {
		return nil, 0, false
	}
	work := freeform.NewFreeformWork()
	work.RaiseReconstructionLimit(freeform.LoftReconstructionWorkLimit)
	spans, err := splinebezier.FitSplineBezierSpans(segment, work)
	if err != nil || len(spans) == 0 || len(spans) > 128 {
		return nil, 0, false
	}
	sides := make([]int, len(spans))
	var crossing *revolveLoftFacesCrossing
	for index, span := range spans {
		var sign int
		seenBand := false
		for band := 0; band+1 < len(trim.maps); band++ {
			q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
			kind, signed := revolveLoftFacesClassifyRows(q)
			if kind == 2 || seenBand && kind != sign {
				return nil, 0, false
			}
			sign = kind
			seenBand = true
			if kind == revolveLoftFacesCrossed &&
				!spiralAllPositive(spiralRootXOrder(trim.maps[band], trim.maps[band+1], span, signed)) {
				return nil, 0, false
			}
		}
		sides[index] = sign
		if sign == revolveLoftFacesCrossed {
			if crossing != nil {
				return nil, 0, false
			}
			crossing = &revolveLoftFacesCrossing{segment: segmentIndex, span: span,
				index: index, caps: make([]spiralRootCrossing, len(trim.maps)),
				x: make([]loftFaceInterval, len(trim.maps))}
		}
	}
	if crossing == nil {
		for _, side := range sides {
			if side != sides[0] || side == 0 {
				return nil, 0, false
			}
		}
		return nil, sides[0], true
	}
	q := spiralRootPolynomial(trim.maps[0], trim.maps[1], crossing.span, m, b)
	before, after := revolveLoftFacesInside, revolveLoftFacesOutside
	if q[0][0].Sign() > 0 {
		before, after = after, before
	}
	for i, side := range sides {
		switch {
		case i < crossing.index && side != before:
			return nil, 0, false
		case i > crossing.index && side != after:
			return nil, 0, false
		}
	}
	for capIndex, mapped := range trim.maps {
		capQ := spiralRootPolynomial(mapped, mapped, crossing.span, m, b)
		kind, signedCap := revolveLoftFacesClassifyRows(capQ)
		if kind != revolveLoftFacesCrossed {
			return nil, 0, false
		}
		root := spiralIsolateRoot(signedCap[0])
		root.span = crossing.index
		direction := revolveLoftFacesCapXDirection(mapped, crossing.span)
		if direction == 2 {
			return nil, 0, false
		}
		crossing.caps[capIndex] = root
		crossing.x[capIndex] = revolveLoftFacesCapXRange(mapped, crossing.span, root, direction)
	}
	for i := 0; i+1 < len(crossing.x); i++ {
		if crossing.x[i].hi.Cmp(crossing.x[i+1].lo) >= 0 {
			return nil, 0, false
		}
	}
	if segment.TStart == 1 {
		before, after = after, before
	}
	crossing.first, crossing.last = before, after
	return crossing, 0, true
}

func revolveLoftFacesClassifyRows(q [3][]*big.Rat) (int, [3][]*big.Rat) {
	if spiralAllPositive(q) {
		return revolveLoftFacesOutside, q
	}
	negative := revolveLoftFacesNegateRows(q)
	if spiralAllPositive(negative) {
		return revolveLoftFacesInside, negative
	}
	if spiralCrossingSigns(q) {
		return revolveLoftFacesCrossed, q
	}
	if spiralCrossingSigns(negative) {
		return revolveLoftFacesCrossed, negative
	}
	return 2, [3][]*big.Rat{}
}

func revolveLoftFacesNegateRows(q [3][]*big.Rat) [3][]*big.Rat {
	var out [3][]*big.Rat
	for i, row := range q {
		out[i] = revolveLoftFacesNegateRow(row)
	}
	return out
}

func revolveLoftFacesNegateRow(row []*big.Rat) []*big.Rat {
	out := make([]*big.Rat, len(row))
	for i, value := range row {
		out[i] = new(big.Rat).Neg(value)
	}
	return out
}

// A rational cap X map is monotone over this Bezier span if every exact
// derivative-numerator coefficient has one strict sign, or all are zero.
func revolveLoftFacesCapXDirection(mapped loftFaceMap, span freeform.BezierSpan) int {
	g, h := spiralCapBern(mapped, span)
	numerator := spiralSub(spiralMul(spiralDeriv(h[0]), g),
		spiralMul(h[0], spiralDeriv(g)))
	negative, positive, zero := true, true, true
	for _, coefficient := range numerator {
		negative = negative && coefficient.Sign() < 0
		positive = positive && coefficient.Sign() > 0
		zero = zero && coefficient.Sign() == 0
	}
	switch {
	case negative:
		return -1
	case zero:
		return 0
	case positive:
		return 1
	default:
		return 2
	}
}

func revolveLoftFacesCapXRange(mapped loftFaceMap, span freeform.BezierSpan,
	root spiralRootCrossing, direction int) loftFaceInterval {
	left, right := spiralBezierPoint(span, root.lo), spiralBezierPoint(span, root.hi)
	a, z := mapped.at(left[0], left[1])[0], mapped.at(right[0], right[1])[0]
	if direction < 0 {
		return loftFaceInterval{lo: z, hi: a}
	}
	return loftFaceInterval{lo: a, hi: z}
}
