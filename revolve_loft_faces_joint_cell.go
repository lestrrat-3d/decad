package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

// A joint cell confines one source-side change to a one-sided Bezier sliver
// at an authenticated fit knot. The source polynomial is strictly monotone
// in that sliver for every time in the exact root bracket; the rest of the
// source span has one strict side. Its mapped X hull lies within the finite
// Revolve section. No held vertex enters this certificate.
type revolveLoftFacesJointCell struct {
	sLo, sHi    *big.Rat
	time        loftFaceInterval
	outsideSide int
	sDerivative int
	x           loftFaceInterval
	finite      bool
}

func revolveLoftFacesKnotRootCell(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int,
	atEnd bool, root revolveLoftFacesEndpointRoot) (revolveLoftFacesJointCell, bool) {
	var empty revolveLoftFacesJointCell
	if trim == nil || band < 0 || band+1 >= len(trim.maps) ||
		root.time.lo == nil || root.time.hi == nil {
		return empty, false
	}
	m, b := revolveLoftFacesMeridian(section)
	q := spiralRootPolynomial(trim.maps[band], trim.maps[band+1], span, m, b)
	endpoint := lfZero()
	if atEnd {
		endpoint = lfOne()
	}
	endQ := []*big.Rat{
		spiralBernValue(q[0], endpoint),
		spiralBernValue(q[1], endpoint),
		spiralBernValue(q[2], endpoint),
	}
	loSign, hiSign := spiralBernValue(endQ, root.time.lo).Sign(),
		spiralBernValue(endQ, root.time.hi).Sign()
	if root.time.lo.Cmp(root.time.hi) == 0 {
		if loSign != 0 ||
			spiralBernValue(spiralDeriv(endQ), root.time.lo).Sign() != root.direction {
			return empty, false
		}
	} else if loSign != -root.direction || hiSign != root.direction {
		return empty, false
	}
	startX, endX := lfF(section.start.U), lfF(section.end.U)
	if startX.Cmp(endX) > 0 {
		startX, endX = endX, startX
	}
	for depth := 0; depth <= 12; depth++ {
		width := big.NewRat(1, int64(1<<depth))
		sLo, sHi := lfZero(), width
		if atEnd {
			sLo, sHi = lfSub(lfOne(), width), lfOne()
		}
		box := revolveLoftFacesRestrictBernstein(q, sLo, sHi,
			root.time.lo, root.time.hi)
		var derivative [3][]*big.Rat
		for row := range box {
			derivative[row] = spiralDeriv(box[row])
		}
		direction := 0
		if spiralAllPositive(derivative) {
			direction = 1
		} else if spiralAllPositive(revolveLoftFacesNegateRows(derivative)) {
			direction = -1
		}
		if direction == 0 {
			continue
		}
		fixed := lfZero()
		if !atEnd {
			fixed = lfOne()
		}
		fixedSide := revolveLoftFacesTimePointSign(box, fixed)
		if fixedSide == 0 {
			continue
		}
		if depth > 0 {
			outerLo, outerHi := lfZero(), sLo
			if !atEnd {
				outerLo, outerHi = sHi, lfOne()
			}
			outer := revolveLoftFacesRestrictBernstein(q, outerLo, outerHi,
				root.time.lo, root.time.hi)
			if fixedSide > 0 && !spiralAllPositive(outer) ||
				fixedSide < 0 && !spiralAllPositive(revolveLoftFacesNegateRows(outer)) {
				continue
			}
		}
		x, ok := revolveLoftFacesSpanXRange(trim.maps[band], trim.maps[band+1],
			span, sLo, sHi, root.time.lo, root.time.hi)
		if !ok || root.finite && (x.lo.Cmp(startX) <= 0 || x.hi.Cmp(endX) >= 0) ||
			!root.finite && !(x.hi.Cmp(startX) < 0 || x.lo.Cmp(endX) > 0) {
			continue
		}
		return revolveLoftFacesJointCell{sLo: sLo, sHi: sHi,
			time: root.time, outsideSide: fixedSide,
			sDerivative: direction, x: x, finite: root.finite}, true
	}
	return empty, false
}
