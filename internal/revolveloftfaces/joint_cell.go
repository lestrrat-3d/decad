package revolveloftfaces

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// A joint cell confines one source-side change to a one-sided Bezier sliver
// at an authenticated fit knot. The source polynomial is strictly monotone
// in that sliver for every time in the exact root bracket; the rest of the
// source span has one strict side. Its mapped X hull lies within the finite
// Revolve section. No held vertex enters this certificate.
type RevolveLoftFacesJointCell struct {
	SLo, SHi    *big.Rat
	Time        loftFaceInterval
	OutsideSide int
	SDerivative int
	X           loftFaceInterval
	Finite      bool
}

func RevolveLoftFacesKnotRootCell(trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int,
	atEnd bool, root loftface.EndpointRoot) (RevolveLoftFacesJointCell, bool) {
	return revolveLoftFacesKnotRootCellWith(nil, trim, section, span, band, atEnd, root)
}

func revolveLoftFacesKnotRootCellWith(memo *loftface.MeridianMemo, trim *loftFacesTrimRecord,
	section revolveSectionSpan, span freeform.BezierSpan, band int,
	atEnd bool, root loftface.EndpointRoot) (RevolveLoftFacesJointCell, bool) {
	var empty RevolveLoftFacesJointCell
	if trim == nil || band < 0 || band+1 >= len(trim.Maps) ||
		root.Time.Lo == nil || root.Time.Hi == nil {
		return empty, false
	}
	m, b := RevolveLoftFacesMeridian(section)
	q := memo.QRows(trim.Maps[band], trim.Maps[band+1], span, m, b)
	endpoint := new(big.Rat)
	if atEnd {
		endpoint = big.NewRat(1, 1)
	}
	endQ := []*big.Rat{
		polynomial.BernsteinValue(q[0], endpoint),
		polynomial.BernsteinValue(q[1], endpoint),
		polynomial.BernsteinValue(q[2], endpoint),
	}
	loSign, hiSign := polynomial.BernsteinSign(endQ, root.Time.Lo),
		polynomial.BernsteinSign(endQ, root.Time.Hi)
	if root.Time.Lo.Cmp(root.Time.Hi) == 0 {
		if loSign != 0 ||
			polynomial.BernsteinSign(polynomial.BernsteinDeriv(endQ), root.Time.Lo) != root.Direction {
			return empty, false
		}
	} else if loSign != -root.Direction || hiSign != root.Direction {
		return empty, false
	}
	startX, endX := proofarith.FloatRat(section.Start.U), proofarith.FloatRat(section.End.U)
	if startX.Cmp(endX) > 0 {
		startX, endX = endX, startX
	}
	timeRows := polynomial.BernsteinRectTimeRestrict(q, root.Time.Lo, root.Time.Hi)
	for depth := range 13 {
		width := big.NewRat(1, int64(1<<depth))
		sLo, sHi := new(big.Rat), width
		if atEnd {
			sLo, sHi = proofarith.SubRat(new(big.Rat), big.NewRat(1, 1), width), big.NewRat(1, 1)
		}
		box := polynomial.BernsteinRectSourceRestrict(timeRows, sLo, sHi)

		var derivative [3][]*big.Rat
		for row := range box {
			derivative[row] = polynomial.BernsteinDeriv(box[row])
		}
		direction := 0
		if polynomial.BernsteinRowsPositive(derivative) {
			direction = 1
		} else if polynomial.BernsteinRowsPositive(loftface.NegateRows(derivative)) {
			direction = -1
		}
		if direction == 0 {
			continue
		}
		fixed := new(big.Rat)
		if !atEnd {
			fixed = big.NewRat(1, 1)
		}
		fixedSide := polynomial.BernsteinRowsPointSign(box, fixed)
		if fixedSide == 0 {
			continue
		}
		if depth > 0 {
			outerLo, outerHi := new(big.Rat), sLo
			if !atEnd {
				outerLo, outerHi = sHi, big.NewRat(1, 1)
			}
			outer := polynomial.BernsteinRectSourceRestrict(timeRows, outerLo, outerHi)

			if fixedSide > 0 && !polynomial.BernsteinRowsPositive(outer) ||
				fixedSide < 0 && !polynomial.BernsteinRowsPositive(loftface.NegateRows(outer)) {
				continue
			}
		}
		x, ok := loftface.SpanXRange(trim.Maps[band], trim.Maps[band+1],
			span, sLo, sHi, root.Time.Lo, root.Time.Hi)
		if !ok || root.Finite && (x.Lo.Cmp(startX) <= 0 || x.Hi.Cmp(endX) >= 0) ||
			!root.Finite && x.Hi.Cmp(startX) >= 0 && x.Lo.Cmp(endX) <= 0 {
			continue
		}
		return RevolveLoftFacesJointCell{SLo: sLo, SHi: sHi,
			Time: root.Time, OutsideSide: fixedSide,
			SDerivative: direction, X: x, Finite: root.Finite}, true
	}
	return empty, false
}
