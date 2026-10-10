package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// certifyIdealFitArcSeparation excludes a second contact between the retained
// exact fit spline and the ideal tangent connector. The tangent root itself is
// allowed inside a neighborhood where H” has one strict sign. Everywhere
// else, a piece must lie outside the connector's short angular wedge or have
// one strict sign for H=|fit-center|²-radius².
func certifyIdealFitArcSeparation(ctx context.Context, spans []freeform.BezierSpan,
	params []*big.Rat, total *big.Rat, fit fitSplineSeg, fitArrives bool,
	arc arcSeg, cert loftFitRootCertificate, radius *big.Rat,
	active int, rootLo, rootHi *big.Rat) error {
	refuse := func() error {
		return fmt.Errorf(`%w: the ideal loft fit/connector has an unresolved second contact`, ErrUnsupported)
	}
	zoneWidth := new(big.Rat).SetFrac64(1, 4096)
	zoneLo := new(big.Rat).Sub(rootLo, zoneWidth)
	zoneHi := new(big.Rat).Add(rootHi, zoneWidth)
	if zoneLo.Sign() <= 0 || zoneHi.Cmp(big.NewRat(1, 1)) >= 0 || active < 0 || active >= len(spans) {
		return refuse()
	}
	if !loftIdealHSecondDerivativeStrict(spans[active], cert, zoneLo, zoneHi) {
		return refuse()
	}
	farT := fit.TEnd
	if fitArrives {
		farT = fit.TStart
	}
	far := new(big.Rat).Mul(new(big.Rat).SetFloat64(farT), total)
	zoneGlobalLo := new(big.Rat).Add(params[active],
		new(big.Rat).Mul(zoneLo, new(big.Rat).Sub(params[active+1], params[active])))
	zoneGlobalHi := new(big.Rat).Add(params[active],
		new(big.Rat).Mul(zoneHi, new(big.Rat).Sub(params[active+1], params[active])))
	retainedLo, retainedHi := far, zoneGlobalLo
	if far.Cmp(zoneGlobalLo) > 0 {
		retainedLo, retainedHi = zoneGlobalHi, far
	}
	if retainedLo.Cmp(retainedHi) >= 0 {
		return refuse()
	}
	startU, startV, endU, endV := cert.FitFootU, cert.FitFootV, cert.CircleFootU, cert.CircleFootV
	if !fitArrives {
		startU, startV, endU, endV = endU, endV, startU, startV
	}
	if arc.TStart > arc.TEnd {
		startU, startV, endU, endV = endU, endV, startU, startV
	}
	for i, span := range spans {
		pieceLo := proofbound.RatMax(retainedLo, params[i])
		pieceHi := proofbound.RatMin(retainedHi, params[i+1])
		if pieceLo.Cmp(pieceHi) >= 0 {
			continue
		}
		width := new(big.Rat).Sub(params[i+1], params[i])
		lo := new(big.Rat).Quo(new(big.Rat).Sub(pieceLo, params[i]), width)
		hi := new(big.Rat).Quo(new(big.Rat).Sub(pieceHi, params[i]), width)
		u, v := freeform.SpanCoordinatePolys(span)
		cu := loftIntervalMidpoint(cert.CenterU)
		cv := loftIntervalMidpoint(cert.CenterV)
		du := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CenterU, proofbound.PointInterval(cu)))
		dv := proofbound.IntervalAbsUpper(proofbound.IntervalSub(cert.CenterV, proofbound.PointInterval(cv)))
		x := polynomial.RpSub(u, polynomial.RatPoly{cu})
		y := polynomial.RpSub(v, polynomial.RatPoly{cv})
		h := polynomial.RpSub(polynomial.RpAdd(polynomial.RpMul(x, x), polynomial.RpMul(y, y)),
			polynomial.RatPoly{new(big.Rat).Mul(radius, radius)})
		nodes := 0
		var exclude func(*big.Rat, *big.Rat, int) bool
		exclude = func(a, b *big.Rat, depth int) bool {
			if ctx.Err() != nil || nodes >= 2048 || depth > 32 {
				return false
			}
			nodes++
			if loftIdealFitOutsideArc(span, cert.CenterU, cert.CenterV,
				startU, startV, endU, endV, a, b) {
				return true
			}
			xIv, yIv := loftFitPolyInterval(x, a, b), loftFitPolyInterval(y, a, b)
			xMax, yMax := proofbound.IntervalAbsUpper(xIv), proofbound.IntervalAbsUpper(yIv)
			errorUpper := new(big.Rat).Add(
				new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Add(
					new(big.Rat).Mul(xMax, du), new(big.Rat).Mul(yMax, dv))),
				new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
			hIv := loftFitPolyInterval(h, a, b)
			if new(big.Rat).Sub(hIv.Lo, errorUpper).Sign() > 0 ||
				new(big.Rat).Add(hIv.Hi, errorUpper).Sign() < 0 {
				return true
			}
			mid := new(big.Rat).Quo(new(big.Rat).Add(a, b), big.NewRat(2, 1))
			return exclude(a, mid, depth+1) && exclude(mid, b, depth+1)
		}
		if !exclude(lo, hi, 0) {
			return refuse()
		}
	}
	return nil
}

func loftIntervalMidpoint(iv proofbound.RatInterval) *big.Rat {
	return new(big.Rat).Quo(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(2, 1))
}

func loftIdealHSecondDerivativeStrict(span freeform.BezierSpan, cert loftFitRootCertificate,
	lo, hi *big.Rat) bool {
	u, v := freeform.SpanCoordinatePolys(span)
	du, dv := polynomial.RpDeriv(u), polynomial.RpDeriv(v)
	ddu, ddv := polynomial.RpDeriv(du), polynomial.RpDeriv(dv)
	uIv := proofbound.IntervalSub(loftFitPolyInterval(u, lo, hi), cert.CenterU)
	vIv := proofbound.IntervalSub(loftFitPolyInterval(v, lo, hi), cert.CenterV)
	duIv, dvIv := loftFitPolyInterval(du, lo, hi), loftFitPolyInterval(dv, lo, hi)
	dduIv, ddvIv := loftFitPolyInterval(ddu, lo, hi), loftFitPolyInterval(ddv, lo, hi)
	second := proofbound.IntervalAdd(
		proofbound.IntervalAdd(proofbound.IntervalMul(duIv, duIv), proofbound.IntervalMul(dvIv, dvIv)),
		proofbound.IntervalAdd(proofbound.IntervalMul(uIv, dduIv), proofbound.IntervalMul(vIv, ddvIv)))
	return second.Lo.Sign() > 0 || second.Hi.Sign() < 0
}

func loftIdealFitOutsideArc(span freeform.BezierSpan, centerU, centerV,
	startU, startV, endU, endV proofbound.RatInterval, lo, hi *big.Rat) bool {
	u, v := make([]*big.Rat, len(span)), make([]*big.Rat, len(span))
	for i, point := range span {
		u[i], v[i] = point.U, point.V
	}
	uLo, uHi := freeform.BernsteinHull(freeform.BernsteinRestrict(u, lo, hi))
	vLo, vHi := freeform.BernsteinHull(freeform.BernsteinRestrict(v, lo, hi))
	pU := proofbound.IntervalSub(proofbound.Interval(uLo, uHi), centerU)
	pV := proofbound.IntervalSub(proofbound.Interval(vLo, vHi), centerV)
	sU, sV := proofbound.IntervalSub(startU, centerU), proofbound.IntervalSub(startV, centerV)
	eU, eV := proofbound.IntervalSub(endU, centerU), proofbound.IntervalSub(endV, centerV)
	startCross := proofbound.IntervalSub(proofbound.IntervalMul(sU, pV), proofbound.IntervalMul(sV, pU))
	endCross := proofbound.IntervalSub(proofbound.IntervalMul(pU, eV), proofbound.IntervalMul(pV, eU))
	return startCross.Hi.Sign() < 0 || endCross.Hi.Sign() < 0
}
