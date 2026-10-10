package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
	"github.com/lestrrat-3d/sketch/geom"
	"github.com/lestrrat-3d/units"
)

// loftFitRootCertificate encloses one ideal radius-r arc tangent to the source
// fit spline and its root circle. Every interval is over rational lifts of the
// recorded floats. The numerical fillet is only a seed for finding this root.
type loftFitRootCertificate struct {
	Param                    proofbound.RatInterval
	FitFootU, FitFootV       proofbound.RatInterval
	CenterU, CenterV         proofbound.RatInterval
	CircleFootU, CircleFootV proofbound.RatInterval
	// SpeedUpper is |dFit/dt| over the entire original recorded fragment.
	SpeedUpper float64
}

type loftFitRootSpan struct {
	start, end *big.Rat
	curve      freeform.BezierSpan
	p, dp      polynomial.RatPoly
	a, b, l    polynomial.RatPoly
	pBern      []*big.Rat
	dpBern     []*big.Rat
}

// certifyLoftFitCircleRoot proves that one ideal contact root is bracketed
// near numericFitT and that no squared-equation root lies between it and the
// original fit corner. The caller must still bound the departure of its held
// ArcSeg from these ideal contacts and audit the complete rewritten section.
func certifyLoftFitCircleRoot(ctx context.Context, fit fitSplineSeg, circle circleSeg, arc arcSeg,
	numericFitT, radius float64, fitArrives bool, offsetSign float64) (loftFitRootCertificate, error) {
	refuse := func(reason string) (loftFitRootCertificate, error) {
		return loftFitRootCertificate{}, fmt.Errorf(`%w: %s`, ErrUnsupported, reason)
	}
	if ctx == nil || !finiteLoftFitValue(numericFitT, radius, offsetSign) || radius <= 0 ||
		(offsetSign != -1 && offsetSign != 1) || len(fit.Fit) < 2 ||
		!betweenLoftFitRange(numericFitT, fit.TStart, fit.TEnd) {
		return refuse("the ideal fit-circle contact has invalid input")
	}
	for _, point := range fit.Fit {
		if !finiteLoftFitValue(point.U, point.V) {
			return refuse("the source fit spline has a non-finite point")
		}
	}
	full := fit
	full.TStart, full.TEnd = 0, 1
	// This conversion charges the source size before it asks Sketch to solve
	// the interpolant. Its result is also the exact curve used in the proof.
	spans, err := splinebezier.FitSplineBezierSpans(full, freeform.NewFreeformWork())
	if err != nil {
		return refuse("the source fit spline has no exact Bezier span chain")
	}
	interp, err := geom.NewFitInterpolant(splinebezier.FitCoords(fit.Fit))
	if err != nil || len(interp.Params) < 2 {
		return refuse("the source fit spline has no finite interpolant")
	}
	if len(spans) != len(interp.Params)-1 {
		return refuse("the source fit spline has no exact Bezier span chain")
	}
	params := make([]*big.Rat, len(interp.Params))
	for i, value := range interp.Params {
		params[i], _ = proofbound.RatOf(value)
		if params[i] == nil {
			return refuse("the source fit spline has a non-finite span parameter")
		}
	}
	total := params[len(params)-1]
	if total.Sign() <= 0 {
		return refuse("the source fit spline has a zero parameter domain")
	}
	toGlobal := func(t float64) *big.Rat {
		return new(big.Rat).Mul(new(big.Rat).SetFloat64(t), total)
	}
	seedGlobal := toGlobal(numericFitT)
	cornerT := fit.TStart
	if fitArrives {
		cornerT = fit.TEnd
	}
	cornerGlobal := toGlobal(cornerT)
	active := -1
	for i := range spans {
		if params[i].Cmp(seedGlobal) < 0 && seedGlobal.Cmp(params[i+1]) < 0 {
			active = i
			break
		}
	}
	if active < 0 {
		return refuse("the proposed fit contact lies on a span boundary")
	}
	cx, okX := proofbound.RatOf(circle.Center.U)
	cy, okY := proofbound.RatOf(circle.Center.V)
	r, okR := proofbound.RatOf(radius)
	rootRadiusMM, unitErr := circle.Radius.In(units.Millimeter)
	rootRadius, okRoot := proofbound.RatOf(rootRadiusMM)
	if !okX || !okY || !okR || !okRoot || unitErr != nil || rootRadius.Sign() <= 0 {
		return refuse("the source circle or fillet radius is not finite")
	}
	circleSense := -1.0
	if circle.TEnd > circle.TStart {
		circleSense = 1
	}
	inside := new(big.Rat).SetFloat64(offsetSign * circleSense)
	rho := new(big.Rat).Sub(rootRadius, new(big.Rat).Mul(inside, r))
	if rho.Sign() <= 0 {
		return refuse("the root circle has no positive offset radius")
	}
	makeSpan := func(i int) (loftFitRootSpan, error) {
		if err := ctx.Err(); err != nil {
			return loftFitRootSpan{}, err
		}
		u, v := freeform.SpanCoordinatePolys(spans[i])
		du, dv := polynomial.RpDeriv(u), polynomial.RpDeriv(v)
		qu := polynomial.RpSub(u, polynomial.RatPoly{cx})
		qv := polynomial.RpSub(v, polynomial.RatPoly{cy})
		q2 := polynomial.RpAdd(polynomial.RpMul(qu, qu), polynomial.RpMul(qv, qv))
		l := polynomial.RpAdd(polynomial.RpMul(du, du), polynomial.RpMul(dv, dv))
		b := polynomial.RpSub(polynomial.RpMul(qv, du), polynomial.RpMul(qu, dv))
		a0 := new(big.Rat).Sub(new(big.Rat).Mul(rho, rho), new(big.Rat).Mul(r, r))
		a := polynomial.RpSub(polynomial.RatPoly{a0}, q2)
		twoR := new(big.Rat).Mul(big.NewRat(2, 1), r)
		br := polynomial.RpScale(b, twoR)
		p := polynomial.RpSub(polynomial.RpMul(polynomial.RpMul(a, a), l), polynomial.RpMul(br, br))
		degree := polynomial.RpDeg(p)
		if degree < 1 || degree > 16 {
			return loftFitRootSpan{}, fmt.Errorf(`%w: the ideal tangent equation has no supported degree`, ErrUnsupported)
		}
		dp := polynomial.RpDeriv(p)
		return loftFitRootSpan{start: params[i], end: params[i+1], curve: spans[i],
			p: p, dp: dp, a: a, b: b, l: l,
			pBern:  freeform.RpToBernstein(p, degree),
			dpBern: freeform.RpToBernstein(dp, degree-1)}, nil
	}
	span, err := makeSpan(active)
	if err != nil {
		return loftFitRootCertificate{}, err
	}
	local := func(global *big.Rat, s loftFitRootSpan) *big.Rat {
		return new(big.Rat).Quo(new(big.Rat).Sub(global, s.start), new(big.Rat).Sub(s.end, s.start))
	}
	seed := local(seedGlobal, span)
	fitMin, fitMax := toGlobal(math.Min(fit.TStart, fit.TEnd)), toGlobal(math.Max(fit.TStart, fit.TEnd))
	allowedLo := new(big.Rat).Set(fitMin)
	allowedHi := new(big.Rat).Set(fitMax)
	if allowedLo.Cmp(span.start) < 0 {
		allowedLo.Set(span.start)
	}
	if allowedHi.Cmp(span.end) > 0 {
		allowedHi.Set(span.end)
	}
	allowedLo, allowedHi = local(allowedLo, span), local(allowedHi, span)
	step := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 42))
	var lo, hi *big.Rat
	for i := 0; i < 42; i++ {
		if err := ctx.Err(); err != nil {
			return loftFitRootCertificate{}, err
		}
		lo = new(big.Rat).Sub(seed, step)
		hi = new(big.Rat).Add(seed, step)
		if lo.Cmp(allowedLo) <= 0 || hi.Cmp(allowedHi) >= 0 {
			break
		}
		lv, hv := polynomial.RpEval(span.p, lo), polynomial.RpEval(span.p, hi)
		if lv.Sign() != 0 && hv.Sign() != 0 && lv.Sign() != hv.Sign() {
			break
		}
		step.Mul(step, big.NewRat(2, 1))
	}
	if lo == nil || lo.Cmp(allowedLo) <= 0 || hi.Cmp(allowedHi) >= 0 {
		return refuse("the ideal tangent root has no interior exact-sign bracket")
	}
	lv, hv := polynomial.RpEval(span.p, lo), polynomial.RpEval(span.p, hi)
	if lv.Sign() == 0 || hv.Sign() == 0 || lv.Sign() == hv.Sign() {
		return refuse("the ideal tangent root has no opposite exact endpoint signs")
	}
	// Exact bisection tightens the ideal root's rational bracket. A derivative
	// hull of one strict sign then proves there is exactly one P-root inside.
	for i := 0; i < 64; i++ {
		if err := ctx.Err(); err != nil {
			return loftFitRootCertificate{}, err
		}
		mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
		mv := polynomial.RpEval(span.p, mid)
		if mv.Sign() == 0 {
			// The exact rational root still receives a nonzero bracket, so
			// interval restrictions and the nearer-root audit keep one shape.
			break
		}
		if mv.Sign() == lv.Sign() {
			lo, lv = mid, mv
		} else {
			hi, hv = mid, mv
		}
	}
	derivative := loftFitPolynomialHull(span.dpBern, lo, hi)
	if derivative.Lo.Sign() <= 0 && derivative.Hi.Sign() >= 0 {
		return refuse("the ideal tangent root has no one-sign derivative proof")
	}
	if err := loftFitExcludeNearerRoots(ctx, spans, params, cornerGlobal, span, lo, hi,
		makeSpan, local); err != nil {
		return loftFitRootCertificate{}, err
	}
	aIv := loftFitPolyInterval(span.a, lo, hi)
	bIv := loftFitPolyInterval(span.b, lo, hi)
	lIv := loftFitPolyInterval(span.l, lo, hi)
	if lIv.Lo.Sign() <= 0 || aIv.Lo.Sign() <= 0 && aIv.Hi.Sign() >= 0 ||
		bIv.Lo.Sign() <= 0 && bIv.Hi.Sign() >= 0 {
		return refuse("the ideal tangent root has no resolved speed or unsquared branch")
	}
	travelSign := math.Copysign(1, fit.TEnd-fit.TStart)
	if aIv.Lo.Sign() != int(offsetSign*travelSign)*bIv.Lo.Sign() {
		return refuse("the squared tangent equation selected the opposite offset branch")
	}
	speedUpper, ok := loftFitFragmentSpeedUpper(spans, params, total, fit)
	if !ok {
		return refuse("the original fit fragment has no finite speed bound")
	}
	cert, err := loftFitEncloseRoot(span, lo, hi, total, cx, cy, rootRadius, rho, r,
		new(big.Rat).SetFloat64(offsetSign), new(big.Rat).SetFloat64(travelSign))
	if err != nil {
		return loftFitRootCertificate{}, err
	}
	cert.SpeedUpper = speedUpper
	if _, _, err := loftFitArcDeparture(cert, arc, fitArrives, radius); err != nil {
		return loftFitRootCertificate{}, err
	}
	if err := certifyIdealFitArcSeparation(ctx, spans, params, total, fit, fitArrives,
		arc, cert, r, active, lo, hi); err != nil {
		return loftFitRootCertificate{}, err
	}
	return cert, nil
}

func loftFitPolynomialHull(coeffs []*big.Rat, lo, hi *big.Rat) proofbound.RatInterval {
	a, b := freeform.BernsteinHull(freeform.BernsteinRestrict(coeffs, lo, hi))
	return proofbound.Interval(a, b)
}

func loftFitPolyInterval(p polynomial.RatPoly, lo, hi *big.Rat) proofbound.RatInterval {
	if polynomial.RpDeg(p) < 0 {
		return proofbound.PointInterval(new(big.Rat))
	}
	return loftFitPolynomialHull(freeform.RpToBernstein(p, polynomial.RpDeg(p)), lo, hi)
}

func loftFitExcludeNearerRoots(ctx context.Context, spans []freeform.BezierSpan, params []*big.Rat,
	cornerGlobal *big.Rat, active loftFitRootSpan, lo, hi *big.Rat,
	makeSpan func(int) (loftFitRootSpan, error), local func(*big.Rat, loftFitRootSpan) *big.Rat) error {
	nearLocal := lo
	activeIndex := -1
	for i := range spans {
		if params[i] == active.start || params[i].Cmp(active.start) == 0 {
			activeIndex = i
			break
		}
	}
	if activeIndex < 0 {
		return fmt.Errorf(`%w: the ideal fit contact has no active span`, ErrUnsupported)
	}
	if cornerGlobal.Cmp(new(big.Rat).Add(active.start,
		new(big.Rat).Mul(lo, new(big.Rat).Sub(active.end, active.start)))) > 0 {
		nearLocal = hi
	}
	nearGlobal := new(big.Rat).Add(active.start,
		new(big.Rat).Mul(nearLocal, new(big.Rat).Sub(active.end, active.start)))
	rangeLo, rangeHi := cornerGlobal, nearGlobal
	if rangeLo.Cmp(rangeHi) > 0 {
		rangeLo, rangeHi = rangeHi, rangeLo
	}
	for i := range spans {
		start := proofbound.RatMax(rangeLo, params[i])
		end := proofbound.RatMin(rangeHi, params[i+1])
		if start.Cmp(end) >= 0 {
			continue
		}
		segment := active
		if i != activeIndex {
			var err error
			segment, err = makeSpan(i)
			if err != nil {
				return err
			}
		}
		if !loftFitExcludePolynomialRoots(ctx, segment,
			local(start, segment), local(end, segment), 0, new(int)) {
			if err := ctx.Err(); err != nil {
				return err
			}
			return fmt.Errorf(`%w: a closer fit-circle tangent root cannot be excluded`, ErrUnsupported)
		}
	}
	return nil
}

// loftFitFragmentSpeedUpper bounds |dFit/dt| over the entire original
// fragment, including portions outside the one span holding the fillet root.
func loftFitFragmentSpeedUpper(spans []freeform.BezierSpan, params []*big.Rat,
	total *big.Rat, fit fitSplineSeg) (float64, bool) {
	start := new(big.Rat).Mul(new(big.Rat).SetFloat64(math.Min(fit.TStart, fit.TEnd)), total)
	end := new(big.Rat).Mul(new(big.Rat).SetFloat64(math.Max(fit.TStart, fit.TEnd)), total)
	upper := 0.0
	for i, span := range spans {
		pieceStart := proofbound.RatMax(start, params[i])
		pieceEnd := proofbound.RatMin(end, params[i+1])
		if pieceStart.Cmp(pieceEnd) >= 0 {
			continue
		}
		width := new(big.Rat).Sub(params[i+1], params[i])
		if width.Sign() <= 0 {
			return 0, false
		}
		localStart := new(big.Rat).Quo(new(big.Rat).Sub(pieceStart, params[i]), width)
		localEnd := new(big.Rat).Quo(new(big.Rat).Sub(pieceEnd, params[i]), width)
		u, v := freeform.SpanCoordinatePolys(span)
		du := loftFitPolyInterval(polynomial.RpDeriv(u), localStart, localEnd)
		dv := loftFitPolyInterval(polynomial.RpDeriv(v), localStart, localEnd)
		speedSq := new(big.Rat).Add(
			new(big.Rat).Mul(proofbound.IntervalAbsUpper(du), proofbound.IntervalAbsUpper(du)),
			new(big.Rat).Mul(proofbound.IntervalAbsUpper(dv), proofbound.IntervalAbsUpper(dv)),
		)
		scaled := new(big.Rat).Mul(speedSq, new(big.Rat).Mul(
			new(big.Rat).Quo(total, width), new(big.Rat).Quo(total, width)))
		value := proofbound.RatSqrtUp(scaled)
		if !finiteLoftFitValue(value) {
			return 0, false
		}
		upper = math.Max(upper, value)
	}
	return upper, upper > 0
}

func loftFitExcludePolynomialRoots(ctx context.Context, span loftFitRootSpan,
	lo, hi *big.Rat, depth int, nodes *int) bool {
	if ctx.Err() != nil || depth > 22 || *nodes >= 512 {
		return false
	}
	*nodes++
	hull := loftFitPolynomialHull(span.pBern, lo, hi)
	if hull.Lo.Sign() > 0 || hull.Hi.Sign() < 0 {
		return true
	}
	derivative := loftFitPolynomialHull(span.dpBern, lo, hi)
	if derivative.Lo.Sign() > 0 || derivative.Hi.Sign() < 0 {
		if polynomial.RpEval(span.p, lo).Sign() == polynomial.RpEval(span.p, hi).Sign() &&
			polynomial.RpEval(span.p, lo).Sign() != 0 {
			return true
		}
	}
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	return loftFitExcludePolynomialRoots(ctx, span, lo, mid, depth+1, nodes) &&
		loftFitExcludePolynomialRoots(ctx, span, mid, hi, depth+1, nodes)
}

func loftFitEncloseRoot(span loftFitRootSpan, lo, hi, total, cx, cy, rootRadius, rho, radius,
	offsetSign, travelSign *big.Rat) (loftFitRootCertificate, error) {
	paramOf := func(local *big.Rat) *big.Rat {
		global := new(big.Rat).Add(span.start,
			new(big.Rat).Mul(local, new(big.Rat).Sub(span.end, span.start)))
		return global.Quo(global, total)
	}
	param := proofbound.Interval(paramOf(lo), paramOf(hi))
	u, v := freeform.SpanCoordinatePolys(span.curve)
	fitU, fitV := loftFitPolyInterval(u, lo, hi), loftFitPolyInterval(v, lo, hi)
	du := loftFitPolyInterval(polynomial.RpDeriv(u), lo, hi)
	dv := loftFitPolyInterval(polynomial.RpDeriv(v), lo, hi)
	du = proofbound.IntervalScale(du, travelSign)
	dv = proofbound.IntervalScale(dv, travelSign)
	speedSq := proofbound.IntervalAdd(proofbound.IntervalSquare(du), proofbound.IntervalSquare(dv))
	if speedSq.Lo.Sign() <= 0 {
		return loftFitRootCertificate{}, fmt.Errorf(`%w: the ideal fit tangent speed may vanish`, ErrUnsupported)
	}
	speed, ok := proofbound.IntervalSqrt(speedSq)
	if !ok {
		return loftFitRootCertificate{}, fmt.Errorf(`%w: the ideal fit tangent speed has no finite enclosure`, ErrUnsupported)
	}
	factor := new(big.Rat).Mul(offsetSign, radius)
	normalU, okU := proofbound.IntervalQuo(proofbound.IntervalScale(dv,
		new(big.Rat).Neg(factor)), speed)
	normalV, okV := proofbound.IntervalQuo(proofbound.IntervalScale(du, factor), speed)
	if !okU || !okV {
		return loftFitRootCertificate{}, fmt.Errorf(`%w: the ideal fit tangent normal has no finite enclosure`, ErrUnsupported)
	}
	centerU := proofbound.IntervalAdd(fitU, normalU)
	centerV := proofbound.IntervalAdd(fitV, normalV)
	centerOffsetU := proofbound.IntervalSub(centerU, proofbound.PointInterval(cx))
	centerOffsetV := proofbound.IntervalSub(centerV, proofbound.PointInterval(cy))
	circleFactor := new(big.Rat).Quo(rootRadius, rho)
	circleU := proofbound.IntervalAdd(proofbound.PointInterval(cx),
		proofbound.IntervalScale(centerOffsetU, circleFactor))
	circleV := proofbound.IntervalAdd(proofbound.PointInterval(cy),
		proofbound.IntervalScale(centerOffsetV, circleFactor))
	return loftFitRootCertificate{
		Param: param, FitFootU: fitU, FitFootV: fitV,
		CenterU: centerU, CenterV: centerV,
		CircleFootU: circleU, CircleFootV: circleV,
	}, nil
}
